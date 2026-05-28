#include "image_trans/simulator.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace {

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

image_trans::AppConfig makeConfig() {
    image_trans::AppConfig config;
    config.input_path = "unused.mp4";
    config.output_path = "unused.mkv";
    config.size = 128;
    config.fps = 12.5;
    config.packet_bytes = 300;
    config.packet_hz = 50;
    config.header_bytes = 12;
    config.refresh_interval = 12;
    return config;
}

cv::Mat makeFlatFrame(int size, std::uint8_t value) {
    return cv::Mat(size, size, CV_8UC1, cv::Scalar(value)).clone();
}

cv::Mat makeGradientFrame(int size) {
    cv::Mat frame(size, size, CV_8UC1);
    for (int row = 0; row < size; ++row) {
        for (int col = 0; col < size; ++col) {
            frame.at<std::uint8_t>(row, col) = static_cast<std::uint8_t>((row + col) % 256);
        }
    }
    return frame;
}

void testPacketSerialization() {
    image_trans::Packet packet;
    packet.header.frame_index = 0x11223344U;
    packet.header.packet_index = 2;
    packet.header.packet_count = 4;
    packet.header.flags = image_trans::kFlagJpegFrame;
    packet.header.payload_len = 3;
    packet.payload = {0xAA, 0xBB, 0xCC};
    packet.header.crc16 = image_trans::computeCrc16Ccitt(packet.payload);

    const auto bytes = packet.serialize();
    expect(bytes.size() == image_trans::PacketHeader::kSerializedSize + packet.payload.size(),
           "serialized packet size mismatch");
    expect(bytes[0] == 0x44 && bytes[1] == 0x33 && bytes[2] == 0x22 && bytes[3] == 0x11,
           "frame index must serialize little-endian");
    expect(bytes[4] == 2 && bytes[5] == 4, "packet indexes did not serialize correctly");
    expect(bytes[8] == 3 && bytes[9] == 0, "payload length did not serialize correctly");
}

void testPacketBudgeting() {
    std::vector<std::uint8_t> encoded(600, 0x5A);
    const auto packets = image_trans::packetizeJpeg(
        7U, image_trans::kFlagJpegFrame, encoded, 300U, image_trans::PacketHeader::kSerializedSize);

    expect(packets.size() == 3, "600 bytes should split into 3 packets with a 288-byte payload");
    for (std::size_t index = 0; index < packets.size(); ++index) {
        const auto serialized = packets[index].serialize();
        expect(serialized.size() <= 300U, "packet exceeded 300-byte budget");
        expect(packets[index].header.packet_index == index, "packet indices must stay in order");
        expect(packets[index].header.packet_count == packets.size(), "packet count must be fixed");
    }
}

void testRepeatPath() {
    image_trans::TransmissionSimulator simulator(makeConfig());
    const cv::Mat frame = makeFlatFrame(128, 90);

    const auto first = simulator.processFrame(frame, 0U);
    expect(!first.used_repeat, "first frame should be encoded");
    expect(!first.reconstructed8.empty(), "first frame must reconstruct successfully");

    const auto second = simulator.processFrame(frame, 1U);
    expect(second.used_repeat, "identical flat frame should go through repeat path");
    expect(second.packets.size() == 1, "repeat path should emit one control packet");
    expect((second.packets.front().header.flags & image_trans::kFlagRepeatLastFrame) != 0,
           "repeat packet flag is missing");

    cv::Mat diff;
    cv::absdiff(first.reconstructed8, second.reconstructed8, diff);
    expect(cv::countNonZero(diff) == 0, "repeat reconstruction must match previous frame");
}

void testForcedRefreshInterval() {
    auto config = makeConfig();
    config.refresh_interval = 1;
    image_trans::TransmissionSimulator simulator(config);
    const cv::Mat frame = makeFlatFrame(128, 50);

    const auto first = simulator.processFrame(frame, 0U);
    const auto second = simulator.processFrame(frame, 1U);

    expect(!first.forced_refresh, "first frame should not be counted as a forced refresh");
    expect(second.forced_refresh, "second frame must be forced refresh when interval is 1");
    expect(!second.used_repeat, "forced refresh should bypass repeat mode");
}

void testGray16Expansion() {
    const cv::Mat frame8 = makeGradientFrame(8);
    const cv::Mat frame16 = image_trans::expandToGray16(frame8);

    expect(frame16.type() == CV_16UC1, "expanded frame must be 16-bit grayscale");
    expect(frame16.at<std::uint16_t>(0, 1) == 257U, "8-bit to 16-bit expansion must multiply by 257");
}

std::vector<image_trans::Packet> makeTinyPackets(std::size_t count, std::uint32_t frame_index) {
    std::vector<image_trans::Packet> packets;
    packets.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        image_trans::Packet packet;
        packet.header.frame_index = frame_index;
        packet.header.packet_index = static_cast<std::uint8_t>(index);
        packet.header.packet_count = static_cast<std::uint8_t>(count);
        packet.header.flags = image_trans::kFlagJpegFrame;
        packet.payload = {static_cast<std::uint8_t>(index & 0xFFU)};
        packet.header.payload_len = static_cast<std::uint16_t>(packet.payload.size());
        packet.header.crc16 = image_trans::computeCrc16Ccitt(packet.payload);
        packets.push_back(std::move(packet));
    }
    return packets;
}

void testSenderRateLimiting() {
    image_trans::SenderSimulator sender(makeConfig());
    const auto packets = makeTinyPackets(60, 3U);
    const auto scheduled = sender.schedulePackets(3U, 0.0, packets);

    expect(scheduled.size() == packets.size(), "sender must schedule all packets");
    for (std::size_t index = 1; index < scheduled.size(); ++index) {
        const double delta = scheduled[index].send_time_sec - scheduled[index - 1].send_time_sec;
        expect(delta >= 0.02 - 1e-9, "scheduled packets must respect the 50Hz pacing");
    }

    const auto& stats = sender.stats();
    expect(stats.peak_packet_rate_hz <= 50.0 + 1e-9, "peak packet rate must stay within the 50Hz limit");
    expect(stats.peak_byte_rate_bytes_per_sec <= 15000.0 + 1e-9,
           "peak wire byte rate must stay within the 300B * 50Hz limit");
}

void testSenderReceiverRoundTrip() {
    const auto config = makeConfig();
    image_trans::TransmissionSimulator encoder(config);
    image_trans::SenderSimulator sender(config);
    image_trans::ReceiverSimulator receiver(config);

    const cv::Mat frame = makeGradientFrame(128);
    const auto encoded = encoder.processFrame(frame, 0U);
    const auto scheduled = sender.schedulePackets(0U, 0.0, encoded.packets);

    std::optional<image_trans::ReceiverFrame> received;
    for (const auto& packet : scheduled) {
        if (auto frame_result = receiver.receive(packet)) {
            received = std::move(frame_result);
        }
    }

    expect(received.has_value(), "receiver must reconstruct the encoded frame");
    cv::Mat diff;
    cv::absdiff(encoded.reconstructed8, received->reconstructed8, diff);
    expect(cv::countNonZero(diff) == 0, "receiver reconstruction must match encoder reconstruction");

    const auto repeat = encoder.processFrame(frame, 1U);
    const auto repeat_scheduled = sender.schedulePackets(1U, 0.08, repeat.packets);
    received.reset();
    for (const auto& packet : repeat_scheduled) {
        if (auto frame_result = receiver.receive(packet)) {
            received = std::move(frame_result);
        }
    }

    expect(repeat.used_repeat, "second identical frame should use repeat mode");
    expect(received.has_value(), "receiver must emit a frame for repeat packets");
    cv::absdiff(encoded.reconstructed8, received->reconstructed8, diff);
    expect(cv::countNonZero(diff) == 0, "repeat packet must reproduce the last reconstructed frame");
}

}  // namespace

int main() {
    try {
        testPacketSerialization();
        testPacketBudgeting();
        testRepeatPath();
        testForcedRefreshInterval();
        testGray16Expansion();
        testSenderRateLimiting();
        testSenderReceiverRoundTrip();
        std::cout << "All core tests passed\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }
}
