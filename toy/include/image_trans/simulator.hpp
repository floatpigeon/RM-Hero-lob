#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace image_trans {

struct AppConfig {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::filesystem::path temp_dir;
    int size = 128;
    double fps = 12.5;
    int packet_bytes = 300;
    int packet_hz = 50;
    int header_bytes = 12;
    int refresh_interval = 12;
    bool keep_temp = false;
};

struct PacketHeader {
    static constexpr std::size_t kSerializedSize = 12;

    std::uint32_t frame_index = 0;
    std::uint8_t packet_index = 0;
    std::uint8_t packet_count = 0;
    std::uint8_t flags = 0;
    std::uint8_t reserved = 0;
    std::uint16_t payload_len = 0;
    std::uint16_t crc16 = 0;
};

enum PacketFlags : std::uint8_t {
    kFlagRepeatLastFrame = 0x01,
    kFlagJpegFrame = 0x02,
    kFlagForcedRefresh = 0x04,
};

struct Packet {
    PacketHeader header;
    std::vector<std::uint8_t> payload;

    std::vector<std::uint8_t> serialize() const;
};

struct FrameDecision {
    cv::Mat reconstructed8;
    std::vector<Packet> packets;
    std::uint8_t flags = 0;
    bool used_repeat = false;
    bool forced_refresh = false;
    bool fallback_repeat = false;
    std::size_t encoded_bytes = 0;
};

struct ScheduledPacket {
    Packet packet;
    double send_time_sec = 0.0;
    double receive_time_sec = 0.0;
    std::size_t wire_bytes = 0;
};

struct RateSample {
    double timestamp_sec = 0.0;
    std::size_t packets_in_window = 0;
    std::size_t bytes_in_window = 0;
    double packet_rate_hz = 0.0;
    double byte_rate_bytes_per_sec = 0.0;
};

struct ReceiverFrame {
    std::uint32_t frame_index = 0;
    double receive_time_sec = 0.0;
    cv::Mat reconstructed8;
};

struct SimulationStats {
    std::size_t frame_count = 0;
    std::size_t encoded_frames = 0;
    std::size_t repeated_frames = 0;
    std::size_t fallback_repeat_frames = 0;
    std::size_t forced_refresh_frames = 0;
    std::size_t total_packets = 0;
    std::size_t total_payload_bytes = 0;
    std::size_t total_jpeg_bytes = 0;
};

struct SenderStats {
    std::size_t total_packets = 0;
    std::size_t total_wire_bytes = 0;
    std::size_t delayed_packets = 0;
    double peak_packet_rate_hz = 0.0;
    double peak_byte_rate_bytes_per_sec = 0.0;
    double last_send_time_sec = 0.0;
};

bool parseArguments(int argc, char** argv, AppConfig& config, bool& show_help, std::string& error);
int run(const AppConfig& config, std::ostream& out, std::ostream& err);
void printUsage(std::ostream& out, const char* program_name);

std::uint16_t computeCrc16Ccitt(const std::vector<std::uint8_t>& data);
cv::Mat preprocessFrame(const cv::Mat& frame, int size);
cv::Mat expandToGray16(const cv::Mat& frame8);
std::vector<Packet> packetizeJpeg(std::uint32_t frame_index,
                                  std::uint8_t flags,
                                  const std::vector<std::uint8_t>& encoded,
                                  std::size_t max_packet_bytes,
                                  std::size_t header_bytes);
std::vector<Packet> makeRepeatPacket(std::uint32_t frame_index,
                                     std::uint8_t flags,
                                     std::size_t header_bytes);

class TransmissionRateMonitor {
public:
    TransmissionRateMonitor(std::size_t max_packets_per_sec, std::size_t max_bytes_per_sec);

    bool canSendNow(double timestamp_sec, std::size_t packet_bytes);
    RateSample record(double timestamp_sec, std::size_t packet_bytes);

private:
    std::size_t max_packets_per_sec_;
    std::size_t max_bytes_per_sec_;
    std::deque<std::pair<double, std::size_t>> window_;
    std::size_t window_bytes_ = 0;

    void prune(double timestamp_sec);
    RateSample sample(double timestamp_sec) const;
};

class SenderSimulator {
public:
    explicit SenderSimulator(AppConfig config);

    std::vector<ScheduledPacket> schedulePackets(std::uint32_t frame_index,
                                                 double frame_ready_time_sec,
                                                 const std::vector<Packet>& packets);
    const SenderStats& stats() const;

private:
    AppConfig config_;
    double packet_period_sec_ = 0.0;
    double next_send_time_sec_ = 0.0;
    SenderStats stats_;
    TransmissionRateMonitor rate_monitor_;
};

class ReceiverSimulator {
public:
    explicit ReceiverSimulator(AppConfig config);

    std::optional<ReceiverFrame> receive(const ScheduledPacket& packet);

private:
    cv::Mat decodeBufferedFrame() const;

    AppConfig config_;
    std::vector<Packet> current_frame_packets_;
    std::optional<std::uint32_t> current_frame_index_;
    cv::Mat last_reconstructed8_;
    bool has_reconstructed_frame_ = false;
};

class TransmissionSimulator {
public:
    explicit TransmissionSimulator(AppConfig config);

    FrameDecision processFrame(const cv::Mat& frame8, std::uint32_t frame_index);

    std::size_t maxPayloadBytes() const;
    std::size_t maxPacketsPerFrame() const;
    const SimulationStats& stats() const;

private:
    bool shouldRepeat(const cv::Mat& frame8) const;
    bool shouldForceRefresh(std::uint32_t frame_index) const;
    std::vector<std::uint8_t> encodeWithinBudget(const cv::Mat& frame8) const;
    cv::Mat degradeFrame(const cv::Mat& frame8) const;
    cv::Mat reconstructJpegFrame(const std::vector<Packet>& packets) const;

    AppConfig config_;
    SimulationStats stats_;
    cv::Mat last_reconstructed8_;
    bool has_reconstructed_frame_ = false;
    bool force_refresh_next_ = false;
};

}  // namespace image_trans
