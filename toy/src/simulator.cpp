#include "image_trans/simulator.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace image_trans {
namespace {

constexpr double kRepeatThresholdRatio = 0.005;
constexpr std::uint8_t kRepeatDiffThreshold = 4;
constexpr std::array<int, 6> kJpegQualities = {60, 45, 35, 25, 18, 12};

[[nodiscard]] bool isPositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool isPositive(int value) { return value > 0; }

void appendLe16(std::vector<std::uint8_t> &bytes, std::uint16_t value) {
  bytes.push_back(static_cast<std::uint8_t>(value & 0xFF));
  bytes.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

void appendLe32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  bytes.push_back(static_cast<std::uint8_t>(value & 0xFF));
  bytes.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
  bytes.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
  bytes.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

std::string formatDouble(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

std::filesystem::path
makeTempDirectory(const std::filesystem::path &requested) {
  if (!requested.empty()) {
    std::filesystem::create_directories(requested);
    return requested;
  }

  const auto timestamp =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const auto pid = static_cast<long long>(::getpid());
  auto path =
      std::filesystem::temp_directory_path() /
      ("image_trans_" + std::to_string(timestamp) + "_" + std::to_string(pid));
  std::filesystem::create_directories(path);
  return path;
}

bool validateConfig(const AppConfig &config, std::string &error) {
  if (config.input_path.empty()) {
    error = "missing --input";
    return false;
  }
  if (config.output_path.empty()) {
    error = "missing --output";
    return false;
  }
  if (!isPositive(config.size)) {
    error = "--size must be positive";
    return false;
  }
  if (!isPositive(config.fps)) {
    error = "--fps must be positive";
    return false;
  }
  if (!isPositive(config.packet_bytes)) {
    error = "--packet-bytes must be positive";
    return false;
  }
  if (!isPositive(config.packet_hz)) {
    error = "--packet-hz must be positive";
    return false;
  }
  if (config.header_bytes != static_cast<int>(PacketHeader::kSerializedSize)) {
    error = "--header-bytes must remain 12 in this implementation";
    return false;
  }
  if (config.packet_bytes <= config.header_bytes) {
    error = "--packet-bytes must exceed --header-bytes";
    return false;
  }
  if (config.refresh_interval < 0) {
    error = "--refresh-interval must be >= 0";
    return false;
  }
  const auto packets_per_frame = static_cast<std::size_t>(
      std::floor(static_cast<double>(config.packet_hz) / config.fps + 1e-9));
  if (packets_per_frame < 1) {
    error = "packet rate is too low for the requested output fps";
    return false;
  }
  return true;
}

int runCommand(const std::vector<std::string> &args) {
  if (args.empty()) {
    throw std::invalid_argument("runCommand requires at least one argument");
  }

  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (const auto &arg : args) {
    argv.push_back(const_cast<char *>(arg.c_str()));
  }
  argv.push_back(nullptr);

  pid_t pid = 0;
  const int spawn_result = ::posix_spawnp(&pid, argv.front(), nullptr, nullptr,
                                          argv.data(), environ);
  if (spawn_result != 0) {
    throw std::system_error(spawn_result, std::generic_category(),
                            "failed to spawn external command");
  }

  int status = 0;
  if (::waitpid(pid, &status, 0) < 0) {
    throw std::system_error(errno, std::generic_category(),
                            "failed to wait for external command");
  }
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return -1;
}

std::filesystem::path framePathForIndex(const std::filesystem::path &temp_dir,
                                        std::uint32_t frame_index) {
  std::ostringstream name;
  name << "frame_" << std::setfill('0') << std::setw(6) << frame_index
       << ".png";
  return temp_dir / name.str();
}

void writeFramePng(const std::filesystem::path &temp_dir,
                   std::uint32_t frame_index, const cv::Mat &frame16) {
  const auto path = framePathForIndex(temp_dir, frame_index);
  const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 3};
  if (!cv::imwrite(path.string(), frame16, params)) {
    throw std::runtime_error("failed to write frame image: " + path.string());
  }
}

void encodeOutputVideo(const std::filesystem::path &temp_dir,
                       const std::filesystem::path &output_path, double fps) {
  const auto sequence_pattern = (temp_dir / "frame_%06d.png").string();
  const std::vector<std::string> command = {
      "ffmpeg", "-y",         "-hide_banner",    "-loglevel",
      "error",  "-framerate", formatDouble(fps), "-start_number",
      "0",      "-i",         sequence_pattern,  "-c:v",
      "ffv1",   "-pix_fmt",   "gray16le",        output_path.string(),
  };

  if (runCommand(command) != 0) {
    throw std::runtime_error("ffmpeg failed while assembling the final video");
  }
}

cv::Mat ensureSingleChannel8(const cv::Mat &frame) {
  if (frame.empty()) {
    throw std::runtime_error("cannot process an empty frame");
  }

  cv::Mat converted;
  if (frame.channels() == 1) {
    if (frame.depth() == CV_8U) {
      converted = frame.clone();
    } else {
      frame.convertTo(converted, CV_8U, 1.0 / 256.0);
    }
    return converted;
  }

  cv::Mat bgr;
  if (frame.channels() == 3) {
    bgr = frame;
  } else if (frame.channels() == 4) {
    cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
  } else {
    throw std::runtime_error("unsupported channel count in input frame");
  }

  cv::cvtColor(bgr, converted, cv::COLOR_BGR2GRAY);
  return converted;
}

cv::Mat centerCropToSquare(const cv::Mat &frame) {
  const int size = std::min(frame.cols, frame.rows);
  const int x = (frame.cols - size) / 2;
  const int y = (frame.rows - size) / 2;
  return frame(cv::Rect(x, y, size, size)).clone();
}

cv::Mat decodePacketSequence(const std::vector<Packet> &packets,
                             int output_size) {
  if (packets.empty()) {
    return {};
  }

  const auto expected_frame_index = packets.front().header.frame_index;
  const auto expected_count = packets.front().header.packet_count;
  std::vector<std::uint8_t> encoded;

  for (std::size_t index = 0; index < packets.size(); ++index) {
    const auto &packet = packets[index];
    if (packet.header.frame_index != expected_frame_index ||
        packet.header.packet_count != expected_count ||
        packet.header.packet_index != index) {
      throw std::runtime_error(
          "packet sequence is inconsistent during reconstruction");
    }
    if (packet.header.payload_len != packet.payload.size()) {
      throw std::runtime_error(
          "payload length field does not match the packet payload");
    }
    if (computeCrc16Ccitt(packet.payload) != packet.header.crc16) {
      throw std::runtime_error("payload CRC mismatch");
    }
    encoded.insert(encoded.end(), packet.payload.begin(), packet.payload.end());
  }

  cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
  if (decoded.empty()) {
    throw std::runtime_error("failed to decode reassembled JPEG frame");
  }
  if (decoded.size() != cv::Size(output_size, output_size)) {
    cv::resize(decoded, decoded, cv::Size(output_size, output_size), 0.0, 0.0,
               cv::INTER_LINEAR);
  }
  return decoded;
}

} // namespace

std::vector<std::uint8_t> Packet::serialize() const {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(PacketHeader::kSerializedSize + payload.size());
  appendLe32(bytes, header.frame_index);
  bytes.push_back(header.packet_index);
  bytes.push_back(header.packet_count);
  bytes.push_back(header.flags);
  bytes.push_back(header.reserved);
  appendLe16(bytes, header.payload_len);
  appendLe16(bytes, header.crc16);
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  return bytes;
}

std::uint16_t computeCrc16Ccitt(const std::vector<std::uint8_t> &data) {
  std::uint16_t crc = 0xFFFF;
  for (const std::uint8_t byte : data) {
    crc ^= static_cast<std::uint16_t>(byte) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      const bool set = (crc & 0x8000U) != 0;
      crc <<= 1U;
      if (set) {
        crc ^= 0x1021U;
      }
    }
  }
  return crc;
}

cv::Mat preprocessFrame(const cv::Mat &frame, int size) {
  cv::Mat gray = ensureSingleChannel8(frame);
  cv::Mat square = centerCropToSquare(gray);
  cv::Mat resized;
  cv::resize(square, resized, cv::Size(size, size), 0.0, 0.0, cv::INTER_AREA);
  return resized;
}

cv::Mat expandToGray16(const cv::Mat &frame8) {
  if (frame8.empty() || frame8.type() != CV_8UC1) {
    throw std::runtime_error(
        "expandToGray16 expects a non-empty 8-bit grayscale frame");
  }

  cv::Mat frame16;
  frame8.convertTo(frame16, CV_16UC1, 257.0);
  return frame16;
}

std::vector<Packet> packetizeJpeg(std::uint32_t frame_index, std::uint8_t flags,
                                  const std::vector<std::uint8_t> &encoded,
                                  std::size_t max_packet_bytes,
                                  std::size_t header_bytes) {
  if (header_bytes != PacketHeader::kSerializedSize) {
    throw std::invalid_argument(
        "header_bytes must match PacketHeader::kSerializedSize");
  }
  if (encoded.empty()) {
    throw std::invalid_argument("cannot packetize an empty JPEG frame");
  }
  if (max_packet_bytes <= header_bytes) {
    throw std::invalid_argument("packet size must exceed header size");
  }

  const std::size_t payload_limit = max_packet_bytes - header_bytes;
  const std::size_t packet_count =
      (encoded.size() + payload_limit - 1U) / payload_limit;
  if (packet_count == 0 || packet_count > 255) {
    throw std::invalid_argument(
        "encoded frame needs an unsupported number of packets");
  }

  std::vector<Packet> packets;
  packets.reserve(packet_count);
  for (std::size_t packet_index = 0; packet_index < packet_count;
       ++packet_index) {
    const std::size_t offset = packet_index * payload_limit;
    const std::size_t remaining = encoded.size() - offset;
    const std::size_t payload_size = std::min(payload_limit, remaining);

    Packet packet;
    packet.payload.assign(
        encoded.begin() + static_cast<std::ptrdiff_t>(offset),
        encoded.begin() + static_cast<std::ptrdiff_t>(offset + payload_size));
    packet.header.frame_index = frame_index;
    packet.header.packet_index = static_cast<std::uint8_t>(packet_index);
    packet.header.packet_count = static_cast<std::uint8_t>(packet_count);
    packet.header.flags = flags;
    packet.header.payload_len =
        static_cast<std::uint16_t>(packet.payload.size());
    packet.header.crc16 = computeCrc16Ccitt(packet.payload);
    packets.push_back(std::move(packet));
  }
  return packets;
}

std::vector<Packet> makeRepeatPacket(std::uint32_t frame_index,
                                     std::uint8_t flags,
                                     std::size_t header_bytes) {
  if (header_bytes != PacketHeader::kSerializedSize) {
    throw std::invalid_argument(
        "header_bytes must match PacketHeader::kSerializedSize");
  }

  Packet packet;
  packet.header.frame_index = frame_index;
  packet.header.packet_index = 0;
  packet.header.packet_count = 1;
  packet.header.flags = static_cast<std::uint8_t>(flags | kFlagRepeatLastFrame);
  packet.header.payload_len = 0;
  packet.header.crc16 = computeCrc16Ccitt(packet.payload);
  return {std::move(packet)};
}

TransmissionRateMonitor::TransmissionRateMonitor(
    std::size_t max_packets_per_sec, std::size_t max_bytes_per_sec)
    : max_packets_per_sec_(max_packets_per_sec),
      max_bytes_per_sec_(max_bytes_per_sec) {}

bool TransmissionRateMonitor::canSendNow(double timestamp_sec,
                                         std::size_t packet_bytes) {
  prune(timestamp_sec);
  return window_.size() + 1U <= max_packets_per_sec_ &&
         window_bytes_ + packet_bytes <= max_bytes_per_sec_;
}

RateSample TransmissionRateMonitor::record(double timestamp_sec,
                                           std::size_t packet_bytes) {
  prune(timestamp_sec);
  window_.emplace_back(timestamp_sec, packet_bytes);
  window_bytes_ += packet_bytes;
  return sample(timestamp_sec);
}

void TransmissionRateMonitor::prune(double timestamp_sec) {
  constexpr double kWindowSec = 1.0;
  constexpr double kEpsilon = 1e-9;
  while (!window_.empty() &&
         window_.front().first <= timestamp_sec - kWindowSec + kEpsilon) {
    window_bytes_ -= window_.front().second;
    window_.pop_front();
  }
}

RateSample TransmissionRateMonitor::sample(double timestamp_sec) const {
  RateSample sample;
  sample.timestamp_sec = timestamp_sec;
  sample.packets_in_window = window_.size();
  sample.bytes_in_window = window_bytes_;
  sample.packet_rate_hz = static_cast<double>(sample.packets_in_window);
  sample.byte_rate_bytes_per_sec = static_cast<double>(sample.bytes_in_window);
  return sample;
}

SenderSimulator::SenderSimulator(AppConfig config)
    : config_(std::move(config)),
      packet_period_sec_(1.0 / static_cast<double>(config_.packet_hz)),
      rate_monitor_(static_cast<std::size_t>(config_.packet_hz),
                    static_cast<std::size_t>(config_.packet_hz) *
                        static_cast<std::size_t>(config_.packet_bytes)) {}

std::vector<ScheduledPacket>
SenderSimulator::schedulePackets(std::uint32_t frame_index,
                                 double frame_ready_time_sec,
                                 const std::vector<Packet> &packets) {
  std::vector<ScheduledPacket> scheduled;
  scheduled.reserve(packets.size());

  double candidate_time = std::max(frame_ready_time_sec, next_send_time_sec_);
  for (std::size_t packet_index = 0; packet_index < packets.size();
       ++packet_index) {
    const auto &packet = packets[packet_index];
    const std::size_t wire_bytes = packet.serialize().size();
    const double earliest_time = candidate_time;

    while (!rate_monitor_.canSendNow(candidate_time, wire_bytes)) {
      candidate_time += packet_period_sec_;
    }
    if (candidate_time > earliest_time + 1e-9) {
      ++stats_.delayed_packets;
    }

    ScheduledPacket scheduled_packet;
    scheduled_packet.packet = packet;
    scheduled_packet.packet.header.frame_index = frame_index;
    scheduled_packet.send_time_sec = candidate_time;
    scheduled_packet.receive_time_sec = candidate_time;
    scheduled_packet.wire_bytes = wire_bytes;
    scheduled.push_back(std::move(scheduled_packet));

    const RateSample rate_sample =
        rate_monitor_.record(candidate_time, wire_bytes);
    stats_.peak_packet_rate_hz =
        std::max(stats_.peak_packet_rate_hz, rate_sample.packet_rate_hz);
    stats_.peak_byte_rate_bytes_per_sec =
        std::max(stats_.peak_byte_rate_bytes_per_sec,
                 rate_sample.byte_rate_bytes_per_sec);
    stats_.total_packets += 1U;
    stats_.total_wire_bytes += wire_bytes;
    stats_.last_send_time_sec = candidate_time;

    next_send_time_sec_ = candidate_time + packet_period_sec_;
    candidate_time = next_send_time_sec_;
  }

  return scheduled;
}

const SenderStats &SenderSimulator::stats() const { return stats_; }

ReceiverSimulator::ReceiverSimulator(AppConfig config)
    : config_(std::move(config)) {}

std::optional<ReceiverFrame>
ReceiverSimulator::receive(const ScheduledPacket &scheduled_packet) {
  const Packet &packet = scheduled_packet.packet;
  if (computeCrc16Ccitt(packet.payload) != packet.header.crc16) {
    throw std::runtime_error("receiver rejected a packet with invalid CRC");
  }
  if (packet.header.payload_len != packet.payload.size()) {
    throw std::runtime_error(
        "receiver rejected a packet with mismatched payload length");
  }

  ReceiverFrame frame;
  frame.frame_index = packet.header.frame_index;
  frame.receive_time_sec = scheduled_packet.receive_time_sec;

  if ((packet.header.flags & kFlagRepeatLastFrame) != 0) {
    if (!has_reconstructed_frame_) {
      last_reconstructed8_ =
          cv::Mat::zeros(config_.size, config_.size, CV_8UC1);
      has_reconstructed_frame_ = true;
    }
    frame.reconstructed8 = last_reconstructed8_.clone();
    current_frame_packets_.clear();
    current_frame_index_.reset();
    return frame;
  }

  if ((packet.header.flags & kFlagJpegFrame) == 0) {
    throw std::runtime_error(
        "receiver got a packet without a supported frame flag");
  }

  if (packet.header.packet_index == 0) {
    current_frame_packets_.clear();
    current_frame_index_ = packet.header.frame_index;
  } else if (!current_frame_index_ ||
             *current_frame_index_ != packet.header.frame_index) {
    throw std::runtime_error("receiver observed an unexpected frame boundary");
  }

  current_frame_packets_.push_back(packet);
  if (current_frame_packets_.size() != packet.header.packet_count) {
    return std::nullopt;
  }

  frame.reconstructed8 = decodeBufferedFrame();
  last_reconstructed8_ = frame.reconstructed8.clone();
  has_reconstructed_frame_ = true;
  current_frame_packets_.clear();
  current_frame_index_.reset();
  return frame;
}

cv::Mat ReceiverSimulator::decodeBufferedFrame() const {
  return decodePacketSequence(current_frame_packets_, config_.size);
}

TransmissionSimulator::TransmissionSimulator(AppConfig config)
    : config_(std::move(config)) {}

FrameDecision TransmissionSimulator::processFrame(const cv::Mat &frame8,
                                                  std::uint32_t frame_index) {
  if (frame8.empty() || frame8.type() != CV_8UC1) {
    throw std::runtime_error(
        "TransmissionSimulator expects 8-bit grayscale frames");
  }

  FrameDecision decision;
  decision.forced_refresh = shouldForceRefresh(frame_index);
  if (decision.forced_refresh) {
    decision.flags =
        static_cast<std::uint8_t>(decision.flags | kFlagForcedRefresh);
    ++stats_.forced_refresh_frames;
  }

  if (has_reconstructed_frame_ && !decision.forced_refresh &&
      shouldRepeat(frame8)) {
    decision.used_repeat = true;
    decision.flags =
        static_cast<std::uint8_t>(decision.flags | kFlagRepeatLastFrame);
    decision.reconstructed8 = last_reconstructed8_.clone();
    decision.packets =
        makeRepeatPacket(frame_index, decision.flags, config_.header_bytes);
    ++stats_.repeated_frames;
  } else {
    std::vector<std::uint8_t> encoded = encodeWithinBudget(frame8);
    if (encoded.empty()) {
      const cv::Mat degraded = degradeFrame(frame8);
      encoded = encodeWithinBudget(degraded);
    }

    if (encoded.empty()) {
      if (!has_reconstructed_frame_) {
        last_reconstructed8_ = cv::Mat::zeros(frame8.size(), CV_8UC1);
        has_reconstructed_frame_ = true;
      }
      decision.used_repeat = true;
      decision.fallback_repeat = true;
      decision.flags =
          static_cast<std::uint8_t>(decision.flags | kFlagRepeatLastFrame);
      decision.reconstructed8 = last_reconstructed8_.clone();
      decision.packets =
          makeRepeatPacket(frame_index, decision.flags, config_.header_bytes);
      force_refresh_next_ = true;
      ++stats_.repeated_frames;
      ++stats_.fallback_repeat_frames;
    } else {
      decision.flags =
          static_cast<std::uint8_t>(decision.flags | kFlagJpegFrame);
      decision.encoded_bytes = encoded.size();
      decision.packets =
          packetizeJpeg(frame_index, decision.flags, encoded,
                        static_cast<std::size_t>(config_.packet_bytes),
                        static_cast<std::size_t>(config_.header_bytes));
      if (decision.packets.size() > maxPacketsPerFrame()) {
        throw std::runtime_error(
            "packetization exceeded the allowed packet budget");
      }
      decision.reconstructed8 = reconstructJpegFrame(decision.packets);
      if (decision.reconstructed8.empty()) {
        throw std::runtime_error("failed to reconstruct an encoded frame");
      }
      last_reconstructed8_ = decision.reconstructed8.clone();
      has_reconstructed_frame_ = true;
      force_refresh_next_ = false;
      ++stats_.encoded_frames;
      stats_.total_jpeg_bytes += encoded.size();
    }
  }

  if (!decision.reconstructed8.empty() && !decision.used_repeat) {
    last_reconstructed8_ = decision.reconstructed8.clone();
    has_reconstructed_frame_ = true;
  }

  ++stats_.frame_count;
  stats_.total_packets += decision.packets.size();
  for (const auto &packet : decision.packets) {
    stats_.total_payload_bytes += packet.payload.size();
  }

  return decision;
}

std::size_t TransmissionSimulator::maxPayloadBytes() const {
  return static_cast<std::size_t>(config_.packet_bytes - config_.header_bytes);
}

std::size_t TransmissionSimulator::maxPacketsPerFrame() const {
  return static_cast<std::size_t>(
      std::floor(static_cast<double>(config_.packet_hz) / config_.fps + 1e-9));
}

const SimulationStats &TransmissionSimulator::stats() const { return stats_; }

bool TransmissionSimulator::shouldRepeat(const cv::Mat &frame8) const {
  cv::Mat diff;
  cv::absdiff(frame8, last_reconstructed8_, diff);

  cv::Mat changed_mask;
  cv::compare(diff, cv::Scalar::all(kRepeatDiffThreshold), changed_mask,
              cv::CMP_GT);
  const double changed = static_cast<double>(cv::countNonZero(changed_mask));
  const double total = static_cast<double>(frame8.total());
  return total > 0.0 && (changed / total) < kRepeatThresholdRatio;
}

bool TransmissionSimulator::shouldForceRefresh(
    std::uint32_t frame_index) const {
  if (force_refresh_next_) {
    return true;
  }
  return config_.refresh_interval > 0 && frame_index != 0 &&
         frame_index % static_cast<std::uint32_t>(config_.refresh_interval) ==
             0;
}

std::vector<std::uint8_t>
TransmissionSimulator::encodeWithinBudget(const cv::Mat &frame8) const {
  const std::size_t max_bytes = maxPayloadBytes() * maxPacketsPerFrame();
  if (max_bytes == 0) {
    return {};
  }

  for (const int quality : kJpegQualities) {
    std::vector<std::uint8_t> encoded;
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
    if (!cv::imencode(".jpg", frame8, encoded, params)) {
      continue;
    }
    if (encoded.size() <= max_bytes) {
      return encoded;
    }
  }
  return {};
}

cv::Mat TransmissionSimulator::degradeFrame(const cv::Mat &frame8) const {
  cv::Mat downsampled;
  cv::resize(frame8, downsampled, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

  cv::Mat upsampled;
  cv::resize(downsampled, upsampled, frame8.size(), 0.0, 0.0, cv::INTER_LINEAR);
  cv::GaussianBlur(upsampled, upsampled, cv::Size(3, 3), 0.0, 0.0);
  return upsampled;
}

cv::Mat TransmissionSimulator::reconstructJpegFrame(
    const std::vector<Packet> &packets) const {
  return decodePacketSequence(packets, config_.size);
}

bool parseArguments(int argc, char **argv, AppConfig &config, bool &show_help,
                    std::string &error) {
  show_help = false;

  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    const auto require_value =
        [&](const char *name) -> std::optional<std::string> {
      if (index + 1 >= argc) {
        error = std::string(name) + " requires a value";
        return std::nullopt;
      }
      ++index;
      return std::string(argv[index]);
    };

    try {
      if (arg == "--help" || arg == "-h") {
        show_help = true;
        return true;
      }
      if (arg == "--input") {
        const auto value = require_value("--input");
        if (!value) {
          return false;
        }
        config.input_path = *value;
      } else if (arg == "--output") {
        const auto value = require_value("--output");
        if (!value) {
          return false;
        }
        config.output_path = *value;
      } else if (arg == "--temp-dir") {
        const auto value = require_value("--temp-dir");
        if (!value) {
          return false;
        }
        config.temp_dir = *value;
      } else if (arg == "--size") {
        const auto value = require_value("--size");
        if (!value) {
          return false;
        }
        config.size = std::stoi(*value);
      } else if (arg == "--fps") {
        const auto value = require_value("--fps");
        if (!value) {
          return false;
        }
        config.fps = std::stod(*value);
      } else if (arg == "--packet-bytes") {
        const auto value = require_value("--packet-bytes");
        if (!value) {
          return false;
        }
        config.packet_bytes = std::stoi(*value);
      } else if (arg == "--packet-hz") {
        const auto value = require_value("--packet-hz");
        if (!value) {
          return false;
        }
        config.packet_hz = std::stoi(*value);
      } else if (arg == "--header-bytes") {
        const auto value = require_value("--header-bytes");
        if (!value) {
          return false;
        }
        config.header_bytes = std::stoi(*value);
      } else if (arg == "--refresh-interval") {
        const auto value = require_value("--refresh-interval");
        if (!value) {
          return false;
        }
        config.refresh_interval = std::stoi(*value);
      } else if (arg == "--keep-temp") {
        config.keep_temp = true;
      } else {
        error = "unknown argument: " + arg;
        return false;
      }
    } catch (const std::exception &exception) {
      error = "failed to parse " + arg + ": " + exception.what();
      return false;
    }
  }

  return validateConfig(config, error);
}

void printUsage(std::ostream &out, const char *program_name) {
  out << "Usage: " << program_name
      << " --input <src> --output <dst.mkv> [options]\n"
      << "Options:\n"
      << "  --size <int>              Output square size, default 128\n"
      << "  --fps <double>            Output frame rate, default 12.5\n"
      << "  --packet-bytes <int>      Max bytes per packet including header, "
         "default 300\n"
      << "  --packet-hz <int>         Max packet frequency, default 50\n"
      << "  --header-bytes <int>      Header size, fixed to 12 in this build\n"
      << "  --refresh-interval <int>  Force a JPEG refresh every N output "
         "frames, default 12\n"
      << "  --temp-dir <path>         Keep intermediate PNG frames in a "
         "specific directory\n"
      << "  --keep-temp               Do not remove intermediate PNG frames\n"
      << "  --help                    Show this message\n";
}

int run(const AppConfig &config, std::ostream &out, std::ostream &err) {
  std::string validation_error;
  if (!validateConfig(config, validation_error)) {
    err << "Invalid configuration: " << validation_error << '\n';
    return 1;
  }

  cv::VideoCapture capture(config.input_path.string());
  if (!capture.isOpened()) {
    err << "Failed to open input video: " << config.input_path << '\n';
    return 1;
  }

  double source_fps = capture.get(cv::CAP_PROP_FPS);
  if (!isPositive(source_fps)) {
    source_fps = config.fps;
  }
  const double source_period = 1.0 / source_fps;
  const double output_period = 1.0 / config.fps;

  if (!config.output_path.parent_path().empty()) {
    std::filesystem::create_directories(config.output_path.parent_path());
  }

  const auto temp_dir = makeTempDirectory(config.temp_dir);
  const bool remove_temp_on_success =
      !config.keep_temp && config.temp_dir.empty();

  TransmissionSimulator simulator(config);
  SenderSimulator sender(config);
  ReceiverSimulator receiver(config);
  cv::Mat raw_frame;
  cv::Mat prepared_frame;
  std::uint32_t output_frame_index = 0;
  std::size_t input_frame_index = 0;
  double next_output_time = 0.0;
  double max_frame_queue_delay_sec = 0.0;
  double max_frame_latency_sec = 0.0;
  bool success = false;

  try {
    while (capture.read(raw_frame)) {
      prepared_frame = preprocessFrame(raw_frame, config.size);
      const double current_time =
          static_cast<double>(input_frame_index) * source_period;
      const double coverage_time = current_time + source_period * 0.5;

      while (coverage_time + 1e-9 >= next_output_time) {
        const double frame_ready_time_sec = next_output_time;
        FrameDecision decision =
            simulator.processFrame(prepared_frame, output_frame_index);
        std::vector<ScheduledPacket> scheduled_packets = sender.schedulePackets(
            output_frame_index, frame_ready_time_sec, decision.packets);
        if (scheduled_packets.empty()) {
          throw std::runtime_error(
              "sender produced no scheduled packets for a frame");
        }

        max_frame_queue_delay_sec = std::max(
            max_frame_queue_delay_sec,
            scheduled_packets.front().send_time_sec - frame_ready_time_sec);

        std::optional<ReceiverFrame> received_frame;
        for (const auto &scheduled_packet : scheduled_packets) {
          if (auto frame = receiver.receive(scheduled_packet)) {
            received_frame = std::move(frame);
          }
        }
        if (!received_frame) {
          throw std::runtime_error("receiver did not reconstruct a frame");
        }
        if (received_frame->frame_index != output_frame_index) {
          throw std::runtime_error(
              "receiver reconstructed an unexpected frame index");
        }

        max_frame_latency_sec =
            std::max(max_frame_latency_sec,
                     received_frame->receive_time_sec - frame_ready_time_sec);
        writeFramePng(temp_dir, output_frame_index,
                      expandToGray16(received_frame->reconstructed8));
        ++output_frame_index;
        next_output_time += output_period;
      }

      ++input_frame_index;
    }

    if (output_frame_index == 0) {
      throw std::runtime_error("input video did not yield any frames");
    }

    encodeOutputVideo(temp_dir, config.output_path, config.fps);
    success = true;
  } catch (const std::exception &exception) {
    err << "Processing failed: " << exception.what() << '\n';
    err << "Intermediate frames kept in: " << temp_dir << '\n';
  }

  if (success) {
    if (!config.keep_temp && remove_temp_on_success) {
      std::error_code cleanup_error;
      std::filesystem::remove_all(temp_dir, cleanup_error);
      if (cleanup_error) {
        err << "Warning: failed to remove temp directory " << temp_dir << ": "
            << cleanup_error.message() << '\n';
      }
    }

    const auto &stats = simulator.stats();
    const auto &sender_stats = sender.stats();
    const double avg_packets =
        stats.frame_count == 0
            ? 0.0
            : static_cast<double>(stats.total_packets) / stats.frame_count;
    const double simulated_tx_duration_sec = sender_stats.last_send_time_sec;
    const double max_wire_rate_bytes_per_sec =
        static_cast<double>(config.packet_hz * config.packet_bytes);
    const double wire_rate_utilization =
        max_wire_rate_bytes_per_sec <= 0.0
            ? 0.0
            : sender_stats.peak_byte_rate_bytes_per_sec /
                  max_wire_rate_bytes_per_sec * 100.0;
    const double packet_rate_utilization =
        config.packet_hz <= 0
            ? 0.0
            : sender_stats.peak_packet_rate_hz /
                  static_cast<double>(config.packet_hz) * 100.0;
    out << "Wrote " << stats.frame_count << " frames to " << config.output_path
        << '\n'
        << "Source fps: " << formatDouble(source_fps)
        << ", output fps: " << formatDouble(config.fps) << '\n'
        << "Encoded frames: " << stats.encoded_frames
        << ", repeated frames: " << stats.repeated_frames
        << ", fallback repeats: " << stats.fallback_repeat_frames << '\n'
        << "Forced refresh frames: " << stats.forced_refresh_frames
        << ", total packets: " << stats.total_packets
        << ", avg packets/frame: " << formatDouble(avg_packets) << '\n'
        << "Simulated tx duration: " << formatDouble(simulated_tx_duration_sec)
        << " s, wire bytes: " << sender_stats.total_wire_bytes
        << ", delayed packets: " << sender_stats.delayed_packets << '\n'
        << "Peak packet rate: "
        << formatDouble(sender_stats.peak_packet_rate_hz) << " pkt/s ("
        << formatDouble(packet_rate_utilization)
        << "% of limit), peak wire rate: "
        << formatDouble(sender_stats.peak_byte_rate_bytes_per_sec) << " B/s ("
        << formatDouble(wire_rate_utilization) << "% of limit)\n"
        << "Max frame queue delay: " << formatDouble(max_frame_queue_delay_sec)
        << " s, max frame transport latency: "
        << formatDouble(max_frame_latency_sec) << " s\n";
    if (config.keep_temp || !config.temp_dir.empty()) {
      out << "Intermediate frames available at: " << temp_dir << '\n';
    }
    return 0;
  }

  return 1;
}

} // namespace image_trans
