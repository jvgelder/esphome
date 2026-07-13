#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace esphome::camera_h264 {

constexpr size_t kibibytes(size_t value) { return value << 10U; }

template<typename Duration> constexpr uint32_t duration_to_microseconds(Duration duration) {
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
}

template<typename Duration> constexpr uint32_t duration_to_milliseconds(Duration duration) {
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
}

constexpr uint32_t one_second_us() { return duration_to_microseconds(std::chrono::seconds{1}); }
constexpr uint32_t one_second_ms() { return duration_to_milliseconds(std::chrono::seconds{1}); }
constexpr uint32_t one_millisecond_us() { return duration_to_microseconds(std::chrono::milliseconds{1}); }

}  // namespace esphome::camera_h264
