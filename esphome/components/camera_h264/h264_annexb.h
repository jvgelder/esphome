#pragma once
#ifdef USE_CAMERA_H264

#include <cstddef>
#include <cstdint>

namespace esphome::camera_h264 {

static constexpr uint8_t K_H264_NAL_TYPE_MASK = 0x1F;
static constexpr uint8_t K_H264_NAL_TYPE_IDR = 5;
static constexpr uint8_t K_H264_NAL_TYPE_SPS = 7;
static constexpr uint8_t K_H264_NAL_TYPE_PPS = 8;
static constexpr size_t K_H264_ANNEXB_SHORT_START_CODE_BYTES = 3;
static constexpr size_t K_H264_ANNEXB_LONG_START_CODE_BYTES = 4;
static constexpr size_t K_H264_ANNEXB_MIN_START_CODE_BYTES = K_H264_ANNEXB_SHORT_START_CODE_BYTES;

struct H264AnnexBNal {
  const uint8_t *data{nullptr};
  size_t len{0};

  uint8_t type() const { return this->len == 0 ? 0 : this->data[0] & K_H264_NAL_TYPE_MASK; }
  bool empty() const { return this->data == nullptr || this->len == 0; }
};

inline const uint8_t *find_annexb_start_code(const uint8_t *cursor, const uint8_t *end) {
  if (cursor == nullptr || end == nullptr) {
    return end;
  }
  while (cursor + K_H264_ANNEXB_MIN_START_CODE_BYTES <= end) {
    if (cursor[0] == 0 && cursor[1] == 0 && cursor[2] == 1) {
      return cursor;
    }
    if (cursor + K_H264_ANNEXB_LONG_START_CODE_BYTES <= end && cursor[0] == 0 && cursor[1] == 0 && cursor[2] == 0 &&
        cursor[3] == 1) {
      return cursor;
    }
    ++cursor;
  }
  return end;
}

inline const uint8_t *skip_annexb_start_code(const uint8_t *start_code, const uint8_t *end) {
  if (start_code + K_H264_ANNEXB_SHORT_START_CODE_BYTES <= end && start_code[0] == 0 && start_code[1] == 0 &&
      start_code[2] == 1) {
    return start_code + K_H264_ANNEXB_SHORT_START_CODE_BYTES;
  }
  if (start_code + K_H264_ANNEXB_LONG_START_CODE_BYTES <= end && start_code[0] == 0 && start_code[1] == 0 &&
      start_code[2] == 0 && start_code[3] == 1) {
    return start_code + K_H264_ANNEXB_LONG_START_CODE_BYTES;
  }
  return end;
}

inline bool next_annexb_nal(const uint8_t **cursor, const uint8_t *end, H264AnnexBNal *nal) {
  if (cursor == nullptr || *cursor == nullptr || nal == nullptr) {
    return false;
  }

  const uint8_t *start_code = find_annexb_start_code(*cursor, end);
  if (start_code == end) {
    *cursor = end;
    *nal = {};
    return false;
  }

  const uint8_t *nal_start = skip_annexb_start_code(start_code, end);
  const uint8_t *next_start_code = find_annexb_start_code(nal_start, end);
  *cursor = next_start_code;
  *nal = {nal_start, static_cast<size_t>(next_start_code - nal_start)};
  return !nal->empty();
}

inline bool annexb_contains_nal_type(const uint8_t *data, size_t len, uint8_t wanted_type) {
  if (data == nullptr || len < K_H264_ANNEXB_SHORT_START_CODE_BYTES + 1) {
    return false;
  }
  const uint8_t *cursor = data;
  const uint8_t *end = data + len;
  H264AnnexBNal nal;
  while (next_annexb_nal(&cursor, end, &nal)) {
    if (nal.type() == wanted_type) {
      return true;
    }
  }
  return false;
}

}  // namespace esphome::camera_h264

#endif  // USE_CAMERA_H264
