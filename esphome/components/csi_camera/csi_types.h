#pragma once
#ifdef USE_CSI_CAMERA

#include <cstdint>
#include <cstring>
#include <hal/color_types.h>

namespace esphome::csi_camera {

// CSI/MIPI sensor identity. Keep this separate from esp32_camera's DVP sensor
// enums: this component owns MIPI CSI + ISP setup, not the esp32-camera driver.
enum class CsiSensorType : uint8_t {
  OV5647,
  IMX219,
  CUSTOM,
};

inline CsiSensorType csi_sensor_type_from_string(const char *value) {
  if (value == nullptr) {
    return CsiSensorType::CUSTOM;
  }
  if (strcmp(value, "ov5647") == 0) {
    return CsiSensorType::OV5647;
  }
  if (strcmp(value, "imx219") == 0) {
    return CsiSensorType::IMX219;
  }
  return CsiSensorType::CUSTOM;
}

enum class CsiBayerOrder : uint8_t {
  RGGB = 0,
  GRBG,
  GBRG,
  BGGR,
};


inline const char *csi_bayer_order_to_string(CsiBayerOrder order) {
  switch (order) {
    case CsiBayerOrder::RGGB: return "rggb";
    case CsiBayerOrder::GRBG: return "grbg";
    case CsiBayerOrder::GBRG: return "gbrg";
    case CsiBayerOrder::BGGR: return "bggr";
  }
  return "rggb";
}

inline CsiBayerOrder csi_bayer_order_from_string(const char *value) {
  if (value == nullptr) {
    return CsiBayerOrder::RGGB;
  }
  if (strcmp(value, "grbg") == 0) {
    return CsiBayerOrder::GRBG;
  }
  if (strcmp(value, "gbrg") == 0) {
    return CsiBayerOrder::GBRG;
  }
  if (strcmp(value, "bggr") == 0) {
    return CsiBayerOrder::BGGR;
  }
  return CsiBayerOrder::RGGB;
}

inline CsiBayerOrder csi_bayer_order_mirrored(CsiBayerOrder order) {
  switch (order) {
    case CsiBayerOrder::RGGB: return CsiBayerOrder::GRBG;
    case CsiBayerOrder::GRBG: return CsiBayerOrder::RGGB;
    case CsiBayerOrder::GBRG: return CsiBayerOrder::BGGR;
    case CsiBayerOrder::BGGR: return CsiBayerOrder::GBRG;
  }
  return CsiBayerOrder::RGGB;
}

inline CsiBayerOrder csi_bayer_order_flipped(CsiBayerOrder order) {
  switch (order) {
    case CsiBayerOrder::RGGB: return CsiBayerOrder::GBRG;
    case CsiBayerOrder::GRBG: return CsiBayerOrder::BGGR;
    case CsiBayerOrder::GBRG: return CsiBayerOrder::RGGB;
    case CsiBayerOrder::BGGR: return CsiBayerOrder::GRBG;
  }
  return CsiBayerOrder::RGGB;
}

inline CsiBayerOrder csi_bayer_order_after_orientation(CsiBayerOrder order, bool horizontal_mirror,
                                                       bool vertical_flip) {
  if (horizontal_mirror) {
    order = csi_bayer_order_mirrored(order);
  }
  if (vertical_flip) {
    order = csi_bayer_order_flipped(order);
  }
  return order;
}

inline color_raw_element_order_t csi_bayer_order_to_esp(CsiBayerOrder order) {
  switch (order) {
    case CsiBayerOrder::RGGB: return COLOR_RAW_ELEMENT_ORDER_RGGB;
    case CsiBayerOrder::GRBG: return COLOR_RAW_ELEMENT_ORDER_GRBG;
    case CsiBayerOrder::GBRG: return COLOR_RAW_ELEMENT_ORDER_GBRG;
    case CsiBayerOrder::BGGR: return COLOR_RAW_ELEMENT_ORDER_BGGR;
  }
  return COLOR_RAW_ELEMENT_ORDER_RGGB;
}

}  // namespace esphome::csi_camera

#endif  // USE_CSI_CAMERA
