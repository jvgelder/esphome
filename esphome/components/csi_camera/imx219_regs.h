#pragma once
#ifdef USE_CSI_CAMERA

#include <cstdint>

namespace esphome::csi_camera {

struct Imx219Reg {
  uint16_t reg;
  uint8_t val;
};

static constexpr Imx219Reg IMX219_REG_END = {0xFFFF, 0xFF};

static constexpr uint16_t IMX219_REG_CHIP_ID_H   = 0x0000;
static constexpr uint16_t IMX219_REG_CHIP_ID_L   = 0x0001;
static constexpr uint16_t IMX219_REG_MODE_SELECT = 0x0100;
static constexpr uint16_t IMX219_REG_IMAGE_ORIENTATION = 0x0172;
static constexpr uint8_t IMX219_IMAGE_ORIENTATION_HMIRROR = 0x01;
static constexpr uint8_t IMX219_IMAGE_ORIENTATION_VFLIP = 0x02;
static constexpr uint8_t  IMX219_MODE_STANDBY    = 0x00;
static constexpr uint8_t  IMX219_MODE_STREAMING  = 0x01;
static constexpr uint16_t IMX219_CHIP_ID         = 0x0219;

extern const Imx219Reg IMX219_COMMON[];
extern const Imx219Reg IMX219_1080P30[];
extern const Imx219Reg IMX219_720P60[];

}  // namespace esphome::csi_camera

#endif  // USE_CSI_CAMERA
