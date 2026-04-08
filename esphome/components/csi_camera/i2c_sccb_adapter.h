#pragma once
#ifdef USE_CSI_CAMERA

#include "esphome/components/i2c/i2c.h"

#include <cstddef>
#include <cstdint>
#include <esp_err.h>

#if __has_include(<esp_sccb_io_interface.h>)
#define CSI_CAMERA_HAS_SCCB_IO_INTERFACE 1
#include <esp_sccb_io_interface.h>
#else
#define CSI_CAMERA_HAS_SCCB_IO_INTERFACE 0
#endif

namespace esphome::csi_camera {

#if CSI_CAMERA_HAS_SCCB_IO_INTERFACE

/// Adapter connecting ESPHome I2CDevice with the ESP-IDF SCCB interface.
/// Pattern from DT-art1's camera_sensor PR — implements esp_sccb_io_t directly,
/// so no esp_sccb_new_i2c_io() factory (and no second I2C master bus) is needed.
struct I2CSCCBAdapter : esp_sccb_io_t {
  explicit I2CSCCBAdapter(i2c::I2CDevice *device) {
    transmit_reg_a8v8 = transmit;
    transmit_reg_a16v8 = transmit;
    transmit_reg_a8v16 = transmit;
    transmit_reg_a16v16 = transmit;
    transmit_receive_reg_a8v8 = transmit_receive;
    transmit_receive_reg_a16v8 = transmit_receive;
    transmit_receive_reg_a8v16 = transmit_receive;
    transmit_receive_reg_a16v16 = transmit_receive;
    transmit_v16 = transmit;
    receive_v16 = receive;
    del = del_cb;
    device_ = device;
  }
  static esp_err_t transmit(esp_sccb_io_t *io, const uint8_t *wbuf, size_t wlen, int timeout_ms) {
    (void) timeout_ms;
    return reinterpret_cast<I2CSCCBAdapter *>(io)->device_->write(wbuf, wlen) == i2c::ERROR_OK ? ESP_OK : ESP_FAIL;
  }
  static esp_err_t transmit_receive(esp_sccb_io_t *io, const uint8_t *wbuf, size_t wlen, uint8_t *rbuf,
                                    size_t rlen, int timeout_ms) {
    (void) timeout_ms;
    auto *adapter = reinterpret_cast<I2CSCCBAdapter *>(io);
    return adapter->device_->write(wbuf, wlen) == i2c::ERROR_OK &&
                   adapter->device_->read(rbuf, rlen) == i2c::ERROR_OK
               ? ESP_OK
               : ESP_FAIL;
  }
  static esp_err_t receive(esp_sccb_io_t *io, uint8_t *rbuf, size_t rlen, int timeout_ms) {
    (void) timeout_ms;
    return reinterpret_cast<I2CSCCBAdapter *>(io)->device_->read(rbuf, rlen) == i2c::ERROR_OK ? ESP_OK : ESP_FAIL;
  }
  static esp_err_t del_cb(esp_sccb_io_t *io) {
    (void) io;
    return ESP_OK;
  }
  i2c::I2CDevice *device_;
};

#else

class I2CSCCBAdapter {
 public:
  explicit I2CSCCBAdapter(i2c::I2CDevice *device) { (void) device; }
};

#endif  // CSI_CAMERA_HAS_SCCB_IO_INTERFACE

}  // namespace esphome::csi_camera
#endif
