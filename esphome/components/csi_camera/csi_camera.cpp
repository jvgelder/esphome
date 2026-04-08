#include "esphome/core/defines.h"
#ifdef USE_CSI_CAMERA

#include "csi_camera.h"
#include "imx219_regs.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <esp_ldo_regulator.h>
#include <esp_cam_ctlr.h>
#include <esp_cam_ctlr_csi.h>

#if __has_include(<esp_cam_sensor.h>) && __has_include(<esp_cam_sensor_detect.h>) && \
    __has_include(<esp_sccb_io_interface.h>)
#define CSI_CAMERA_HAS_MANAGED_SENSOR_HEADERS 1
#include <esp_cam_sensor.h>
#include <esp_cam_sensor_detect.h>
#else
#define CSI_CAMERA_HAS_MANAGED_SENSOR_HEADERS 0
#endif
#include <driver/isp.h>
#include <driver/isp_ccm.h>
#include <driver/isp_color.h>
#include <driver/isp_demosaic.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_cache.h>
#include <esp_err.h>
#include <cinttypes>
#include <cstring>
#include <cstdio>
#include <ratio>

namespace esphome::csi_camera {

static const char *const TAG = "csi_camera";

static constexpr uint32_t K_OV5647_AEC_CTRL00_REG = 0x3A00;
static constexpr uint32_t K_OV5647_AEC_CTRL00_NIGHT_MODE_MASK = 1U << 2U;

static constexpr uint8_t K_REGISTER_ADDRESS_BYTES = 2;
static constexpr uint8_t K_REGISTER_WRITE_BYTES = 3;
static constexpr uint8_t K_IMX219_I2C_ADDRESS = 0x10;
static constexpr uint16_t K_IMX219_720P_WIDTH = 1280;
static constexpr uint16_t K_IMX219_720P_HEIGHT = 720;
static constexpr uint16_t K_IMX219_1080P_WIDTH = 1920;
static constexpr uint16_t K_IMX219_1080P_HEIGHT = 1080;
static constexpr uint32_t K_SETUP_LDO_DELAY_MS = 200;
static constexpr uint32_t K_HERTZ_PER_MEGAHERTZ = static_cast<uint32_t>(std::mega::num);
static constexpr uint32_t K_ISP_PROCESSOR_CLOCK_MHZ = 80;
static constexpr uint8_t K_DEFAULT_MIPI_LANE_COUNT = 2;
static constexpr uint8_t K_FRAME_DIMENSION_ALIGNMENT = 16;
static constexpr uint8_t K_YUV420_BYTES_NUMERATOR = 3;
static constexpr uint8_t K_YUV420_BYTES_DENOMINATOR = 2;
static constexpr size_t K_FRAME_BUFFER_ALIGNMENT_BYTES = 64;
static constexpr uint32_t K_ISP_PROCESSOR_CLOCK_HZ = K_ISP_PROCESSOR_CLOCK_MHZ * K_HERTZ_PER_MEGAHERTZ;
static constexpr uint8_t K_CSI_CONTROLLER_ID = 0;
static constexpr uint8_t K_CSI_QUEUE_ITEMS = 1;
static constexpr uint8_t K_SENSOR_STREAM_ENABLE = 1;
static constexpr uint32_t K_CAPTURE_TASK_STACK_BYTES = 8192;
static constexpr UBaseType_t K_CAPTURE_TASK_PRIORITY = 5;
static constexpr BaseType_t K_CAPTURE_TASK_CORE = 1;
static constexpr uint32_t K_CAPTURE_QUEUE_WAIT_MS = 1000;
static constexpr uint8_t K_LOW_LATENCY_RECOMMENDED_FRAME_BUFFERS = 3;
static constexpr uint32_t K_INITIAL_FRAME_LOG_COUNT = 5;
static constexpr uint32_t K_FRAME_RATE_LOG_INTERVAL = 300;
static constexpr uint8_t K_FRAME_RATE_LOG_DECIMAL_SCALE = 10;
static constexpr uint32_t K_COLOR_FIXED_POINT_SCALE = 128;
static constexpr uint32_t K_COLOR_MAX_INTEGER = 1;
static constexpr uint32_t K_COLOR_MAX_DECIMAL = 127;
static constexpr uint32_t K_DEMOSAIC_GRAD_RATIO_INTEGER = 2;
static constexpr uint32_t K_DEMOSAIC_GRAD_RATIO_DECIMAL = 5;

#if CSI_CAMERA_HAS_MANAGED_SENSOR_HEADERS

struct IspColorFixedPoint {
  uint32_t integer;
  uint32_t decimal;
};

static IspColorFixedPoint make_isp_color_fixed_point(float value) {
  const uint32_t integer = clamp_at_most(static_cast<uint32_t>(value), K_COLOR_MAX_INTEGER);
  const float fractional = value - static_cast<float>(integer);
  const uint32_t decimal = clamp_at_most(static_cast<uint32_t>(fractional * K_COLOR_FIXED_POINT_SCALE),
                                         K_COLOR_MAX_DECIMAL);
  return {integer, decimal};
}

static void log_sensor_control_result(const char *name, esp_err_t err) {
  if (err == ESP_OK) {
    return;
  }
  ESP_LOGW(TAG, "Sensor control %s failed: %s", name, esp_err_to_name(err));
}

static void apply_managed_sensor_bool_parameter(esp_cam_sensor_device_t *sensor, uint32_t id, const char *name,
                                                bool enabled) {
  int value = enabled ? 1 : 0;
  log_sensor_control_result(name, esp_cam_sensor_set_para_value(sensor, id, &value, sizeof(value)));
}

static void apply_managed_sensor_int_parameter(esp_cam_sensor_device_t *sensor, uint32_t id, const char *name,
                                               int value) {
  log_sensor_control_result(name, esp_cam_sensor_set_para_value(sensor, id, &value, sizeof(value)));
}

static void apply_managed_sensor_test_pattern(esp_cam_sensor_device_t *sensor, bool enabled) {
  int value = enabled ? 1 : 0;
  log_sensor_control_result("test_pattern", esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_TEST_PATTERN, &value));
}

static const char *onoff(bool value) { return value ? "ON" : "OFF"; }

void CsiCamera::set_night_mode(bool enabled) {
  this->night_mode_ = enabled;
  this->night_mode_configured_ = true;
  this->apply_night_mode_();
}

bool CsiCamera::update_managed_sensor_register_bit_(uint32_t reg, uint32_t mask, bool enabled, const char *name) {
  if (this->sensor_ == nullptr) {
    return false;
  }
  auto *sensor = static_cast<esp_cam_sensor_device_t *>(this->sensor_);
  esp_cam_sensor_reg_val_t reg_value = {.regaddr = reg, .value = 0};
  esp_err_t err = esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_G_REG, &reg_value);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "%s read failed: err=%d reg=0x%04" PRIx32, name, static_cast<int>(err), reg);
    return false;
  }

  const uint32_t old_value = reg_value.value;
  if (enabled) {
    reg_value.value |= mask;
  } else {
    reg_value.value &= ~mask;
  }

  if (reg_value.value == old_value) {
    ESP_LOGI(TAG, "%s unchanged: %s", name, onoff(enabled));
    return true;
  }

  err = esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg_value);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "%s write failed: err=%d reg=0x%04" PRIx32 " value=0x%02" PRIx32, name, static_cast<int>(err),
             reg, reg_value.value);
    return false;
  }

  ESP_LOGI(TAG, "%s: %s", name, onoff(enabled));
  return true;
}

void CsiCamera::apply_night_mode_() {
  if (!this->night_mode_configured_) {
    return;
  }
  if (this->sensor_type_ != CsiSensorType::OV5647) {
    ESP_LOGW(TAG, "night_mode is only implemented for OV5647 managed sensors; ignoring for profile=%s",
             this->sensor_profile_.c_str());
    return;
  }
  this->update_managed_sensor_register_bit_(K_OV5647_AEC_CTRL00_REG, K_OV5647_AEC_CTRL00_NIGHT_MODE_MASK,
                                            this->night_mode_, "night_mode");
}

static CsiBayerOrder csi_bayer_order_from_sensor_isp(esp_cam_sensor_bayer_pattern_t pattern) {
  switch (pattern) {
    case ESP_CAM_SENSOR_BAYER_RGGB: return CsiBayerOrder::RGGB;
    case ESP_CAM_SENSOR_BAYER_GRBG: return CsiBayerOrder::GRBG;
    case ESP_CAM_SENSOR_BAYER_GBRG: return CsiBayerOrder::GBRG;
    case ESP_CAM_SENSOR_BAYER_BGGR: return CsiBayerOrder::BGGR;
    case ESP_CAM_SENSOR_BAYER_MONO: break;
  }
  return CsiBayerOrder::GBRG;
}

CsiCameraImage::~CsiCameraImage() {
  if (this->data_ != nullptr && this->return_queue_ != nullptr) {
    void *returned_frame_buffer = this->data_;
    xQueueSendToBack(this->return_queue_, &returned_frame_buffer, portMAX_DELAY);
    this->data_ = nullptr;
  }
}

// ── ISR DMA callbacks ────────────────────────────────────────────────────────

bool IRAM_ATTR CsiCamera::s_dma_start(esp_cam_ctlr_handle_t handle,
                                          esp_cam_ctlr_trans_t *transaction, void *user_data) {
  (void) handle;
  return static_cast<CsiCamera *>(user_data)->dma_start_cb_(transaction);
}
bool IRAM_ATTR CsiCamera::s_dma_complete(esp_cam_ctlr_handle_t handle,
                                           esp_cam_ctlr_trans_t *transaction, void *user_data) {
  (void) handle;
  return static_cast<CsiCamera *>(user_data)->dma_complete_cb_(transaction);
}

bool CsiCamera::dma_start_cb_(esp_cam_ctlr_trans_t *trans) {
  void *next_frame_buffer = nullptr;
  if (xQueueReceiveFromISR(this->consumed_, &next_frame_buffer, nullptr) != pdPASS) {
    if (xQueueReceiveFromISR(this->produced_, &next_frame_buffer, nullptr) != pdPASS) {
      return false;
    }
  }
  trans->buffer = next_frame_buffer;
  trans->buflen = this->fb_size_;
  return true;
}

bool CsiCamera::dma_complete_cb_(esp_cam_ctlr_trans_t *trans) {
  if (xQueueSendFromISR(this->produced_, &trans->buffer, nullptr) != pdPASS) {
    xQueueSendFromISR(this->consumed_, &trans->buffer, nullptr);
  }
  return true;
}


// ── Local IMX219 SCCB/register path ─────────────────────────────────────────

bool CsiCamera::i2c_write_reg16_(uint16_t reg, uint8_t val) {
  uint8_t register_write[K_REGISTER_WRITE_BYTES] = {static_cast<uint8_t>(reg >> 8),
                                                 static_cast<uint8_t>(reg & 0xFF), val};
  return this->write(register_write, sizeof(register_write)) == i2c::ERROR_OK;
}

bool CsiCamera::i2c_read_reg16_(uint16_t reg, uint8_t *val) {  // NOLINT(readability-non-const-parameter)
  uint8_t register_address[K_REGISTER_ADDRESS_BYTES] = {static_cast<uint8_t>(reg >> 8),
                                                    static_cast<uint8_t>(reg & 0xFF)};
  return this->write(register_address, sizeof(register_address)) == i2c::ERROR_OK &&
         this->read(val, sizeof(*val)) == i2c::ERROR_OK;
}

bool CsiCamera::imx219_detect_() {
  this->set_i2c_address(K_IMX219_I2C_ADDRESS);
  uint8_t id_h = 0;
  uint8_t id_l = 0;
  if (!this->i2c_read_reg16_(IMX219_REG_CHIP_ID_H, &id_h) ||
      !this->i2c_read_reg16_(IMX219_REG_CHIP_ID_L, &id_l)) {
    ESP_LOGW(TAG, "IMX219: chip-id read failed at 0x%02x", K_IMX219_I2C_ADDRESS);
    return false;
  }
  const uint16_t id = encode_uint16(id_h, id_l);
  ESP_LOGI(TAG, "IMX219 chip id: 0x%04" PRIx16, id);
  return id == IMX219_CHIP_ID;
}

bool CsiCamera::imx219_apply_table_(const Imx219Reg *table) {
  for (const Imx219Reg *register_entry = table; register_entry->reg != IMX219_REG_END.reg; ++register_entry) {
    if (!this->i2c_write_reg16_(register_entry->reg, register_entry->val)) {
      ESP_LOGE(TAG, "IMX219: write 0x%04" PRIx16 " = 0x%02x failed", register_entry->reg, register_entry->val);
      return false;
    }
  }
  return true;
}

bool CsiCamera::imx219_apply_mode_() {
  if (!this->imx219_apply_table_(IMX219_COMMON)) {
    return false;
  }
  const bool mode_720 = this->sensor_format_.find("1280x720") != std::string::npos ||
                        this->sensor_format_.find("720") != std::string::npos;
  if (mode_720) {
    this->target_width_ = K_IMX219_720P_WIDTH;
    this->target_height_ = K_IMX219_720P_HEIGHT;
    ESP_LOGI(TAG, "IMX219 local mode: 1280x720 RAW10");
    return this->imx219_apply_table_(IMX219_720P60);
  }
  this->target_width_ = K_IMX219_1080P_WIDTH;
  this->target_height_ = K_IMX219_1080P_HEIGHT;
  ESP_LOGI(TAG, "IMX219 local mode: 1920x1080 RAW10");
  return this->imx219_apply_table_(IMX219_1080P30);
}

bool CsiCamera::imx219_apply_orientation_() {
  uint8_t orientation = 0;
  if (this->horizontal_mirror_) {
    orientation |= IMX219_IMAGE_ORIENTATION_HMIRROR;
  }
  if (this->vertical_flip_) {
    orientation |= IMX219_IMAGE_ORIENTATION_VFLIP;
  }
  if (!this->i2c_write_reg16_(IMX219_REG_IMAGE_ORIENTATION, orientation)) {
    ESP_LOGW(TAG, "IMX219: orientation write failed");
    return false;
  }
  if (this->test_pattern_) {
    ESP_LOGW(TAG, "test_pattern is not supported by the local IMX219 register driver");
  }
  return true;
}

// ── Setup ────────────────────────────────────────────────────────────────────

void CsiCamera::setup() {
  ESP_LOGI(TAG, "Initializing CSI camera sensor=%s %" PRIu32 "x%" PRIu32,
           this->sensor_profile_.c_str(), this->target_width_, this->target_height_);
  if (this->power_down_pin_ >= 0) {
    gpio_set_direction(static_cast<gpio_num_t>(this->power_down_pin_), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(this->power_down_pin_), 0);
  }
  this->set_timeout(K_SETUP_LDO_DELAY_MS, [this]() { this->setup_phase1_(); });
}

void CsiCamera::setup_phase1_() {
  // ── LDO for MIPI PHY ─────────────────────────────────────────────────────
  esp_ldo_channel_handle_t ldo = nullptr;
  esp_ldo_channel_config_t ldo_cfg = {.chan_id = this->ldo_chan_,
                                       .voltage_mv = this->ldo_mv_};
  if (esp_ldo_acquire_channel(&ldo_cfg, &ldo) != ESP_OK) {
    ESP_LOGE(TAG, "LDO failed");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "LDO ch%d @ %dmV OK", this->ldo_chan_, this->ldo_mv_);

  // ── Sensor detect / mode setup ────────────────────────────────────────────
  // IMX219 is not currently available in upstream esp_cam_sensor, so use the
  // local zero-copy register path for that profile. Other sensors continue to
  // use esp_cam_sensor's detect/format abstraction.
  esp_cam_sensor_device_t *sensor = nullptr;
  const bool use_local_imx219_driver = (this->sensor_type_ == CsiSensorType::IMX219);
  bool selected_format_needs_isp = true;
  bool selected_format_uses_line_sync = false;
  bool selected_format_is_raw8 = false;
  uint8_t selected_format_lane_count = K_DEFAULT_MIPI_LANE_COUNT;
  CsiBayerOrder sensor_bayer_order = this->bayer_order_;

  if (use_local_imx219_driver) {
    if (!this->imx219_detect_()) {
      ESP_LOGE(TAG, "No IMX219 sensor detected at 0x%02x", K_IMX219_I2C_ADDRESS);
      this->mark_failed();
      return;
    }
    if (!this->imx219_apply_mode_()) {
      ESP_LOGE(TAG, "IMX219 mode setup failed");
      this->mark_failed();
      return;
    }
    if (this->horizontal_mirror_configured_ || this->vertical_flip_configured_) {
      this->imx219_apply_orientation_();
    }
    if (this->wb_mode_configured_ || this->aec_mode_configured_ ||
        this->ae_level_configured_ || this->agc_mode_configured_ || this->sharpness_configured_ ||
        this->denoise_configured_ || this->dead_pixel_correction_configured_ ||
        this->black_level_correction_configured_ || this->lens_shading_correction_configured_ ||
        this->night_mode_configured_) {
      ESP_LOGW(TAG, "sensor tuning controls are only implemented for managed esp_cam_sensor drivers; ignoring for local IMX219");
    }
    ESP_LOGI(TAG, "Detected sensor: imx219-local (profile=%s)", this->sensor_profile_.c_str());
    ESP_LOGI(TAG, "Format set: %s → %" PRIu32 "x%" PRIu32 " lane=%u bitrate=%" PRIu32 "Mbps",
             this->sensor_format_.c_str(), this->target_width_, this->target_height_,
             selected_format_lane_count, this->lane_bit_rate_mbps_);
  } else {
    // Sensor detect via linked detect functions. Each entry has sccb_addr —
    // create an SCCB handle per candidate.
    for (esp_cam_sensor_detect_fn_t *detect_fn = &__esp_cam_sensor_detect_fn_array_start;
         detect_fn < &__esp_cam_sensor_detect_fn_array_end; ++detect_fn) {
      if (detect_fn->port != ESP_CAM_SENSOR_MIPI_CSI) {
        continue;
      }

      this->set_i2c_address(detect_fn->sccb_addr);

      esp_cam_sensor_config_t sensor_config = {};
      sensor_config.sccb_handle = &this->i2c_adapter_;
      sensor_config.reset_pin = GPIO_NUM_NC;
      sensor_config.pwdn_pin = static_cast<gpio_num_t>(this->power_down_pin_);
      sensor_config.sensor_port = ESP_CAM_SENSOR_MIPI_CSI;

      sensor = (*(detect_fn->detect))(&sensor_config);
      if (sensor != nullptr) {
        break;
      }
    }

    if (sensor == nullptr) {
      ESP_LOGE(TAG, "No MIPI sensor detected");
      this->mark_failed();
      return;
    }
    this->sensor_ = sensor;
    ESP_LOGI(TAG, "Detected sensor: %s (profile=%s)", sensor->name, this->sensor_profile_.c_str());

    esp_cam_sensor_format_array_t available_formats = {};
    esp_cam_sensor_query_format(sensor, &available_formats);
    const esp_cam_sensor_format_t *selected_format = nullptr;
    for (size_t format_index = 0; format_index < available_formats.count; format_index++) {
      ESP_LOGI(TAG, "  [%zu] %s", format_index, available_formats.format_array[format_index].name);
      if (this->sensor_format_ == available_formats.format_array[format_index].name) {
        selected_format = &available_formats.format_array[format_index];
      }
    }
    if (selected_format == nullptr) {
      ESP_LOGE(TAG, "Format '%s' not found", this->sensor_format_.c_str());
      ESP_LOGE(TAG, "The sensor was detected, but this mode was not compiled into esp_cam_sensor. "
                    "Enable the Kconfig option for the selected sensor mode or choose another mode.");
      this->mark_failed();
      return;
    }
    if (esp_cam_sensor_set_format(sensor, selected_format) != ESP_OK) {
      ESP_LOGE(TAG, "set_format failed");
      this->mark_failed();
      return;
    }
    this->target_width_ = selected_format->width;
    this->target_height_ = selected_format->height;
    this->lane_bit_rate_mbps_ = selected_format->mipi_info.mipi_clk / K_HERTZ_PER_MEGAHERTZ;
    selected_format_lane_count = selected_format->mipi_info.lane_num;
    selected_format_uses_line_sync = selected_format->mipi_info.line_sync_en;
    selected_format_is_raw8 = selected_format->format == ESP_CAM_SENSOR_PIXFORMAT_RAW8;
    selected_format_needs_isp = selected_format->isp_info != nullptr;
    if (selected_format->isp_info != nullptr) {
      sensor_bayer_order = csi_bayer_order_from_sensor_isp(selected_format->isp_info->isp_v1_info.bayer_type);
    }
    if (this->horizontal_mirror_configured_) {
      apply_managed_sensor_bool_parameter(sensor, ESP_CAM_SENSOR_HMIRROR, "horizontal_mirror", this->horizontal_mirror_);
    }
    if (this->vertical_flip_configured_) {
      apply_managed_sensor_bool_parameter(sensor, ESP_CAM_SENSOR_VFLIP, "vertical_flip", this->vertical_flip_);
    }
    if (this->test_pattern_) {
      apply_managed_sensor_test_pattern(sensor, true);
    }
    if (this->wb_mode_configured_) {
      apply_managed_sensor_bool_parameter(sensor, ESP_CAM_SENSOR_AWB, "wb_mode:auto", this->wb_mode_ == 0);
      apply_managed_sensor_int_parameter(sensor, ESP_CAM_SENSOR_WB, "wb_mode", this->wb_mode_);
    }
    if (this->aec_mode_configured_) {
      apply_managed_sensor_bool_parameter(sensor, ESP_CAM_SENSOR_AE_CONTROL, "aec_mode", this->aec_enabled_);
    }
    if (this->ae_level_configured_) {
      apply_managed_sensor_int_parameter(sensor, ESP_CAM_SENSOR_AE_LEVEL, "ae_level", this->ae_level_);
    }
    if (this->agc_mode_configured_) {
      apply_managed_sensor_bool_parameter(sensor, ESP_CAM_SENSOR_AGC, "agc_mode", this->agc_enabled_);
    }
    if (this->sharpness_configured_) {
      apply_managed_sensor_int_parameter(sensor, ESP_CAM_SENSOR_SHARPNESS, "sharpness", this->sharpness_);
    }
    if (this->denoise_configured_) {
      apply_managed_sensor_int_parameter(sensor, ESP_CAM_SENSOR_DENOISE, "denoise", this->denoise_);
    }
    if (this->dead_pixel_correction_configured_) {
      apply_managed_sensor_bool_parameter(sensor, ESP_CAM_SENSOR_DPC, "dead_pixel_correction",
                                          this->dead_pixel_correction_);
    }
    if (this->black_level_correction_configured_) {
      apply_managed_sensor_bool_parameter(sensor, ESP_CAM_SENSOR_BLC, "black_level_correction",
                                          this->black_level_correction_);
    }
    if (this->lens_shading_correction_configured_) {
      apply_managed_sensor_bool_parameter(sensor, ESP_CAM_SENSOR_LENC, "lens_shading_correction",
                                          this->lens_shading_correction_);
    }
    this->apply_night_mode_();
    ESP_LOGI(TAG, "Format set: %s → %" PRIu32 "x%" PRIu32,
             selected_format->name, this->target_width_, this->target_height_);
  }

  const bool apply_horizontal_mirror = this->horizontal_mirror_configured_ && this->horizontal_mirror_;
  const bool apply_vertical_flip = this->vertical_flip_configured_ && this->vertical_flip_;
  CsiBayerOrder effective_bayer_order = this->bayer_order_;
  if (this->bayer_order_auto_) {
    effective_bayer_order = csi_bayer_order_after_orientation(sensor_bayer_order, apply_horizontal_mirror,
                                                             apply_vertical_flip);
  }
  const char *effective_bayer_order_name = csi_bayer_order_to_string(effective_bayer_order);
  ESP_LOGI(TAG, "Bayer order: sensor=%s requested=%s effective=%s orientation=%s/%s",
           csi_bayer_order_to_string(sensor_bayer_order), this->bayer_order_name_, effective_bayer_order_name,
           this->horizontal_mirror_configured_ ? (this->horizontal_mirror_ ? "mirror" : "no-mirror") : "unchanged",
           this->vertical_flip_configured_ ? (this->vertical_flip_ ? "flip" : "no-flip") : "unchanged");

  // ── Frame buffer queues ───────────────────────────────────────────────────
  // Pad to 16-aligned dims: the H264 encoder reads align16(w)*align16(h)*1.5
  // (official example pattern) — e.g. 1080 rows → 1088. ISP writes real dims;
  // padding rows are unused slack at the end of the buffer.
  const auto align_frame_dimension = [](uint32_t value) -> uint32_t {
    return (value + K_FRAME_DIMENSION_ALIGNMENT - 1U) & ~(K_FRAME_DIMENSION_ALIGNMENT - 1U);
  };
  const uint32_t aligned_width = align_frame_dimension(this->target_width_);
  const uint32_t aligned_height = align_frame_dimension(this->target_height_);
  const size_t aligned_pixel_count = static_cast<size_t>(aligned_width) * aligned_height;
  this->fb_size_ = aligned_pixel_count * K_YUV420_BYTES_NUMERATOR / K_YUV420_BYTES_DENOMINATOR;
  this->produced_ = xQueueCreate(this->fb_count_, sizeof(void *));
  this->consumed_ = xQueueCreate(this->fb_count_, sizeof(void *));
  if (this->fb_count_ < K_LOW_LATENCY_RECOMMENDED_FRAME_BUFFERS) {
    ESP_LOGW(TAG, "frame_buffer_count=%d leaves no spare buffer while H.264 holds one CSI buffer; use at least %u for low-latency streaming",
             this->fb_count_, K_LOW_LATENCY_RECOMMENDED_FRAME_BUFFERS);
  }

  bool fb_dma_capable = true;
  for (int frame_buffer_index = 0; frame_buffer_index < this->fb_count_; frame_buffer_index++) {
    void *frame_buffer = heap_caps_aligned_alloc(K_FRAME_BUFFER_ALIGNMENT_BYTES, this->fb_size_,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (frame_buffer == nullptr) {
      fb_dma_capable = false;
      frame_buffer = heap_caps_aligned_alloc(K_FRAME_BUFFER_ALIGNMENT_BYTES, this->fb_size_,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (frame_buffer == nullptr) {
      ESP_LOGE(TAG, "FB %zu alloc failed (%zu B)", static_cast<size_t>(frame_buffer_index), this->fb_size_);
      this->mark_failed();
      return;
    }
    xQueueSendToBack(this->consumed_, &frame_buffer, 0);
  }
  ESP_LOGI(TAG, "Frame buffers: %zu × %zu B SPIRAM%s", static_cast<size_t>(this->fb_count_), this->fb_size_,
           fb_dma_capable ? " DMA" : "");
  if (!fb_dma_capable) {
    ESP_LOGW(TAG, "CSI frame buffers are not MALLOC_CAP_DMA; H.264 may need an internal input copy");
  }

  // ── ISP (RAW Bayer → YUV420) ───────────────────────────────────────────────
  if (selected_format_needs_isp) {
    esp_isp_processor_cfg_t isp_cfg = {};
    isp_cfg.clk_hz                 = K_ISP_PROCESSOR_CLOCK_HZ;
    isp_cfg.input_data_source      = ISP_INPUT_DATA_SOURCE_CSI;
    isp_cfg.input_data_color_type  = selected_format_is_raw8 ? ISP_COLOR_RAW8 : ISP_COLOR_RAW10;
    isp_cfg.output_data_color_type = ISP_COLOR_YUV420;
    isp_cfg.bayer_order            = csi_bayer_order_to_esp(effective_bayer_order);
    isp_cfg.has_line_start_packet  = selected_format_uses_line_sync;
    isp_cfg.has_line_end_packet    = selected_format_uses_line_sync;
    isp_cfg.h_res                  = this->target_width_;
    isp_cfg.v_res                  = this->target_height_;
    if (esp_isp_new_processor(&isp_cfg, &this->isp_proc_) != ESP_OK) {
      ESP_LOGE(TAG, "ISP init failed");
      this->mark_failed();
      return;
    }
    ESP_ERROR_CHECK(esp_isp_enable(this->isp_proc_));

    esp_isp_demosaic_config_t demosaic_cfg = {};
    demosaic_cfg.grad_ratio.integer = K_DEMOSAIC_GRAD_RATIO_INTEGER;
    demosaic_cfg.grad_ratio.decimal = K_DEMOSAIC_GRAD_RATIO_DECIMAL;
    demosaic_cfg.padding_mode = ISP_DEMOSAIC_EDGE_PADDING_MODE_SRND_DATA;
    esp_err_t demosaic_err = esp_isp_demosaic_configure(this->isp_proc_, &demosaic_cfg);
    if (demosaic_err == ESP_OK) {
      demosaic_err = esp_isp_demosaic_enable(this->isp_proc_);
    }
    if (demosaic_err != ESP_OK) {
      ESP_LOGW(TAG, "ISP demosaic enable failed: %s", esp_err_to_name(demosaic_err));
    } else {
      ESP_LOGI(TAG, "ISP demosaic enabled, bayer_order=%s", effective_bayer_order_name);
    }

    if (this->enable_ccm_) {
      // Sensor color correction matrix. Default is identity for IMX219 bring-up.
      esp_isp_ccm_config_t ccm_cfg = {};
      ccm_cfg.matrix[0][0] = this->ccm_[0];
      ccm_cfg.matrix[0][1] = this->ccm_[1];
      ccm_cfg.matrix[0][2] = this->ccm_[2];
      ccm_cfg.matrix[1][0] = this->ccm_[3];
      ccm_cfg.matrix[1][1] = this->ccm_[4];
      ccm_cfg.matrix[1][2] = this->ccm_[5];
      ccm_cfg.matrix[2][0] = this->ccm_[6];
      ccm_cfg.matrix[2][1] = this->ccm_[7];
      ccm_cfg.matrix[2][2] = this->ccm_[8];
      ccm_cfg.saturation = true;
      ccm_cfg.flags.update_once_configured = true;
      esp_err_t ccm_err = esp_isp_ccm_configure(this->isp_proc_, &ccm_cfg);
      if (ccm_err == ESP_OK) {
        ccm_err = esp_isp_ccm_enable(this->isp_proc_);
      }
      if (ccm_err != ESP_OK) {
        ESP_LOGW(TAG, "ISP CCM enable failed: %s", esp_err_to_name(ccm_err));
      } else {
        ESP_LOGI(TAG, "ISP CCM enabled: [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]",
                 this->ccm_[0], this->ccm_[1], this->ccm_[2],
                 this->ccm_[3], this->ccm_[4], this->ccm_[5],
                 this->ccm_[6], this->ccm_[7], this->ccm_[8]);
      }
    }

    esp_isp_color_config_t color_cfg = {};
    const IspColorFixedPoint contrast = make_isp_color_fixed_point(this->contrast_);
    const IspColorFixedPoint saturation = make_isp_color_fixed_point(this->saturation_);
    color_cfg.color_contrast.integer = contrast.integer;
    color_cfg.color_contrast.decimal = contrast.decimal;
    color_cfg.color_saturation.integer = saturation.integer;
    color_cfg.color_saturation.decimal = saturation.decimal;
    color_cfg.color_hue = this->hue_;
    color_cfg.color_brightness = this->brightness_;
    color_cfg.flags.update_once_configured = true;
    esp_err_t color_err = esp_isp_color_configure(this->isp_proc_, &color_cfg);
    if (color_err == ESP_OK) {
      color_err = esp_isp_color_enable(this->isp_proc_);
    }
    if (color_err != ESP_OK) {
      ESP_LOGW(TAG, "ISP color controller enable failed: %s", esp_err_to_name(color_err));
    } else {
      ESP_LOGI(TAG, "ISP color enabled: hue=%" PRIu32 " brightness=%d contrast=%.2f saturation=%.2f", this->hue_,
               this->brightness_, this->contrast_, this->saturation_);
    }

    ESP_LOGI(TAG, "ISP: RAW Bayer → YUV420, bayer_order=%s", effective_bayer_order_name);
  } else {
    ESP_LOGW(TAG, "No ISP info — format may not need ISP");
  }

  // ── CSI controller ───────────────────────────────────────────────────────
  esp_cam_ctlr_csi_config_t csi_cfg = {};
  csi_cfg.ctlr_id                = K_CSI_CONTROLLER_ID;
  csi_cfg.clk_src                = MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  csi_cfg.h_res                  = this->target_width_;
  csi_cfg.v_res                  = this->target_height_;
  csi_cfg.data_lane_num          = selected_format_lane_count;
  csi_cfg.lane_bit_rate_mbps     = this->lane_bit_rate_mbps_;
  csi_cfg.input_data_color_type  = selected_format_is_raw8 ? CAM_CTLR_COLOR_RAW8 : CAM_CTLR_COLOR_RAW10;
  csi_cfg.output_data_color_type = CAM_CTLR_COLOR_YUV420;
  csi_cfg.queue_items            = K_CSI_QUEUE_ITEMS;
  csi_cfg.byte_swap_en           = false;
  csi_cfg.bk_buffer_dis          = false;

  if (esp_cam_new_csi_ctlr(&csi_cfg, &this->cam_handle_) != ESP_OK) {
    ESP_LOGE(TAG, "CSI ctlr failed");
    this->mark_failed();
    return;
  }

  esp_cam_ctlr_evt_cbs_t cbs = {};
  cbs.on_get_new_trans  = CsiCamera::s_dma_start;
  cbs.on_trans_finished = CsiCamera::s_dma_complete;
  ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this));
  ESP_LOGI(TAG, "CSI queue_items=%u frame_buffer_count=%d", K_CSI_QUEUE_ITEMS, this->fb_count_);
  ESP_ERROR_CHECK(esp_cam_ctlr_enable(this->cam_handle_));
  ESP_ERROR_CHECK(esp_cam_ctlr_start(this->cam_handle_));
  ESP_LOGI(TAG, "CSI controller started");

  // ── Start sensor stream ──────────────────────────────────────────────────
  if (use_local_imx219_driver) {
    if (!this->i2c_write_reg16_(IMX219_REG_MODE_SELECT, IMX219_MODE_STREAMING)) {
      ESP_LOGE(TAG, "IMX219 stream start failed");
      this->mark_failed();
      return;
    }
  } else {
    int stream_enable = K_SENSOR_STREAM_ENABLE;
    if (esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_enable) != ESP_OK) {
      ESP_LOGE(TAG, "Sensor stream start failed");
      this->mark_failed();
      return;
    }
  }
  ESP_LOGI(TAG, "Sensor streaming started");

  // ── Capture task ─────────────────────────────────────────────────────────
  xTaskCreatePinnedToCore(CsiCamera::capture_task_trampoline, "csi_cap",
                          K_CAPTURE_TASK_STACK_BYTES, this, K_CAPTURE_TASK_PRIORITY, nullptr,
                          K_CAPTURE_TASK_CORE);

  ESP_LOGI(TAG, "CSI camera ready %" PRIu32 "x%" PRIu32 " yuv420",
           this->target_width_, this->target_height_);
}

void CsiCamera::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32-P4 CSI Camera:");
  ESP_LOGCONFIG(TAG, "  %" PRIu32 "x%" PRIu32, this->target_width_, this->target_height_);
  ESP_LOGCONFIG(TAG, "  LDO: ch%d @ %d mV", this->ldo_chan_, this->ldo_mv_);
  ESP_LOGCONFIG(TAG, "  Sensor profile: %s", this->sensor_profile_.c_str());
  ESP_LOGCONFIG(TAG, "  Format: %s", this->sensor_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Output: YUV420 via ISP demosaic");
  ESP_LOGCONFIG(TAG, "  ISP CCM: %s", this->enable_ccm_ ? "enabled" : "disabled");
  ESP_LOGCONFIG(TAG, "  ISP hue: %" PRIu32 ", brightness: %d, contrast: %.2f, saturation: %.2f", this->hue_,
                this->brightness_, this->contrast_, this->saturation_);
  ESP_LOGCONFIG(TAG, "  Orientation: horizontal_mirror=%s, vertical_flip=%s, test_pattern=%s",
                onoff(this->horizontal_mirror_), onoff(this->vertical_flip_), onoff(this->test_pattern_));
  ESP_LOGCONFIG(TAG, "  Sensor controls: wb_mode=%s aec_mode=%s ae_level=%d agc_mode=%s night_mode=%s",
                this->wb_mode_configured_ ? (this->wb_mode_ == 0 ? "auto" : "manual-preset") : "unchanged",
                this->aec_mode_configured_ ? (this->aec_enabled_ ? "auto" : "manual") : "unchanged", this->ae_level_,
                this->agc_mode_configured_ ? (this->agc_enabled_ ? "auto" : "manual") : "unchanged",
                this->night_mode_configured_ ? onoff(this->night_mode_) : "unchanged");
  ESP_LOGCONFIG(TAG, "  Sensor corrections: dead_pixel=%s black_level=%s lens_shading=%s",
                this->dead_pixel_correction_configured_ ? onoff(this->dead_pixel_correction_) : "unchanged",
                this->black_level_correction_configured_ ? onoff(this->black_level_correction_) : "unchanged",
                this->lens_shading_correction_configured_ ? onoff(this->lens_shading_correction_) : "unchanged");
  ESP_LOGCONFIG(TAG, "  ISP bayer_order: %s", this->bayer_order_name_);
}

void CsiCamera::start_stream(camera::CameraRequester requester) {
  for (auto *l : this->listeners_) {
    l->on_stream_start();
  }
  this->stream_requesters_ |= (1U << requester);
}

void CsiCamera::stop_stream(camera::CameraRequester requester) {
  for (auto *l : this->listeners_) {
    l->on_stream_stop();
  }
  this->stream_requesters_ &= ~(1U << requester);
}

void CsiCamera::capture_task_body_() {
  uint32_t frame_count = 0;
  uint32_t rate_start_us = 0;
  while (true) {
    void *frame_buffer = nullptr;
    if (xQueueReceive(this->produced_, &frame_buffer, pdMS_TO_TICKS(K_CAPTURE_QUEUE_WAIT_MS)) != pdTRUE) {
      ESP_LOGW(TAG, "No frame in %" PRIu32 "ms", K_CAPTURE_QUEUE_WAIT_MS);
      continue;
    }
    frame_count++;
    const uint32_t now_us = micros();
    if (rate_start_us == 0) {
      rate_start_us = now_us;
    }
    if (frame_count <= K_INITIAL_FRAME_LOG_COUNT) {
      ESP_LOGI(TAG, "Frame #%" PRIu32 " buf=%p", frame_count, frame_buffer);
    } else if (frame_count % K_FRAME_RATE_LOG_INTERVAL == 0) {
      uint32_t source_fps_tenths = 0;
      const uint32_t elapsed_us = now_us - rate_start_us;
      if (elapsed_us != 0) {
        source_fps_tenths = static_cast<uint32_t>(
            (static_cast<uint64_t>(K_FRAME_RATE_LOG_INTERVAL) * K_FRAME_RATE_LOG_DECIMAL_SCALE * K_HERTZ_PER_MEGAHERTZ +
             elapsed_us / 2U) /
            elapsed_us);
      }
      ESP_LOGI(TAG, "Frame #%" PRIu32 " buf=%p source=%" PRIu32 ".%" PRIu32 "fps", frame_count, frame_buffer,
               source_fps_tenths / K_FRAME_RATE_LOG_DECIMAL_SCALE,
               source_fps_tenths % K_FRAME_RATE_LOG_DECIMAL_SCALE);
      rate_start_us = now_us;
    }

    esp_cache_msync(frame_buffer, this->fb_size_, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

    const uint8_t single = this->single_requesters_.load();
    const uint8_t stream = this->stream_requesters_.load();
    const bool deliver = single || stream || !this->listeners_.empty();
    if (deliver) {
      auto img = std::make_shared<CsiCameraImage>(frame_buffer, this->fb_size_, single | stream, this->consumed_);
      for (auto *l : this->listeners_) {
        l->on_camera_image(img);
      }
      this->single_requesters_.store(0);
    } else {
      xQueueSendToBack(this->consumed_, &frame_buffer, portMAX_DELAY);
    }
  }
}

#else

void CsiCamera::setup() {
  ESP_LOGE(TAG, "esp_cam_sensor/SCCB managed-component headers are not available in this build environment");
  this->mark_failed();
}

void CsiCamera::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32-P4 CSI Camera: managed camera sensor headers unavailable");
}

void CsiCamera::start_stream(camera::CameraRequester requester) { (void) requester; }
void CsiCamera::stop_stream(camera::CameraRequester requester) { (void) requester; }
void CsiCamera::set_night_mode(bool enabled) {
  this->night_mode_ = enabled;
  this->night_mode_configured_ = true;
}
void CsiCamera::apply_night_mode_() {}
bool CsiCamera::update_managed_sensor_register_bit_(uint32_t reg, uint32_t mask, bool enabled, const char *name) {
  (void) reg;
  (void) mask;
  (void) enabled;
  (void) name;
  return false;
}

#endif  // CSI_CAMERA_HAS_MANAGED_SENSOR_HEADERS

}  // namespace esphome::csi_camera

#endif
