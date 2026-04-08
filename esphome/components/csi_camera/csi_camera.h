#pragma once
#ifdef USE_CSI_CAMERA

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/i2c/i2c.h"
#include "i2c_sccb_adapter.h"
#include "csi_types.h"
#include "imx219_regs.h"
#include <atomic>
#include <string>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/isp.h>
#include <esp_cam_ctlr.h>
#include <esp_cam_ctlr_csi.h>
#include <array>
#include <utility>
#include <hal/color_types.h>

namespace esphome::csi_camera {

class CsiCameraImage : public camera::CameraImage {
 public:
  CsiCameraImage(void *data, size_t len, uint8_t req, QueueHandle_t return_queue)
      : data_(static_cast<uint8_t *>(data)), len_(len), requesters_(req), return_queue_(return_queue) {}
  uint8_t *get_data_buffer() override { return this->data_; }
  size_t   get_data_length() override { return this->len_; }
  bool was_requested_by(camera::CameraRequester r) const override {
    return (this->requesters_ & (1U << r)) != 0;
  }
  ~CsiCameraImage() override;
 private:
  uint8_t *data_;
  size_t len_;
  uint8_t requesters_;
  QueueHandle_t return_queue_;
};

class CsiCameraImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> img) override {
    image_ = std::move(img); offset_ = 0;
  }
  size_t   available() const override { return image_ ? image_->get_data_length() - offset_ : 0; }
  uint8_t *peek_data_buffer() override { return image_->get_data_buffer() + offset_; }
  void     consume_data(size_t len) override { offset_ += len; }
  void     return_image() override { image_.reset(); }
 private:
  std::shared_ptr<camera::CameraImage> image_;
  size_t offset_{0};
};

// camera::Camera already inherits Component — do NOT also inherit Component
class CsiCamera : public camera::Camera, public i2c::I2CDevice {
 public:
  void set_power_down_pin(int p)       { power_down_pin_ = p; }
  void set_ldo_chan(int c)             { ldo_chan_       = c; }
  void set_ldo_mv(int mv)             { ldo_mv_        = mv; }
  void set_frame_buffer_count(int n)   { fb_count_ = n; }
  void set_sensor_profile(const std::string &s) {
    sensor_profile_ = s;
    sensor_type_ = csi_sensor_type_from_string(sensor_profile_.c_str());
  }
  void set_sensor_format(const std::string &f) { sensor_format_ = f; }
  void set_lane_bit_rate_mbps(uint32_t v) { lane_bit_rate_mbps_ = v; }
  void set_enable_ccm(bool e)          { enable_ccm_    = e; }
  void set_ccm(float rr, float rg, float rb, float gr, float gg, float gb, float br, float bg, float bb) {
    ccm_ = {rr, rg, rb, gr, gg, gb, br, bg, bb};
  }
  void set_hue(uint32_t h)             { hue_           = h; }
  void set_brightness(int b)           { brightness_    = b; }
  void set_contrast(float c)           { contrast_      = c; }
  void set_saturation(float s)         { saturation_    = s; }
  void set_vertical_flip(bool enabled) {
    this->vertical_flip_ = enabled;
    this->vertical_flip_configured_ = true;
  }
  void set_horizontal_mirror(bool enabled) {
    this->horizontal_mirror_ = enabled;
    this->horizontal_mirror_configured_ = true;
  }
  void set_test_pattern(bool enabled)  { test_pattern_  = enabled; }
  void set_wb_mode(int mode) {
    this->wb_mode_ = mode;
    this->wb_mode_configured_ = true;
  }
  void set_aec_mode(bool enabled) {
    this->aec_enabled_ = enabled;
    this->aec_mode_configured_ = true;
  }
  void set_ae_level(int level) {
    this->ae_level_ = level;
    this->ae_level_configured_ = true;
  }
  void set_agc_mode(bool enabled) {
    this->agc_enabled_ = enabled;
    this->agc_mode_configured_ = true;
  }
  void set_sharpness(int sharpness) {
    this->sharpness_ = sharpness;
    this->sharpness_configured_ = true;
  }
  void set_denoise(int denoise) {
    this->denoise_ = denoise;
    this->denoise_configured_ = true;
  }
  void set_dead_pixel_correction(bool enabled) {
    this->dead_pixel_correction_ = enabled;
    this->dead_pixel_correction_configured_ = true;
  }
  void set_black_level_correction(bool enabled) {
    this->black_level_correction_ = enabled;
    this->black_level_correction_configured_ = true;
  }
  void set_lens_shading_correction(bool enabled) {
    this->lens_shading_correction_ = enabled;
    this->lens_shading_correction_configured_ = true;
  }
  void set_night_mode(bool enabled);
  void set_bayer_order(const std::string &order) {
    if (order == "auto") {
      this->bayer_order_auto_ = true;
      this->bayer_order_name_ = "auto";
      return;
    }
    this->bayer_order_auto_ = false;
    this->bayer_order_ = csi_bayer_order_from_string(order.c_str());
    this->bayer_order_name_ = csi_bayer_order_to_string(this->bayer_order_);
  }

  void setup() override;
  void loop() override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  camera::CameraImageReader *create_image_reader() override { return new CsiCameraImageReader(); }
  void request_image(camera::CameraRequester r) override { single_requesters_ |= (1U << r); }
  void start_stream(camera::CameraRequester r) override;
  void stop_stream(camera::CameraRequester r) override;
  void add_listener(camera::CameraListener *l) override { listeners_.push_back(l); }

  // ISR-accessible queues (public for ISR static callbacks)
  QueueHandle_t produced_{nullptr};
  QueueHandle_t consumed_{nullptr};
  size_t        fb_size_{0};

 protected:
  void setup_phase1_();
  void capture_task_body_();
  static void capture_task_trampoline(void *arg) {
    static_cast<CsiCamera *>(arg)->capture_task_body_();
    vTaskDelete(nullptr);
  }

  // ISR DMA callbacks — static trampolines are IRAM_ATTR, members are not
  static bool IRAM_ATTR s_dma_start(esp_cam_ctlr_handle_t, esp_cam_ctlr_trans_t *, void *);
  static bool IRAM_ATTR s_dma_complete(esp_cam_ctlr_handle_t, esp_cam_ctlr_trans_t *, void *);
  bool dma_start_cb_(esp_cam_ctlr_trans_t *trans);
  bool dma_complete_cb_(esp_cam_ctlr_trans_t *trans);

  bool i2c_write_reg16_(uint16_t reg, uint8_t val);
  bool i2c_read_reg16_(uint16_t reg, uint8_t *val);  // NOLINT(readability-non-const-parameter)
  bool imx219_detect_();
  bool imx219_apply_table_(const Imx219Reg *table);
  bool imx219_apply_mode_();
  bool imx219_apply_orientation_();
  void apply_night_mode_();
  bool update_managed_sensor_register_bit_(uint32_t reg, uint32_t mask, bool enabled, const char *name);

  int         power_down_pin_{-1};
  int         ldo_chan_      {3};
  int         ldo_mv_        {2500};
  int         fb_count_      {2};
  std::string sensor_profile_{"ov5647"};
  CsiSensorType sensor_type_{CsiSensorType::OV5647};
  std::string sensor_format_ {"MIPI_2lane_24Minput_RAW10_1920x1080_30fps"};
  uint32_t    target_width_  {1920};
  uint32_t    target_height_ {1080};
  uint32_t    lane_bit_rate_mbps_{912};

  bool enable_ccm_{true};
  std::array<float, 9> ccm_{
      1.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 1.0f,
  };
  uint32_t hue_{0};
  int brightness_{0};
  float contrast_{1.0f};
  float saturation_{1.0f};
  bool vertical_flip_{false};
  bool vertical_flip_configured_{false};
  bool horizontal_mirror_{false};
  bool horizontal_mirror_configured_{false};
  bool test_pattern_{false};
  int wb_mode_{0};
  bool wb_mode_configured_{false};
  bool aec_enabled_{true};
  bool aec_mode_configured_{false};
  int ae_level_{0};
  bool ae_level_configured_{false};
  bool agc_enabled_{true};
  bool agc_mode_configured_{false};
  int sharpness_{0};
  bool sharpness_configured_{false};
  int denoise_{0};
  bool denoise_configured_{false};
  bool dead_pixel_correction_{false};
  bool dead_pixel_correction_configured_{false};
  bool black_level_correction_{false};
  bool black_level_correction_configured_{false};
  bool lens_shading_correction_{false};
  bool lens_shading_correction_configured_{false};
  bool night_mode_{false};
  bool night_mode_configured_{false};
  bool bayer_order_auto_{true};
  CsiBayerOrder bayer_order_{CsiBayerOrder::GBRG};
  const char *bayer_order_name_{"auto"};
  I2CSCCBAdapter i2c_adapter_{this};
  esp_cam_ctlr_handle_t   cam_handle_{nullptr};
  isp_proc_handle_t       isp_proc_  {nullptr};
  void *sensor_{nullptr};


  std::atomic<uint8_t> single_requesters_{0};
  std::atomic<uint8_t> stream_requesters_{0};
  std::vector<camera::CameraListener *> listeners_;
};


template<typename... Ts> class CsiCameraSetNightModeAction : public Action<Ts...> {
 public:
  explicit CsiCameraSetNightModeAction(CsiCamera *camera) : camera_(camera) {}
  TEMPLATABLE_VALUE(bool, night_mode)

 protected:
  void play(Ts... x) override { this->camera_->set_night_mode(this->night_mode_.value(x...)); }

  CsiCamera *camera_;
};

}  // namespace esphome::csi_camera
#endif
