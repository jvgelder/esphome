#pragma once
#ifdef USE_CAMERA_H264

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/camera/buffer.h"
#include "esphome/components/camera/encoder.h"

#include <atomic>
#include <functional>
#include <utility>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

struct esp_h264_enc_if;
using esp_h264_enc_handle_t = esp_h264_enc_if *;

namespace esphome::camera_h264 {

// camera::EncoderBuffer implementation backed by esp_h264's aligned allocator.
// This reuses the upstream camera encoder interface without giving up the 64-byte
// alignment / internal-vs-PSRAM placement required by the ESP32-P4 H.264 block.
class H264EncoderBuffer final : public camera::EncoderBuffer {
 public:
  bool set_buffer_size(size_t size) override;
  uint8_t *get_data() const override { return this->data_; }
  size_t get_size() const override { return this->size_; }
  size_t get_max_size() const override { return this->capacity_; }

  bool reserve(size_t size, size_t max_size);
  void set_encoded_size(size_t size) { this->size_ = size; }
  void release();
  ~H264EncoderBuffer() override { this->release(); }

 protected:
  uint8_t *data_{nullptr};
  size_t size_{0};      // bytes currently produced by the encoder
  size_t capacity_{0};  // allocated bytes available to the encoder
};

class CameraH264Encoder : public Component,
                               public camera::CameraListener,
                               public camera::Encoder {
 public:
  void set_width(uint32_t v)    { this->width_   = v; }
  void set_height(uint32_t v)   { this->height_  = v; }
  void set_fps(uint8_t v)       { this->fps_     = v; }
  void set_bitrate(uint32_t v)  { this->bitrate_ = v; }
  void set_gop(uint32_t v)      { this->gop_     = v; }
  void set_qp_min(uint8_t v)    { this->qp_min_  = v; }
  void set_qp_max(uint8_t v)    { this->qp_max_  = v; }
  void set_always_on(bool v) { this->always_on_ = v; }
  void add_camera(camera::Camera *cam) {
    cam->add_listener(this);
    this->cam_ = cam;
  }
  camera::Camera *get_camera() { return this->cam_; }

  // RTSP uses this to size its fixed slot pool. This is capacity, not last AU size.
  size_t get_out_size() const { return this->output_.get_max_size(); }
  uint8_t get_fps() const { return this->fps_; }
  bool is_always_on() const { return this->always_on_; }
  void add_frame_callback(std::function<void(const uint8_t *, size_t, bool)> &&cb) {
    this->callbacks_.add(std::move(cb));
  }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI - 1; }

  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;
  void on_stream_start() override;
  void on_stream_stop() override;
  void request_keyframe();

  // camera::Encoder interface. The CSI path enters through on_camera_image()
  // and passes O_UYY_E_VYY frames directly to the hardware encoder.
  camera::EncoderError encode_pixels(camera::CameraImageSpec *spec, camera::Buffer *pixels) override;
  camera::EncoderBuffer *get_output_buffer() override { return &this->output_; }

  ~CameraH264Encoder() override;

 protected:
  bool init_encoder_();
  void reset_encoder_for_keyframe_();
  void start_encoder_task_();
  void encoder_task_body_();
  static void encoder_task_trampoline(void *arg) {
    static_cast<CameraH264Encoder *>(arg)->encoder_task_body_();
    vTaskDelete(nullptr);
  }
  bool queue_camera_image_(const std::shared_ptr<camera::CameraImage> &image);
  bool allocate_output_buffer_(uint32_t want);
  camera::EncoderError encode_frame_(const uint8_t *data, size_t len, bool emit = true);
  bool should_encode_live_frame_();

  uint32_t width_   {800};
  uint32_t height_  {800};
  uint8_t  fps_     {25};
  uint32_t bitrate_ {2000000};
  uint32_t gop_     {15};
  uint8_t  qp_min_  {24};
  uint8_t  qp_max_  {51};

  camera::Camera    *cam_{nullptr};
  esp_h264_enc_handle_t enc_{nullptr};

  H264EncoderBuffer output_{};
  bool always_on_{false};
  uint32_t last_encode_start_us_{0};
  std::atomic<bool> streaming_{false};
  std::atomic<bool> force_keyframe_next_{false};
  std::atomic<bool> encoder_task_stop_{false};
  SemaphoreHandle_t frame_mutex_{nullptr};
  SemaphoreHandle_t frame_sem_{nullptr};
  TaskHandle_t encoder_task_{nullptr};
  std::shared_ptr<camera::CameraImage> pending_image_;
  CallbackManager<void(const uint8_t *, size_t, bool)> callbacks_;
};

}  // namespace esphome::camera_h264

#endif
