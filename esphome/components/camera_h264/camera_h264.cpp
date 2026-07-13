#include "esphome/core/defines.h"
#ifdef USE_CAMERA_H264

#include "camera_h264.h"
#include "h264_annexb.h"
#include "h264_units.h"
#include "esphome/core/log.h"

#include "sdkconfig.h"

#include <cstddef>
#include <cstdint>
#include <cinttypes>

#if __has_include(<esp_h264_enc_single.h>) && __has_include(<esp_h264_enc_single_hw.h>) && \
    __has_include(<esp_h264_alloc.h>)
#define CAMERA_H264_HAS_ESP_H264 1
extern "C" {
#include <esp_h264_enc_single.h>
#include <esp_h264_enc_single_hw.h>
}

// esp_h264_alloc.h lacks extern "C" guards in some component versions.
extern "C" {
#include <esp_h264_alloc.h>
}
#include <esp_heap_caps.h>

#else
#define CAMERA_H264_HAS_ESP_H264 0
#endif

namespace esphome::camera_h264 {

static const char *const TAG = "h264_encoder";
static constexpr size_t K_H264_ALLOC_ALIGNMENT_BYTES = 64;
static constexpr size_t K_H264_INTERNAL_ALLOC_LIMIT_BYTES = kibibytes(256);
static constexpr size_t K_H264_OUTPUT_BUFFER_MIN_BYTES = kibibytes(64);
static constexpr size_t K_H264_OUTPUT_BUFFER_MARGIN_BYTES = kibibytes(128);
static constexpr uint32_t K_LOW_LATENCY_GOP_WARN_FRAMES = 15;
static constexpr uint32_t K_LOW_BITRATE_WARN_BITS_PER_SEC = 8000000;
static constexpr uint8_t K_LOW_BITRATE_QP_MAX_WARN = 45;
static constexpr uint32_t K_ENCODER_TASK_STACK_BYTES = 12288;
static constexpr UBaseType_t K_ENCODER_TASK_PRIORITY = 5;
static constexpr BaseType_t K_ENCODER_TASK_CORE = 0;

#if CAMERA_H264_HAS_ESP_H264

// Official example pattern: buffers sized on 16-aligned dims, cfg.res keeps real dims.
static inline uint32_t align16(uint32_t v) { return (v + 15u) & ~15u; }

static size_t h264_input_frame_size(uint32_t width, uint32_t height) {
  return static_cast<size_t>(align16(width)) * align16(height) *
         ESP_H264_GET_BPP_BY_PIC_TYPE(ESP_H264_RAW_FMT_O_UYY_E_VYY);
}

static uint32_t h264_output_buffer_size(uint32_t width, uint32_t height) {
  const size_t size = h264_input_frame_size(width, height) + K_H264_OUTPUT_BUFFER_MARGIN_BYTES;
  return static_cast<uint32_t>(clamp_at_least(size, K_H264_OUTPUT_BUFFER_MIN_BYTES));
}

bool H264EncoderBuffer::set_buffer_size(size_t size) {
  return this->reserve(size, size);
}

bool H264EncoderBuffer::reserve(size_t size, size_t max_size) {
  size = clamp_at_most(clamp_at_least(size, K_H264_OUTPUT_BUFFER_MIN_BYTES), max_size);
  if (size <= this->capacity_ && this->data_ != nullptr) {
    this->size_ = 0;
    return true;
  }

  uint32_t actual = 0;
  auto mem_type = (size <= K_H264_INTERNAL_ALLOC_LIMIT_BYTES) ? ESP_H264_MEM_INTERNAL : ESP_H264_MEM_SPIRAM;
  const char *mem_name = (mem_type == ESP_H264_MEM_INTERNAL) ? "INTERNAL" : "SPIRAM";

  uint8_t *new_buf = static_cast<uint8_t *>(
      esp_h264_aligned_calloc(K_H264_ALLOC_ALIGNMENT_BYTES, 1, size, &actual, mem_type));

  if (!new_buf && mem_type == ESP_H264_MEM_INTERNAL) {
    mem_type = ESP_H264_MEM_SPIRAM;
    mem_name = "SPIRAM";
    new_buf = static_cast<uint8_t *>(
        esp_h264_aligned_calloc(K_H264_ALLOC_ALIGNMENT_BYTES, 1, size, &actual, mem_type));
  }

  if (new_buf == nullptr) {
    return false;
  }

  this->release();
  this->data_ = new_buf;
  this->capacity_ = actual;
  this->size_ = 0;
  ESP_LOGI(TAG, "Output buffer: %p (%zu B requested=%zu %s max=%zu)",
           this->data_, this->capacity_, size, mem_name, max_size);
  return true;
}

void H264EncoderBuffer::release() {
  if (this->data_ != nullptr) {
    esp_h264_free(this->data_);
    this->data_ = nullptr;
  }
  this->capacity_ = 0;
  this->size_ = 0;
}

CameraH264Encoder::~CameraH264Encoder() {
  this->encoder_task_stop_.store(true);
  if (this->frame_sem_ != nullptr) {
    xSemaphoreGive(this->frame_sem_);
  }
  if (this->enc_) {
    esp_h264_enc_close(this->enc_);
    esp_h264_enc_del(this->enc_);
    this->enc_ = nullptr;
  }
}

void CameraH264Encoder::setup() {
  ESP_LOGI(TAG, "Internal DMA free: %zu  SPIRAM free: %zu",
           heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  // Create the encoder EAGERLY, before any other allocation in this component:
  // its reference frame needs ~135KB of CONTIGUOUS internal RAM (hard
  // ESP_H264_MEM_INTERNAL in esp_h264_enc_hw_param.c) and lazy init on first
  // client fails from fragmentation (observed: free=241KB, largest=126KB).
  if (!this->init_encoder_()) {
    ESP_LOGE(TAG, "encoder init failed (contiguous internal RAM?)");
    this->mark_failed();
    return;
  }

  // The output buffer holds one encoded H.264 access unit, not the raw
  // CSI frame. Size it from the aligned raw input frame plus margin so the
  // code stays correct when resolution changes and avoids a fixed 1080p magic
  // number. For 1920x1080 YUV420 this is about 3.25 MiB.
  const uint32_t output_buffer_size = h264_output_buffer_size(this->width_, this->height_);
  if (!this->allocate_output_buffer_(output_buffer_size)) {
    ESP_LOGE(TAG, "out_buf alloc failed (%" PRIu32 " B)", output_buffer_size);
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "Input format O_UYY_E_VYY — zero-copy to encoder");

  if (this->always_on_) {
    this->streaming_.store(true);
    ESP_LOGI(TAG, "Encoder always_on enabled — pre-rolling SPS/PPS before RTSP clients");
  }

  this->start_encoder_task_();
}

void CameraH264Encoder::dump_config() {
  ESP_LOGCONFIG(TAG, "H.264 Encoder: %" PRIu32 "x%" PRIu32 " @%" PRIu8 "fps bitrate=%" PRIu32,
                this->width_, this->height_, this->fps_, this->bitrate_);
  ESP_LOGCONFIG(TAG, "  gop: %" PRIu32, this->gop_);
  ESP_LOGCONFIG(TAG, "  qp: %" PRIu8 "..%" PRIu8, this->qp_min_, this->qp_max_);
  ESP_LOGCONFIG(TAG, "  always_on: %s", YESNO(this->always_on_));
  ESP_LOGCONFIG(TAG, "  live frame limiter: %" PRIu8 " fps", this->fps_);
  if (this->always_on_ && this->gop_ > K_LOW_LATENCY_GOP_WARN_FRAMES) {
    ESP_LOGW(TAG, "always_on with gop=%" PRIu32 " can add RTSP PLAY startup latency; use gop <= 10 for low-latency NVR startup", this->gop_);
  }
}

bool CameraH264Encoder::allocate_output_buffer_(uint32_t want) {
  return this->output_.reserve(want, want);
}

bool CameraH264Encoder::init_encoder_() {
  esp_h264_enc_cfg_hw_t cfg = {};
  cfg.gop        = this->gop_;
  cfg.fps        = this->fps_;
  cfg.res.width  = this->width_;    // real dims in cfg (official pattern)
  cfg.res.height = this->height_;
  cfg.rc.bitrate = this->bitrate_;
  cfg.rc.qp_min  = this->qp_min_;
  cfg.rc.qp_max  = this->qp_max_;
  cfg.pic_type   = ESP_H264_RAW_FMT_O_UYY_E_VYY;  // only HW-supported format

  ESP_LOGI(TAG, "esp_h264_enc_hw_new %" PRIu32 "x%" PRIu32 " @%ufps bitrate=%" PRIu32
                " (live limiter=%" PRIu8 "fps)",
           this->width_, this->height_, static_cast<unsigned>(this->fps_), this->bitrate_,
           this->fps_);
  ESP_LOGI(TAG, "encoder cfg: struct=%zu gop=%u fps=%u bitrate=%u qp=%u..%u pic_type=%d(O_UYY_E_VYY) "
                "res=%ux%u",
           sizeof(cfg), static_cast<unsigned>(cfg.gop), static_cast<unsigned>(cfg.fps),
           static_cast<unsigned>(cfg.rc.bitrate), static_cast<unsigned>(cfg.rc.qp_min),
           static_cast<unsigned>(cfg.rc.qp_max), static_cast<int>(cfg.pic_type),
           static_cast<unsigned>(cfg.res.width), static_cast<unsigned>(cfg.res.height));
  if (this->qp_max_ < K_LOW_BITRATE_QP_MAX_WARN && this->bitrate_ <= K_LOW_BITRATE_WARN_BITS_PER_SEC) {
    ESP_LOGW(TAG, "qp_max=%u may prevent rate control from reaching low bitrate target=%" PRIu32
                  "bit/s; use qp_max 45..51 for constrained low-latency streaming",
             static_cast<unsigned>(this->qp_max_), this->bitrate_);
  }
  ESP_LOGI(TAG, "internal free=%zu largest=%zu",
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
           heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (esp_h264_enc_hw_new(&cfg, &this->enc_) != ESP_H264_ERR_OK) {
    ESP_LOGE(TAG, "esp_h264_enc_hw_new failed");
    return false;
  }
  if (esp_h264_enc_open(this->enc_) != ESP_H264_ERR_OK) {
    ESP_LOGE(TAG, "esp_h264_enc_open failed");
    return false;
  }
  ESP_LOGI(TAG, "Encoder ready");
  return true;
}

void CameraH264Encoder::on_stream_start() {
  this->last_encode_start_us_ = 0;
  if (!this->enc_ && !this->init_encoder_()) {
    ESP_LOGE(TAG, "init_encoder failed");
    return;
  }
  this->streaming_.store(true);
}

void CameraH264Encoder::request_keyframe() {
  this->force_keyframe_next_.store(true);
}

void CameraH264Encoder::reset_encoder_for_keyframe_() {
  if (!this->force_keyframe_next_.exchange(false)) {
    return;
  }
  ESP_LOGI(TAG, "Resetting H.264 encoder for low-latency RTSP PLAY IDR");
  if (this->enc_ != nullptr) {
    esp_h264_enc_close(this->enc_);
    esp_h264_enc_del(this->enc_);
    this->enc_ = nullptr;
  }
  this->last_encode_start_us_ = 0;
  if (!this->init_encoder_()) {
    ESP_LOGE(TAG, "encoder reset for IDR failed");
  }
}

void CameraH264Encoder::on_stream_stop() {
  if (!this->always_on_) {
    this->streaming_.store(false);
  }
}

void CameraH264Encoder::start_encoder_task_() {
  if (this->frame_mutex_ == nullptr) {
    this->frame_mutex_ = xSemaphoreCreateMutex();
  }
  if (this->frame_sem_ == nullptr) {
    this->frame_sem_ = xSemaphoreCreateBinary();
  }
  if (this->frame_mutex_ == nullptr || this->frame_sem_ == nullptr) {
    ESP_LOGE(TAG, "failed to allocate encoder task synchronisation");
    this->mark_failed();
    return;
  }
  if (this->encoder_task_ == nullptr) {
    xTaskCreatePinnedToCore(CameraH264Encoder::encoder_task_trampoline, "h264_enc",
                            K_ENCODER_TASK_STACK_BYTES, this, K_ENCODER_TASK_PRIORITY,
                            &this->encoder_task_, K_ENCODER_TASK_CORE);
    ESP_LOGI(TAG, "Encoder task started; CSI buffer lifetime is held by CameraImage shared ownership");
  }
}

bool CameraH264Encoder::queue_camera_image_(const std::shared_ptr<camera::CameraImage> &image) {
  if (this->frame_mutex_ == nullptr || this->frame_sem_ == nullptr || image == nullptr) {
    return false;
  }
  if (xSemaphoreTake(this->frame_mutex_, 0) != pdTRUE) {
    return false;
  }
  this->pending_image_ = image;
  xSemaphoreGive(this->frame_mutex_);
  xSemaphoreGive(this->frame_sem_);
  return true;
}

void CameraH264Encoder::encoder_task_body_() {
  while (!this->encoder_task_stop_.load()) {
    if (xSemaphoreTake(this->frame_sem_, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    std::shared_ptr<camera::CameraImage> image;
    if (this->frame_mutex_ != nullptr && xSemaphoreTake(this->frame_mutex_, portMAX_DELAY) == pdTRUE) {
      image = std::move(this->pending_image_);
      this->pending_image_.reset();
      xSemaphoreGive(this->frame_mutex_);
    }
    if (image == nullptr) {
      continue;
    }
    const uint8_t *image_data = image->get_data_buffer();
    const size_t image_data_len = image->get_data_length();
    if (image_data == nullptr || image_data_len == 0) {
      continue;
    }
    (void) this->encode_frame_(image_data, image_data_len);
  }
}

bool CameraH264Encoder::should_encode_live_frame_() {
  if (this->fps_ == 0) {
    return true;
  }

  const uint32_t now = micros();
  const uint32_t interval_us = one_second_us() / static_cast<uint32_t>(this->fps_);
  if (this->last_encode_start_us_ == 0) {
    this->last_encode_start_us_ = now;
    return true;
  }

  const uint32_t elapsed = now - this->last_encode_start_us_;
  if (elapsed < interval_us) {
    return false;
  }

  this->last_encode_start_us_ = now;
  return true;
}

void CameraH264Encoder::on_camera_image(
    const std::shared_ptr<camera::CameraImage> &image) {
  if (!this->streaming_.load() || this->enc_ == nullptr || image == nullptr) {
    return;
  }
  if (!this->should_encode_live_frame_()) {
    return;
  }
  (void) this->queue_camera_image_(image);
}

camera::EncoderError CameraH264Encoder::encode_pixels(camera::CameraImageSpec *spec, camera::Buffer *pixels) {
  (void) spec;
  if (pixels == nullptr) {
    return camera::ENCODER_ERROR_SKIP_FRAME;
  }
  return this->encode_frame_(pixels->get_data_buffer(), pixels->get_data_length());
}

camera::EncoderError CameraH264Encoder::encode_frame_(const uint8_t *data, size_t len, bool emit) {
  this->reset_encoder_for_keyframe_();
  if (this->enc_ == nullptr || this->output_.get_data() == nullptr) {
    return camera::ENCODER_ERROR_CONFIGURATION;
  }

  // Input length uses 16-ALIGNED dims (official example pattern):
  // in.len = align16(w) * align16(h) * bpp — buffer must be padded accordingly.
  // The camera component allocates frame buffers with padded height.
  const size_t expected =
      static_cast<size_t>(align16(this->width_)) * align16(this->height_) *
      ESP_H264_GET_BPP_BY_PIC_TYPE(ESP_H264_RAW_FMT_O_UYY_E_VYY);
  if (len < expected) {
    ESP_LOGW(TAG, "Frame too small: %zu < %zu (16-aligned)", len, expected);
    return camera::ENCODER_ERROR_SKIP_FRAME;
  }

  esp_h264_enc_in_frame_t in = {};
  in.raw_data.buffer = const_cast<uint8_t *>(data);
  in.raw_data.len    = static_cast<uint32_t>(expected);
  in.pts             = millis();

  esp_h264_enc_out_frame_t out = {};
  out.raw_data.buffer = this->output_.get_data();
  out.raw_data.len    = static_cast<uint32_t>(this->output_.get_max_size());

  esp_h264_err_t err = esp_h264_enc_process(this->enc_, &in, &out);

  // -3 is the observed esp_h264/HW error for "out buffer too small".
  if (err != ESP_H264_ERR_OK) {
    ESP_LOGW(TAG, "enc_process err=%d out_len=%" PRIu32, static_cast<int>(err), out.raw_data.len);
    return (static_cast<int>(err) == -3) ? camera::ENCODER_ERROR_RETRY_FRAME : camera::ENCODER_ERROR_SKIP_FRAME;
  }
  // NOTE: out.raw_data.len is the CAPACITY we passed in; the driver reports the
  // real bitstream size in out.length ("Actual length of encoder data in bytes",
  // esp_h264_types.h:170). Confusing API — do not read raw_data.len here.
  if (out.length == 0) {
    ESP_LOGW(TAG, "enc_process produced 0 bytes");
    return camera::ENCODER_ERROR_SKIP_FRAME;
  }
  if (out.length > this->output_.get_max_size()) {
    ESP_LOGW(TAG, "enc output overflow (%" PRIu32 ")", out.length);
    return camera::ENCODER_ERROR_RETRY_FRAME;
  }

  uint32_t nal_len = out.length;

  const bool driver_key = (out.frame_type == ESP_H264_FRAME_TYPE_IDR) ||
                          (out.frame_type == ESP_H264_FRAME_TYPE_I);
  const bool idr_nal = annexb_contains_nal_type(this->output_.get_data(), static_cast<size_t>(nal_len), K_H264_NAL_TYPE_IDR);
  const bool is_key = driver_key || idr_nal;
  this->output_.set_encoded_size(nal_len);

  if (emit) {
    this->callbacks_.call(this->output_.get_data(), static_cast<size_t>(nal_len), is_key);
  }
  return camera::ENCODER_ERROR_SUCCESS;
}

#else

bool H264EncoderBuffer::set_buffer_size(size_t size) {
  (void) size;
  return false;
}

bool H264EncoderBuffer::reserve(size_t size, size_t max_size) {
  (void) size;
  (void) max_size;
  return false;
}

void H264EncoderBuffer::release() {
  this->data_ = nullptr;
  this->size_ = 0;
  this->capacity_ = 0;
}

CameraH264Encoder::~CameraH264Encoder() {
  this->encoder_task_stop_.store(true);
  if (this->frame_sem_ != nullptr) {
    xSemaphoreGive(this->frame_sem_);
  }
}

void CameraH264Encoder::setup() {
  ESP_LOGE(TAG, "esp_h264 managed-component headers are not available in this build environment");
  this->mark_failed();
}

void CameraH264Encoder::dump_config() {
  ESP_LOGCONFIG(TAG, "H.264 Encoder: esp_h264 headers unavailable");
}

void CameraH264Encoder::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  (void) image;
}

void CameraH264Encoder::on_stream_start() { this->streaming_ = true; }
void CameraH264Encoder::on_stream_stop() { this->streaming_ = false; }

camera::EncoderError CameraH264Encoder::encode_pixels(camera::CameraImageSpec *spec, camera::Buffer *pixels) {
  (void) spec;
  (void) pixels;
  return camera::ENCODER_ERROR_CONFIGURATION;
}

#endif  // CAMERA_H264_HAS_ESP_H264

}  // namespace esphome::camera_h264
#endif
