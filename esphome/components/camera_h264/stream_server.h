#pragma once
#ifdef USE_CAMERA_H264

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

namespace esphome::camera_h264 {

class CameraH264Encoder;

static constexpr size_t H264_MAX_CLIENTS = 3;
static constexpr size_t H264_RAW_STREAM_SLOTS = 2;
static constexpr uint16_t H264_RAW_STREAM_DEFAULT_PORT = 8080;

struct StreamPacket {
  uint8_t *payload;
  size_t len;
  bool is_key;
};

class H264StreamServer : public Component {
 public:
  void set_port(uint16_t port) { this->port_ = port; }
  void set_encoder(CameraH264Encoder *encoder) { this->encoder_ = encoder; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void push_frame(const uint8_t *data, size_t len, bool is_key);

 protected:
  static void server_task_trampoline(void *arg);
  static void broadcast_task_trampoline(void *arg);
  void server_task_();
  void broadcast_task_();
  void recycle_slot_(uint8_t *slot);

  uint16_t port_{H264_RAW_STREAM_DEFAULT_PORT};
  CameraH264Encoder *encoder_{nullptr};

  Mutex clients_mutex_;
  QueueHandle_t frame_queue_{nullptr};  // StreamPacket
  QueueHandle_t slot_free_{nullptr};  // uint8_t *

  uint8_t *slots_[H264_RAW_STREAM_SLOTS]{};
  size_t slot_size_{0};

  StaticVector<int, H264_MAX_CLIENTS> clients_;
};

}  // namespace esphome::camera_h264

#endif
