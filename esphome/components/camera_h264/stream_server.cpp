#include "esphome/core/defines.h"
#ifdef USE_CAMERA_H264

#include "stream_server.h"
#include "camera_h264.h"
#include "esphome/core/log.h"
#include <lwip/sockets.h>
#include "esphome/components/network/util.h"
#include <cinttypes>
#include <cstring>
#include <esp_heap_caps.h>

namespace esphome::camera_h264 {

static const char *const TAG = "h264_server";

static constexpr size_t K_STREAM_SLOT_ALIGNMENT_BYTES = 64;
static constexpr uint32_t K_SERVER_TASK_STACK_BYTES = 8192;
static constexpr UBaseType_t K_SERVER_TASK_PRIORITY = 5;
static constexpr BaseType_t K_SERVER_TASK_CORE = 0;
static constexpr uint32_t K_BROADCAST_TASK_STACK_BYTES = 8192;
static constexpr UBaseType_t K_BROADCAST_TASK_PRIORITY = 10;
static constexpr BaseType_t K_BROADCAST_TASK_CORE = 0;
static constexpr uint32_t K_NETWORK_WAIT_MS = 500;
static constexpr time_t K_CLIENT_SEND_TIMEOUT_SECONDS = 2;
static constexpr suseconds_t K_CLIENT_SEND_TIMEOUT_US = 0;
static constexpr uint32_t K_ACCEPT_LOOP_DELAY_MS = 50;
static constexpr int K_REUSE_ADDRESS_ENABLED = 1;

static void remove_client_at(StaticVector<int, H264_MAX_CLIENTS> &clients, size_t index) {
  StaticVector<int, H264_MAX_CLIENTS> remaining_clients;
  for (size_t client_index = 0; client_index < clients.size(); client_index++) {
    if (client_index != index) {
      remaining_clients.push_back(clients[client_index]);
    }
  }
  clients = remaining_clients;
}

void H264StreamServer::setup() {
  if (this->encoder_ == nullptr) {
    ESP_LOGE(TAG, "No encoder");
    this->mark_failed();
    return;
  }
  if (this->encoder_->is_failed()) {
    ESP_LOGE(TAG, "Encoder failed");
    this->mark_failed();
    return;
  }

  this->slot_size_ = this->encoder_->get_out_size();
  this->frame_queue_ = xQueueCreate(H264_RAW_STREAM_SLOTS, sizeof(StreamPacket));
  this->slot_free_ = xQueueCreate(H264_RAW_STREAM_SLOTS, sizeof(uint8_t *));
  if (this->frame_queue_ == nullptr || this->slot_free_ == nullptr) {
    ESP_LOGE(TAG, "FreeRTOS object create failed");
    this->mark_failed();
    return;
  }

  for (size_t slot_index = 0; slot_index < H264_RAW_STREAM_SLOTS; slot_index++) {
    this->slots_[slot_index] = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        K_STREAM_SLOT_ALIGNMENT_BYTES, this->slot_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (this->slots_[slot_index] == nullptr) {
      ESP_LOGE(TAG, "slot %zu alloc failed (%zu B)", slot_index, this->slot_size_);
      this->mark_failed();
      return;
    }
    uint8_t *slot = this->slots_[slot_index];
    xQueueSendToBack(this->slot_free_, &slot, 0);
  }

  this->encoder_->add_frame_callback(
      [this](const uint8_t *data, size_t len, bool is_key) { this->push_frame(data, len, is_key); });

  xTaskCreatePinnedToCore(H264StreamServer::server_task_trampoline, "h264_srv", K_SERVER_TASK_STACK_BYTES, this,
                            K_SERVER_TASK_PRIORITY, nullptr, K_SERVER_TASK_CORE);
  xTaskCreatePinnedToCore(H264StreamServer::broadcast_task_trampoline, "h264_bcast",
                            K_BROADCAST_TASK_STACK_BYTES, this, K_BROADCAST_TASK_PRIORITY, nullptr,
                            K_BROADCAST_TASK_CORE);

  ESP_LOGI(TAG, "H.264 raw TCP server setup done, port %" PRIu16 " slots=%zu×%zuB", this->port_,
           H264_RAW_STREAM_SLOTS, this->slot_size_);
}

void H264StreamServer::dump_config() { ESP_LOGCONFIG(TAG, "H.264 Stream Server: port=%" PRIu16, this->port_); }

void H264StreamServer::recycle_slot_(uint8_t *slot) {
  if (slot != nullptr) {
    xQueueSend(this->slot_free_, &slot, 0);
  }
}

void H264StreamServer::push_frame(const uint8_t *data, size_t len, bool is_key) {
  if (this->frame_queue_ == nullptr || this->slot_free_ == nullptr || data == nullptr || len == 0) {
    return;
  }
  if (this->clients_.empty()) {
    return;
  }
  if (len > this->slot_size_) {
    return;
  }

  uint8_t *slot = nullptr;
  if (xQueueReceive(this->slot_free_, &slot, 0) != pdTRUE) {
    return;
  }

  memcpy(slot, data, len);
  StreamPacket packet{slot, len, is_key};
  if (xQueueSend(this->frame_queue_, &packet, 0) == pdTRUE) {
    return;
  }

  // Keep only the latest frame when the broadcaster is behind.
  StreamPacket old_packet{};
  if (xQueueReceive(this->frame_queue_, &old_packet, 0) == pdTRUE) {
    this->recycle_slot_(old_packet.payload);
    if (xQueueSend(this->frame_queue_, &packet, 0) == pdTRUE) {
      return;
    }
  }
  this->recycle_slot_(slot);
}

void H264StreamServer::broadcast_task_() {
  StreamPacket packet{};
  while (true) {
    if (xQueueReceive(this->frame_queue_, &packet, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (this->clients_mutex_.try_lock()) {
      size_t client_index = 0;
      while (client_index < this->clients_.size()) {
        const ssize_t bytes_sent = send(this->clients_[client_index], packet.payload, packet.len, 0);
        if (bytes_sent <= 0) {
          close(this->clients_[client_index]);
          ESP_LOGI(TAG, "Client disconnected (%zu remaining)", this->clients_.size() - 1);
          remove_client_at(this->clients_, client_index);
          if (this->clients_.empty()) {
            this->encoder_->on_stream_stop();
          }
        } else {
          client_index++;
        }
      }
      this->clients_mutex_.unlock();
    }
    this->recycle_slot_(packet.payload);
  }
}

void H264StreamServer::server_task_() {
  ESP_LOGI(TAG, "Waiting for network...");
  while (!network::is_connected()) {
    vTaskDelay(pdMS_TO_TICKS(K_NETWORK_WAIT_MS));
  }

  int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "socket() failed");
    return;
  }

  int reuse_addr = K_REUSE_ADDRESS_ENABLED;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));

  struct sockaddr_in listen_addr = {};
  listen_addr.sin_family = AF_INET;
  listen_addr.sin_addr.s_addr = INADDR_ANY;
  listen_addr.sin_port = htons(this->port_);

  if (bind(listen_fd, reinterpret_cast<const sockaddr *>(&listen_addr), sizeof(listen_addr)) < 0) {
    ESP_LOGE(TAG, "bind() failed");
    close(listen_fd);
    return;
  }
  listen(listen_fd, H264_MAX_CLIENTS);
  ESP_LOGI(TAG, "Listening — ffplay tcp://DEVICE_IP:%" PRIu16, this->port_);

  while (true) {
    int client_fd = accept(listen_fd, nullptr, nullptr);
    if (client_fd >= 0) {
      struct timeval send_timeout = {.tv_sec = K_CLIENT_SEND_TIMEOUT_SECONDS, .tv_usec = K_CLIENT_SEND_TIMEOUT_US};
      setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));

      bool was_empty = false;
      bool accepted = false;
      {
        LockGuard lock(this->clients_mutex_);
        if (this->clients_.size() >= H264_MAX_CLIENTS) {
          ESP_LOGW(TAG, "Max clients, rejecting");
        } else {
          was_empty = this->clients_.empty();
          this->clients_.push_back(client_fd);
          accepted = true;
          ESP_LOGI(TAG, "%zu/%zu client(s) connected", this->clients_.size(), H264_MAX_CLIENTS);
        }
      }
      if (!accepted) {
        close(client_fd);
      } else if (was_empty) {
        this->encoder_->on_stream_start();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(K_ACCEPT_LOOP_DELAY_MS));
  }
}

void H264StreamServer::server_task_trampoline(void *arg) {
  if (arg != nullptr) {
    static_cast<H264StreamServer *>(arg)->server_task_();
  }
  vTaskDelete(nullptr);
}

void H264StreamServer::broadcast_task_trampoline(void *arg) {
  if (arg != nullptr) {
    static_cast<H264StreamServer *>(arg)->broadcast_task_();
  }
  vTaskDelete(nullptr);
}

}  // namespace esphome::camera_h264
#endif
