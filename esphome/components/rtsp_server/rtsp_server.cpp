#include "esphome/core/defines.h"
#ifdef USE_CAMERA_H264

#include "rtsp_server.h"
#include "esphome/components/camera_h264/h264_annexb.h"
#include "esphome/components/camera_h264/h264_units.h"
#include "esphome/core/log.h"
#include "esphome/core/alloc_helpers.h"
#include "esphome/core/helpers.h"

#include <lwip/sockets.h>
#include <lwip/inet.h>
#include "esphome/components/network/util.h"
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_rom_sys.h>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cinttypes>
#include <climits>
#include <cstdlib>
#include <cerrno>
#include <cstdint>
#include <strings.h>

namespace esphome::rtsp_server {

static const char *const TAG = "rtsp_server";
static constexpr uint8_t K_RTCP_SENDER_REPORT_PACKET_TYPE = 200;
static constexpr uint64_t K_NTP_UNIX_OFFSET_SECONDS = 2208988800ULL;
static constexpr uint8_t K_H264_FU_A_NAL_TYPE = 28;
static constexpr uint8_t K_H264_FU_A_HEADER_BYTES = 2;
static constexpr uint8_t K_H264_FU_START_BIT = 0x80;
static constexpr uint8_t K_H264_FU_END_BIT = 0x40;
static constexpr uint32_t K_RTP_CLOCK_HZ = 90000;
static constexpr uint8_t K_DEFAULT_ENCODER_FPS = 25;
static constexpr uint8_t K_BITS_PER_BYTE = 8;
static constexpr uint8_t K_RTP_HEADER_BYTES = 12;
static constexpr uint8_t K_RTP_TCP_INTERLEAVED_HEADER_BYTES = 4;
static constexpr uint8_t K_RTP_TCP_PACKET_HEADER_BYTES = K_RTP_TCP_INTERLEAVED_HEADER_BYTES + K_RTP_HEADER_BYTES;
static constexpr uint8_t K_RTP_TCP_CHANNEL = 0;
static constexpr uint8_t K_RTP_VERSION_2 = 0x80;
static constexpr uint8_t K_RTP_MARKER_BIT = 0x80;
static constexpr uint8_t K_RTP_PAYLOAD_TYPE_H264 = 96;
static constexpr uint8_t K_RTCP_SENDER_REPORT_PACKET_BYTES = 28;
static constexpr uint8_t K_RTCP_SENDER_REPORT_LENGTH_WORDS_MINUS_ONE = 6;
static constexpr uint32_t K_RTCP_SENDER_REPORT_INTERVAL_US =
    camera_h264::duration_to_microseconds(std::chrono::seconds{5});
static constexpr size_t K_RTP_SLOT_ALIGNMENT_BYTES = 64;
static constexpr uint32_t K_SOCKET_SEND_BUFFER_BYTES = camera_h264::kibibytes(16);
static constexpr suseconds_t K_UDP_SOCKET_SEND_TIMEOUT_US = 20000;
static constexpr suseconds_t K_RTSP_SOCKET_SEND_TIMEOUT_US = 100000;
static constexpr uint8_t K_RTSP_LISTEN_BACKLOG = 2;
static constexpr time_t K_RTSP_IDLE_SELECT_TIMEOUT_SECONDS = 1;
static constexpr suseconds_t K_RTSP_ACTIVE_SELECT_TIMEOUT_US = 0;
static constexpr size_t K_RTSP_INTERLEAVED_FRAME_HEADER_BYTES = 4;
static constexpr int K_DEFAULT_CSEQ = 1;
static constexpr uint8_t K_DESCRIBE_PARAMETER_WAIT_ATTEMPTS = 30;
static constexpr uint32_t K_DESCRIBE_PARAMETER_WAIT_MS = 100;
static constexpr size_t K_RTSP_RESPONSE_BUFFER_BYTES = 768;
static constexpr size_t K_SDP_BUFFER_BYTES = 512;
static constexpr size_t K_PROFILE_LEVEL_ID_BUFFER_BYTES = 8;
static constexpr uint16_t K_UDP_RTCP_PORT_OFFSET = 1;
static constexpr uint16_t K_UDP_PACING_BURST_RTP_PACKETS = 4;
static constexpr uint16_t K_ESTIMATED_RTP_UDP_IP_OVERHEAD_BYTES = 40;
static constexpr uint16_t K_UDP_IP_OVERHEAD_BYTES = 28;
static constexpr uint32_t K_UDP_KBPS_US_PER_BYTE = 8000;
static constexpr uint32_t K_UDP_PACING_TASK_DELAY_THRESHOLD_US = 2000;
static constexpr uint32_t K_UDP_SEND_RETRY_COUNT = 10;
static constexpr uint8_t K_UDP_FAST_RETRY_COUNT = 3;
static constexpr uint32_t K_UDP_FAST_RETRY_DELAY_MS = 1;
static constexpr uint32_t K_UDP_SLOW_RETRY_DELAY_MS = 2;
static constexpr uint32_t K_PERIODIC_FRAME_LOG_INTERVAL = 100;
static constexpr uint32_t K_STALE_FRAME_LOG_THRESHOLD_MS = 500;
static constexpr uint32_t K_SLOW_SEND_LOG_THRESHOLD_MS = 80;
static constexpr uint32_t K_SERVER_TASK_STACK_BYTES = 8192;
static constexpr UBaseType_t K_SERVER_TASK_PRIORITY = 5;
static constexpr BaseType_t K_SERVER_TASK_CORE = 0;
using camera_h264::H264AnnexBNal;
using camera_h264::K_H264_NAL_TYPE_IDR;
using camera_h264::K_H264_NAL_TYPE_PPS;
using camera_h264::K_H264_NAL_TYPE_SPS;
using camera_h264::annexb_contains_nal_type;
using camera_h264::next_annexb_nal;
using camera_h264::one_millisecond_us;
using camera_h264::one_second_ms;
using camera_h264::one_second_us;

static constexpr uint8_t K_H264_NAL_HEADER_BYTES = 1;

static const char *transport_name(RtspTransport transport) {
  return transport == RtspTransport::UDP ? "udp" : "tcp";
}


// ── writev that handles partial writes ───────────────────────────────────────
static bool send_iov_all(int fd, struct iovec *segments, int segment_count) {
  while (segment_count > 0) {
    ssize_t bytes_written = lwip_writev(fd, segments, segment_count);  // libc writev not linked here
    if (bytes_written <= 0) {
      return false;
    }

    while (bytes_written > 0 && segment_count > 0) {
      if (static_cast<size_t>(bytes_written) >= segments[0].iov_len) {
        bytes_written -= segments[0].iov_len;
        segments++;
        segment_count--;
      } else {
        segments[0].iov_base = static_cast<uint8_t *>(segments[0].iov_base) + bytes_written;
        segments[0].iov_len -= bytes_written;
        bytes_written = 0;
      }
    }
  }
  return true;
}

template<typename T> static void write_big_endian(uint8_t *dst, T value) {
  const T big_endian_value = convert_big_endian(value);
  memcpy(dst, &big_endian_value, sizeof(big_endian_value));
}

void H264RtspServer::ntp_from_us(uint64_t us, uint32_t *msw, uint32_t *lsw) {
  const uint64_t sec = us / one_second_us();
  const uint64_t frac_us = us % one_second_us();
  if (msw != nullptr) {
    *msw = static_cast<uint32_t>(sec + K_NTP_UNIX_OFFSET_SECONDS);
  }
  if (lsw != nullptr) {
    *lsw = static_cast<uint32_t>((frac_us << 32ULL) / one_second_us());
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────

void H264RtspServer::setup() {
  if (!this->encoder_ || this->encoder_->is_failed()) {
    ESP_LOGE(TAG, "encoder not ready");
    this->mark_failed();
    return;
  }

  this->frame_queue_ = xQueueCreate(this->frame_slots_, sizeof(RtspFrame));
  this->slot_free_   = xQueueCreate(this->frame_slots_, sizeof(uint8_t *));
  if (!this->frame_queue_ || !this->slot_free_) {
    ESP_LOGE(TAG, "queue/mutex alloc failed");
    this->mark_failed();
    return;
  }

  // Slot pool: sized to encoder output buffer, 64-byte aligned PSRAM.
  this->slot_size_ = this->encoder_->get_out_size();
  if (this->slot_size_ == 0) {
    ESP_LOGE(TAG, "encoder output buffer size is 0");
    this->mark_failed();
    return;
  }
  for (size_t slot_index = 0; slot_index < this->frame_slots_; slot_index++) {
    this->slots_[slot_index] = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(K_RTP_SLOT_ALIGNMENT_BYTES, this->slot_size_,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!this->slots_[slot_index]) {
      ESP_LOGE(TAG, "slot %zu alloc failed (%zu B)", slot_index, this->slot_size_);
      this->mark_failed();
      return;
    }
    uint8_t *free_slot = this->slots_[slot_index];
    xQueueSendToBack(this->slot_free_, &free_slot, 0);
  }


  if (!this->username_.empty()) {
    char credential[RTSP_AUTH_CREDENTIAL_MAX];
    int credential_len = snprintf(credential, sizeof(credential), "%s:%s",
                                  this->username_.c_str(), this->password_.c_str());
    if (credential_len < 0 || static_cast<size_t>(credential_len) >= sizeof(credential)) {
      ESP_LOGE(TAG, "RTSP username/password too long");
      this->mark_failed();
      return;
    }
    this->auth_b64_ = base64_encode(reinterpret_cast<const uint8_t *>(credential),
                                    static_cast<size_t>(credential_len));
  }

  this->encoder_->add_frame_callback(
      [this](const uint8_t *data, size_t len, bool is_key) { this->on_encoded_frame_(data, len, is_key); });

  xTaskCreatePinnedToCore(H264RtspServer::server_task_trampoline, "rtsp_srv",
                          K_SERVER_TASK_STACK_BYTES, this, K_SERVER_TASK_PRIORITY, nullptr,
                          K_SERVER_TASK_CORE);
  ESP_LOGI(TAG,
           "RTSP on port %u  transport=%s  slots=%u×%zuB  copy=sync memcpy  rtp_payload=%u  udp_bandwidth_kbps=%" PRIu32 "  live_budget=%" PRIu32 "ms  force_keyframe_on_play=%s",
           this->port_, transport_name(this->transport_), this->frame_slots_, this->slot_size_,
           this->rtp_payload_size_, this->udp_bandwidth_kbps_, this->frame_period_ms_(),
           YESNO(this->force_keyframe_on_play_));
}

void H264RtspServer::dump_config() {
  ESP_LOGCONFIG(TAG, "RTSP Server: rtsp://%s:****@<ip>:%u/stream",
                this->username_.c_str(), this->port_);
  ESP_LOGCONFIG(TAG, "  Transport: %s", transport_name(this->transport_));
  ESP_LOGCONFIG(TAG, "  RTP payload size: %u", this->rtp_payload_size_);
  ESP_LOGCONFIG(TAG, "  UDP bandwidth budget: %" PRIu32 " kbit/s", this->udp_bandwidth_kbps_);
  ESP_LOGCONFIG(TAG, "  Live frame budget: %" PRIu32 " ms", this->frame_period_ms_());
  ESP_LOGCONFIG(TAG, "  Force keyframe on PLAY: %s", YESNO(this->force_keyframe_on_play_));
}

// ── Encoder → pool slot (low-RAM encoded-AU copy) ────────────────────────────────

void H264RtspServer::queue_or_replace_frame_(RtspFrame frame) {
  if (xQueueSend(this->frame_queue_, &frame, 0) == pdTRUE) {
    return;
  }

  // Low-latency policy: never grow a FIFO. If the queued frame is a keyframe
  // and the new one is a P-frame, keep the keyframe so clients do not wait an
  // extra GOP. Otherwise replace old with new and recycle old's slot.
  RtspFrame old{};
  if (xQueueReceive(this->frame_queue_, &old, 0) == pdTRUE) {
    if (old.has_idr && !frame.has_idr) {
      xQueueSend(this->frame_queue_, &old, 0);
      xQueueSend(this->slot_free_, &frame.data, 0);
      return;
    }
    xQueueSend(this->slot_free_, &old.data, 0);
    if (xQueueSend(this->frame_queue_, &frame, 0) == pdTRUE) {
      return;
    }
  }
  xQueueSend(this->slot_free_, &frame.data, 0);
}


void H264RtspServer::purge_queued_frames_() {
  if (this->frame_queue_ == nullptr || this->slot_free_ == nullptr) {
    return;
  }
  RtspFrame dropped{};
  uint8_t count = 0;
  while (xQueueReceive(this->frame_queue_, &dropped, 0) == pdTRUE) {
    xQueueSend(this->slot_free_, &dropped.data, 0);
    count++;
  }
  if (count != 0) {
    ESP_LOGD(TAG, "purged %u queued RTSP frame(s) before PLAY", count);
  }
}

bool H264RtspServer::any_playing_client_waits_for_keyframe_() {
  // RTSP task owns client state; encoder callbacks only enqueue frames.
  for (auto &c : this->clients_) {
    if (c.fd >= 0 && c.state == RtspState::PLAYING && c.needs_keyframe) {
      return true;
    }
  }
  return false;
}

bool H264RtspServer::should_drop_outside_live_budget_(const RtspFrame &frame, bool client_needs_keyframe) const {
  const uint32_t frame_period_ms = this->frame_period_ms_();
  if (frame_period_ms == 0) {
    return false;
  }

  const uint32_t age_ms = micros_to_millis(micros() - frame.capture_us);
  const uint32_t estimated_send_ms = this->estimate_udp_send_ms_(frame.len);
  const bool outside_budget = age_ms > frame_period_ms || estimated_send_ms > frame_period_ms;
  if (!outside_budget) {
    return false;
  }

  if (frame.has_idr && client_needs_keyframe) {
    ESP_LOGD(TAG, "sending oversized IDR for decoder recovery age=%" PRIu32
                  "ms estimated_send=%" PRIu32 "ms frame_period=%" PRIu32
                  "ms len=%zu udp_bw=%" PRIu32 "kbit/s",
             age_ms, estimated_send_ms, frame_period_ms, frame.len, this->udp_bandwidth_kbps_);
    return false;
  }

  ESP_LOGD(TAG, "dropping frame outside live budget key=%d age=%" PRIu32
                "ms estimated_send=%" PRIu32 "ms frame_period=%" PRIu32
                "ms len=%zu udp_bw=%" PRIu32 "kbit/s",
           static_cast<int>(frame.has_idr), age_ms, estimated_send_ms, frame_period_ms,
           frame.len, this->udp_bandwidth_kbps_);
  return true;
}

void H264RtspServer::on_encoded_frame_(const uint8_t *data, size_t len, bool is_key) {
  const uint32_t frame_ready_us = micros();
  if (is_key && !this->have_params_.load()) {
    this->extract_sps_pps_(data, len);
  }
  if (!this->streaming_.load()) {
    return;
  }
  if (len > this->slot_size_) {
    return;
  }

  uint8_t *slot = nullptr;
  if (xQueueReceive(this->slot_free_, &slot, 0) != pdTRUE) {
    return;  // no slot — drop
  }


  const uint32_t rtp_timestamp = static_cast<uint32_t>(
      static_cast<uint64_t>(frame_ready_us) * K_RTP_CLOCK_HZ / one_second_us());
  const bool has_idr = annexb_contains_nal_type(data, len, K_H264_NAL_TYPE_IDR);
  RtspFrame frame{slot, len, is_key || has_idr, has_idr, rtp_timestamp, frame_ready_us};

  // Keep this copy synchronous: data points at the encoder output buffer,
  // which may be reused by the next encode as soon as this callback returns.
  // Async GDMA would need a separate source buffer or an explicit completion
  // wait, which removes most of the benefit for these already-compressed AUs.
  memcpy(slot, data, len);
  this->queue_or_replace_frame_(frame);
}

void H264RtspServer::extract_sps_pps_(const uint8_t *au, size_t len) {
  const uint8_t *cursor = au;
  const uint8_t *end = au + len;
  H264AnnexBNal nal;
  while (next_annexb_nal(&cursor, end, &nal)) {
    if (nal.type() == K_H264_NAL_TYPE_SPS && nal.len <= SPS_PPS_MAX) {
      memcpy(this->sps_, nal.data, nal.len);
      this->sps_len_ = nal.len;
    } else if (nal.type() == K_H264_NAL_TYPE_PPS && nal.len <= SPS_PPS_MAX) {
      memcpy(this->pps_, nal.data, nal.len);
      this->pps_len_ = nal.len;
    }

    if (this->sps_len_ != 0 && this->pps_len_ != 0) {
      this->have_params_.store(true);
      ESP_LOGI(TAG, "SPS(%zu)/PPS(%zu) captured", this->sps_len_, this->pps_len_);
      return;
    }
  }
}

// ── UDP transport helpers ───────────────────────────────────────────────────

static const char *find_case_insensitive(const char *haystack, const char *needle) {
  if (haystack == nullptr || needle == nullptr || needle[0] == '\0') {
    return haystack;
  }
  const size_t needle_len = strlen(needle);
  for (const char *candidate = haystack; *candidate != '\0'; candidate++) {
    if (strncasecmp(candidate, needle, needle_len) == 0) {
      return candidate;
    }
  }
  return nullptr;
}

static bool rtsp_transport_requests_tcp(const char *req) {
  return find_case_insensitive(req, "RTP/AVP/TCP") != nullptr ||
         find_case_insensitive(req, "interleaved=") != nullptr;
}

static bool rtsp_transport_requests_udp(const char *req) {
  return find_case_insensitive(req, "client_port=") != nullptr &&
         !rtsp_transport_requests_tcp(req);
}

bool H264RtspServer::parse_client_rtp_port(const char *req, uint16_t *port) {
  const char *p = find_case_insensitive(req, "client_port=");
  if (p == nullptr || port == nullptr) {
    return false;
  }
  p += strlen("client_port=");
  char *end = nullptr;
  errno = 0;
  const int64_t parsed = strtoll(p, &end, 10);
  if (end == p || errno != 0 || parsed <= 0 || parsed >= UINT16_MAX) {
    return false;
  }
  *port = static_cast<uint16_t>(parsed);
  return true;
}

bool H264RtspServer::open_client_udp_socket_(RtspClient &c) {
  this->close_client_udp_socket_(c);

  int rtp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  int rtcp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (rtp_fd < 0 || rtcp_fd < 0) {
    ESP_LOGE(TAG, "udp socket() failed: errno=%d", errno);
    if (rtp_fd >= 0) {
      close(rtp_fd);
    }
    if (rtcp_fd >= 0) {
      close(rtcp_fd);
    }
    return false;
  }

  struct timeval udp_send_timeout = {.tv_sec = 0, .tv_usec = K_UDP_SOCKET_SEND_TIMEOUT_US};
  setsockopt(rtp_fd, SOL_SOCKET, SO_SNDTIMEO, &udp_send_timeout, sizeof(udp_send_timeout));
  setsockopt(rtcp_fd, SOL_SOCKET, SO_SNDTIMEO, &udp_send_timeout, sizeof(udp_send_timeout));
  int socket_send_buffer_bytes = K_SOCKET_SEND_BUFFER_BYTES;
  setsockopt(rtp_fd, SOL_SOCKET, SO_SNDBUF, &socket_send_buffer_bytes, sizeof(socket_send_buffer_bytes));
  setsockopt(rtcp_fd, SOL_SOCKET, SO_SNDBUF, &socket_send_buffer_bytes, sizeof(socket_send_buffer_bytes));

  sockaddr_in local_rtp = {};
  local_rtp.sin_family = AF_INET;
  local_rtp.sin_addr.s_addr = htonl(INADDR_ANY);
  local_rtp.sin_port = 0;
  if (bind(rtp_fd, reinterpret_cast<sockaddr *>(&local_rtp), sizeof(local_rtp)) != 0) {
    ESP_LOGE(TAG, "udp rtp bind() failed: errno=%d", errno);
    close(rtp_fd);
    close(rtcp_fd);
    return false;
  }

  socklen_t rtp_len = sizeof(local_rtp);
  if (getsockname(rtp_fd, reinterpret_cast<sockaddr *>(&local_rtp), &rtp_len) != 0) {
    ESP_LOGE(TAG, "udp rtp getsockname() failed: errno=%d", errno);
    close(rtp_fd);
    close(rtcp_fd);
    return false;
  }

  sockaddr_in local_rtcp = {};
  local_rtcp.sin_family = AF_INET;
  local_rtcp.sin_addr.s_addr = htonl(INADDR_ANY);
  local_rtcp.sin_port = 0;
  if (bind(rtcp_fd, reinterpret_cast<sockaddr *>(&local_rtcp), sizeof(local_rtcp)) != 0) {
    ESP_LOGE(TAG, "udp rtcp bind() failed: errno=%d", errno);
    close(rtp_fd);
    close(rtcp_fd);
    return false;
  }

  socklen_t rtcp_len = sizeof(local_rtcp);
  if (getsockname(rtcp_fd, reinterpret_cast<sockaddr *>(&local_rtcp), &rtcp_len) != 0) {
    ESP_LOGE(TAG, "udp rtcp getsockname() failed: errno=%d", errno);
    close(rtp_fd);
    close(rtcp_fd);
    return false;
  }

  const uint16_t server_rtp_port = ntohs(local_rtp.sin_port);
  const uint16_t server_rtcp_port = ntohs(local_rtcp.sin_port);
  if (server_rtp_port == 0 || server_rtcp_port == 0) {
    ESP_LOGE(TAG, "invalid UDP source ports rtp=%u rtcp=%u", server_rtp_port, server_rtcp_port);
    close(rtp_fd);
    close(rtcp_fd);
    return false;
  }

  sockaddr_in rtcp_addr = c.udp_rtp_addr;
  rtcp_addr.sin_port = htons(static_cast<uint16_t>(c.udp_rtp_port + K_UDP_RTCP_PORT_OFFSET));
  if (connect(rtp_fd, reinterpret_cast<sockaddr *>(&c.udp_rtp_addr), sizeof(c.udp_rtp_addr)) != 0 ||
      connect(rtcp_fd, reinterpret_cast<sockaddr *>(&rtcp_addr), sizeof(rtcp_addr)) != 0) {
    ESP_LOGE(TAG, "udp connect() failed: errno=%d", errno);
    close(rtp_fd);
    close(rtcp_fd);
    return false;
  }

  c.udp_rtp_fd = rtp_fd;
  c.udp_server_port = server_rtp_port;
  c.udp_rtcp_fd = rtcp_fd;
  c.udp_rtcp_port = server_rtcp_port;
  c.udp_bucket_last_us = 0;
  c.udp_bucket_bytes = 0;
  c.rtp_packet_count = 0;
  c.rtp_octet_count = 0;
  c.last_rtcp_sr_us = 0;
  ESP_LOGI(TAG, "UDP RTP connected %s:%u from source port %u; RTCP source port %u",
           inet_ntoa(c.udp_rtp_addr.sin_addr), c.udp_rtp_port, c.udp_server_port, c.udp_rtcp_port);
  return true;
}

void H264RtspServer::close_client_udp_socket_(RtspClient &c) {
  if (c.udp_rtp_fd >= 0) {
    close(c.udp_rtp_fd);
  }
  if (c.udp_rtcp_fd >= 0) {
    close(c.udp_rtcp_fd);
  }
  c.udp_rtp_fd = -1;
  c.udp_server_port = 0;
  c.udp_rtcp_fd = -1;
  c.udp_rtcp_port = 0;
  c.udp_bucket_last_us = 0;
  c.udp_bucket_bytes = 0;
  c.rtp_packet_count = 0;
  c.rtp_octet_count = 0;
  c.last_rtcp_sr_us = 0;
}

// ── Server task ──────────────────────────────────────────────────────────────

void H264RtspServer::server_task_() {
  while (!network::is_connected()) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
    vTaskDelete(nullptr);
    return;
  }
  int reuse_addr = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->port_);
  if (bind(listen_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0 ||
      listen(listen_fd, K_RTSP_LISTEN_BACKLOG) != 0) {
    ESP_LOGE(TAG, "bind/listen failed on %u", this->port_);
    close(listen_fd);
    vTaskDelete(nullptr);
    return;
  }
  ESP_LOGI(TAG, "Listening rtsp://<ip>:%u/stream", this->port_);

  while (true) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(listen_fd, &read_fds);
    int max_fd = listen_fd;
    for (auto &c : this->clients_) {
      if (c.fd >= 0) {
        FD_SET(c.fd, &read_fds);
        if (c.fd > max_fd) {
          max_fd = c.fd;
        }
      }
    }

    const bool streaming = this->streaming_.load();
    struct timeval select_timeout = streaming ?
        timeval{.tv_sec = 0, .tv_usec = K_RTSP_ACTIVE_SELECT_TIMEOUT_US} :
        timeval{.tv_sec = K_RTSP_IDLE_SELECT_TIMEOUT_SECONDS, .tv_usec = 0};
    const int ready = select(max_fd + 1, &read_fds, nullptr, nullptr, &select_timeout);

    if (ready > 0 && FD_ISSET(listen_fd, &read_fds)) {
      int client_fd = accept(listen_fd, nullptr, nullptr);
      if (client_fd >= 0) {
        struct timeval rtsp_send_timeout = {.tv_sec = 0, .tv_usec = K_RTSP_SOCKET_SEND_TIMEOUT_US};
        setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &rtsp_send_timeout, sizeof(rtsp_send_timeout));
        int socket_send_buffer_bytes = K_SOCKET_SEND_BUFFER_BYTES;
        setsockopt(client_fd, SOL_SOCKET, SO_SNDBUF, &socket_send_buffer_bytes, sizeof(socket_send_buffer_bytes));
        int tcp_nodelay = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &tcp_nodelay, sizeof(tcp_nodelay));
        bool placed = false;
        for (auto &c : this->clients_) {
          if (c.fd < 0) {
            c = RtspClient{};
            c.fd = client_fd;
            c.session_id = micros();
            placed = true;
            break;
          }
        }
        if (!placed) {
          close(client_fd);
          ESP_LOGW(TAG, "max clients");
        } else {
          ESP_LOGI(TAG, "RTSP client connected");
        }
      }
    }

    if (ready > 0) {
      for (auto &c : this->clients_) {
        if (c.fd < 0 || !FD_ISSET(c.fd, &read_fds)) {
          continue;
        }
        int received_bytes = recv(c.fd, c.rx + c.rx_len, sizeof(c.rx) - c.rx_len - 1, MSG_DONTWAIT);
        if (received_bytes <= 0) {
          this->close_client_(c);
          continue;
        }
        c.rx_len += received_bytes;
        c.rx[c.rx_len] = 0;

        // Skip '$'-framed RTCP from client.
        while (c.rx_len >= K_RTSP_INTERLEAVED_FRAME_HEADER_BYTES && c.rx[0] == '$') {
          uint16_t interleaved_payload_len = encode_uint16(static_cast<uint8_t>(c.rx[2]), static_cast<uint8_t>(c.rx[3]));
          size_t interleaved_frame_len = K_RTSP_INTERLEAVED_FRAME_HEADER_BYTES + interleaved_payload_len;
          if (c.rx_len < interleaved_frame_len) {
            break;
          }
          memmove(c.rx, c.rx + interleaved_frame_len, c.rx_len - interleaved_frame_len);
          c.rx_len -= interleaved_frame_len;
          c.rx[c.rx_len] = 0;
        }

        if (strstr(c.rx, "\r\n\r\n")) {
          this->handle_request_(c);
          c.rx_len = 0;
        } else if (c.rx_len >= sizeof(c.rx) - 1) {
          c.rx_len = 0;
        }
      }
    }

    if (this->streaming_.load()) {
      const uint32_t frame_period_ms = this->frame_period_ms_();
      const TickType_t frame_wait_ticks = pdMS_TO_TICKS(frame_period_ms == 0 ? 1U : frame_period_ms);
      this->send_next_frame_(frame_wait_ticks);
    }
  }
}

// ── RTSP handlers ────────────────────────────────────────────────────────────

void H264RtspServer::send_response_(int fd, const char *fmt, ...) {
  char buf[K_RTSP_RESPONSE_BUFFER_BYTES];
  va_list ap;
  va_start(ap, fmt);
  int response_len = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (response_len > 0) {
    const size_t len = static_cast<size_t>(response_len) < sizeof(buf) ? static_cast<size_t>(response_len) : sizeof(buf) - 1;
    send(fd, buf, len, 0);
  }
}

bool H264RtspServer::check_auth_(const char *req) {
  if (this->username_.empty()) {
    return true;
  }
  const char *a = strstr(req, "Authorization: Basic ");
  if (a == nullptr) {
    return false;
  }
  a += strlen("Authorization: Basic ");
  return !this->auth_b64_.empty() && strncmp(a, this->auth_b64_.c_str(), this->auth_b64_.size()) == 0 &&
         (a[this->auth_b64_.size()] == '\r' || a[this->auth_b64_.size()] == '\n' ||
          a[this->auth_b64_.size()] == '\0' || a[this->auth_b64_.size()] == ' ');
}

void H264RtspServer::handle_request_(RtspClient &c) {
  int cseq = K_DEFAULT_CSEQ;
  const char *cseq_header = strstr(c.rx, "CSeq:");
  if (cseq_header != nullptr) {
    char *end = nullptr;
    const int64_t parsed = strtoll(cseq_header + strlen("CSeq:"), &end, 10);
    if (end != cseq_header + strlen("CSeq:") && parsed > 0 && parsed <= INT_MAX) {
      cseq = static_cast<int>(parsed);
    }
  }

  if (strncmp(c.rx, "OPTIONS", 7) == 0) {
    this->handle_options_(c, cseq);
  } else if (strncmp(c.rx, "DESCRIBE", 8) == 0) {
    this->handle_describe_(c, cseq);
  } else if (strncmp(c.rx, "SETUP", 5) == 0) {
    this->handle_setup_(c, cseq, c.rx);
  } else if (strncmp(c.rx, "PLAY", 4) == 0) {
    this->handle_play_(c, cseq);
  } else if (strncmp(c.rx, "TEARDOWN", 8) == 0) {
    this->handle_teardown_(c, cseq);
  } else {
    this->send_response_(c.fd, "RTSP/1.0 405 Method Not Allowed\r\nCSeq: %d\r\n\r\n", cseq);
  }
}

void H264RtspServer::handle_options_(RtspClient &c, int cseq) {
  this->send_response_(c.fd,
      "RTSP/1.0 200 OK\r\nCSeq: %d\r\n"
      "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n\r\n", cseq);
}

void H264RtspServer::handle_describe_(RtspClient &c, int cseq) {
  if (!this->check_auth_(c.rx)) {
    this->send_response_(c.fd,
        "RTSP/1.0 401 Unauthorized\r\nCSeq: %d\r\n"
        "WWW-Authenticate: Basic realm=\"esp32\"\r\n\r\n", cseq);
    return;
  }

  if (!this->have_params_.load()) {
    this->encoder_->on_stream_start();
    for (uint8_t attempt = 0; attempt < K_DESCRIBE_PARAMETER_WAIT_ATTEMPTS && !this->have_params_.load(); attempt++) {
      vTaskDelay(pdMS_TO_TICKS(K_DESCRIBE_PARAMETER_WAIT_MS));
    }
  }
  if (!this->have_params_.load()) {
    this->send_response_(c.fd, "RTSP/1.0 503 Service Unavailable\r\nCSeq: %d\r\n\r\n", cseq);
    return;
  }

  const std::string sps_b64 = base64_encode(this->sps_, this->sps_len_);
  const std::string pps_b64 = base64_encode(this->pps_, this->pps_len_);
  char profile_level_id[K_PROFILE_LEVEL_ID_BUFFER_BYTES];
  snprintf(profile_level_id, sizeof(profile_level_id), "%02X%02X%02X", this->sps_[1], this->sps_[2], this->sps_[3]);

  char sdp[K_SDP_BUFFER_BYTES];
  int sdp_len = snprintf(sdp, sizeof(sdp),
      "v=0\r\n"
      "o=- 0 0 IN IP4 0.0.0.0\r\n"
      "s=ESP32-P4\r\n"
      "t=0 0\r\n"
      "m=video 0 RTP/AVP 96\r\n"
      "c=IN IP4 0.0.0.0\r\n"
      "a=rtpmap:96 H264/90000\r\n"
      "a=fmtp:96 packetization-mode=1;profile-level-id=%s;sprop-parameter-sets=%s,%s\r\n"
      "a=control:track0\r\n",
      profile_level_id, sps_b64.c_str(), pps_b64.c_str());

  this->send_response_(c.fd,
      "RTSP/1.0 200 OK\r\nCSeq: %d\r\n"
      "Content-Type: application/sdp\r\nContent-Length: %d\r\n\r\n%s",
      cseq, sdp_len, sdp);
}

void H264RtspServer::handle_setup_(RtspClient &c, int cseq, const char *req) {
  if (!this->check_auth_(req)) {
    this->send_response_(c.fd,
        "RTSP/1.0 401 Unauthorized\r\nCSeq: %d\r\n"
        "WWW-Authenticate: Basic realm=\"esp32\"\r\n\r\n", cseq);
    return;
  }

  if (this->transport_ == RtspTransport::TCP) {
    if (!rtsp_transport_requests_tcp(req)) {
      ESP_LOGW(TAG, "SETUP transport rejected: configured=tcp request=%.180s", req);
      this->send_response_(c.fd, "RTSP/1.0 461 Unsupported Transport\r\nCSeq: %d\r\n\r\n", cseq);
      return;
    }
    c.transport = RtspTransport::TCP;
    c.state = RtspState::READY;
    this->send_response_(c.fd,
        "RTSP/1.0 200 OK\r\nCSeq: %d\r\n"
        "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n"
        "Session: %08" PRIX32 "\r\n\r\n", cseq, c.session_id);
    return;
  }

  uint16_t client_port = 0;
  if (!rtsp_transport_requests_udp(req) || !H264RtspServer::parse_client_rtp_port(req, &client_port)) {
    ESP_LOGW(TAG, "SETUP transport rejected: configured=udp request=%.180s", req);
    this->send_response_(c.fd, "RTSP/1.0 461 Unsupported Transport\r\nCSeq: %d\r\n\r\n", cseq);
    return;
  }
  sockaddr_in peer = {};
  socklen_t peer_len = sizeof(peer);
  if (getpeername(c.fd, reinterpret_cast<sockaddr *>(&peer), &peer_len) != 0) {
    this->send_response_(c.fd, "RTSP/1.0 500 Internal Server Error\r\nCSeq: %d\r\n\r\n", cseq);
    return;
  }

  c.transport = RtspTransport::UDP;
  c.udp_rtp_port = client_port;
  c.udp_rtp_addr = peer;
  c.udp_rtp_addr.sin_port = htons(client_port);
  if (!this->open_client_udp_socket_(c)) {
    this->send_response_(c.fd, "RTSP/1.0 500 Internal Server Error\r\nCSeq: %d\r\n\r\n", cseq);
    return;
  }

  c.state = RtspState::READY;
  ESP_LOGI(TAG, "UDP SETUP: client RTP port %u, server RTP port %u RTCP port %u",
           client_port, c.udp_server_port, c.udp_rtcp_port);
  this->send_response_(c.fd,
      "RTSP/1.0 200 OK\r\nCSeq: %d\r\n"
      "Transport: RTP/AVP;unicast;client_port=%u-%u;server_port=%u-%u;ssrc=%08" PRIX32 "\r\n"
      "Session: %08" PRIX32 "\r\n\r\n",
      cseq, client_port, static_cast<uint16_t>(client_port + K_UDP_RTCP_PORT_OFFSET),
      c.udp_server_port, c.udp_rtcp_port, this->ssrc_, c.session_id);

}

void H264RtspServer::handle_play_(RtspClient &c, int cseq) {
  if (c.state != RtspState::READY && c.state != RtspState::PLAYING) {
    this->send_response_(c.fd, "RTSP/1.0 455 Method Not Valid\r\nCSeq: %d\r\n\r\n", cseq);
    return;
  }
  this->purge_queued_frames_();
  c.needs_keyframe = true;
  this->send_response_(c.fd,
      "RTSP/1.0 200 OK\r\nCSeq: %d\r\nSession: %08" PRIX32 "\r\n"
      "Range: npt=0.000-\r\n\r\n",
      cseq, c.session_id);
  c.state = RtspState::PLAYING;
  this->streaming_.store(true);
  this->encoder_->on_stream_start();
  const bool warm = this->encoder_ != nullptr && this->encoder_->is_always_on();
  const bool force_idr = this->force_keyframe_on_play_ && !warm;
  if (force_idr) {
    this->encoder_->request_keyframe();
  }
  ESP_LOGI(TAG, "Client PLAYING waiting_for_keyframe=YES forced_idr=%s%s%s",
           YESNO(force_idr), warm ? " always_on_pre_roll=YES" : "",
           warm && this->force_keyframe_on_play_ ? " warm_wait_natural_idr=YES" : "");
}

void H264RtspServer::handle_teardown_(RtspClient &c, int cseq) {
  this->send_response_(c.fd, "RTSP/1.0 200 OK\r\nCSeq: %d\r\n\r\n", cseq);
  this->close_client_(c);
}

void H264RtspServer::close_client_(RtspClient &c) {
  this->close_client_udp_socket_(c);
  if (c.fd >= 0) {
    close(c.fd);
  }
  c.fd = -1;
  c.state = RtspState::INIT;

  bool any = false;
  for (auto &o : this->clients_) {
    if (o.fd >= 0 && o.state == RtspState::PLAYING) {
      any = true;
      break;
    }
  }
  if (!any) {
    this->streaming_.store(false);
    this->encoder_->on_stream_stop();
    ESP_LOGI(TAG, "No RTSP clients — encoder stopped");
  }
}

// ── Sender: pool slot → RTP writev → return slot ─────────────────────────────

bool H264RtspServer::send_next_frame_(TickType_t wait_ticks) {
  RtspFrame frame;
  if (xQueueReceive(this->frame_queue_, &frame, wait_ticks) != pdTRUE) {
    return false;
  }

  const bool client_needs_keyframe = this->any_playing_client_waits_for_keyframe_();

  // Keep the newest completed frame. Only prefer an older IDR when a client
  // is waiting for decoder recovery; otherwise a large stale IDR is just
  // extra latency and pbuf pressure.
  RtspFrame newer{};
  while (xQueueReceive(this->frame_queue_, &newer, 0) == pdTRUE) {
    if (client_needs_keyframe && frame.has_idr && !newer.has_idr) {
      xQueueSendToBack(this->slot_free_, &newer.data, 0);
    } else {
      xQueueSendToBack(this->slot_free_, &frame.data, 0);
      frame = newer;
    }
  }

  if (!frame.has_idr && client_needs_keyframe) {
    ESP_LOGD(TAG, "dropping frame without IDR while client waits for keyframe len=%zu key=%d", frame.len,
             static_cast<int>(frame.is_key));
    xQueueSendToBack(this->slot_free_, &frame.data, 0);
    return true;
  }

  if (this->should_drop_outside_live_budget_(frame, client_needs_keyframe)) {
    xQueueSendToBack(this->slot_free_, &frame.data, 0);
    return true;
  }

  bool sent_to_all_clients = true;
  const uint32_t send_start_us = micros();
  for (auto &c : this->clients_) {
    if (c.fd < 0 || c.state != RtspState::PLAYING) {
      continue;
    }
    if (!this->send_frame_rtp_(c, frame.data, frame.len, frame.has_idr, frame.rtp_timestamp)) {
      sent_to_all_clients = false;
    }
  }
  const uint32_t send_us = micros() - send_start_us;
  const uint32_t send_ms = micros_to_millis(send_us);
  if (!sent_to_all_clients && this->encoder_ != nullptr && !this->encoder_->is_always_on()) {
    this->encoder_->request_keyframe();
  }
  const uint32_t age_ms = micros_to_millis(micros() - frame.capture_us);
  this->sent_frames_++;
  if (!sent_to_all_clients || frame.has_idr || age_ms > K_STALE_FRAME_LOG_THRESHOLD_MS ||
      send_ms > K_SLOW_SEND_LOG_THRESHOLD_MS || (this->sent_frames_ % K_PERIODIC_FRAME_LOG_INTERVAL) == 0) {
    ESP_LOGI(TAG, "rtsp frame: n=%" PRIu32 " key=%d len=%zu age=%" PRIu32
                  "ms send=%" PRIu32 "ms ok=%d udp_bw=%" PRIu32 "kbit/s",
             this->sent_frames_, static_cast<int>(frame.has_idr), frame.len, age_ms, send_ms,
             static_cast<int>(sent_to_all_clients), this->udp_bandwidth_kbps_);
  }

  xQueueSendToBack(this->slot_free_, &frame.data, 0);
  return true;
}

bool H264RtspServer::send_rtcp_sr_(RtspClient &c, uint32_t rtp_timestamp) {
  if (c.transport != RtspTransport::UDP || c.udp_rtcp_fd < 0) {
    return true;
  }

  uint8_t pkt[K_RTCP_SENDER_REPORT_PACKET_BYTES];
  uint32_t ntp_msw = 0;
  uint32_t ntp_lsw = 0;
  H264RtspServer::ntp_from_us(esp_timer_get_time(), &ntp_msw, &ntp_lsw);

  pkt[0] = K_RTP_VERSION_2;
  pkt[1] = K_RTCP_SENDER_REPORT_PACKET_TYPE;
  pkt[2] = 0;
  pkt[3] = K_RTCP_SENDER_REPORT_LENGTH_WORDS_MINUS_ONE;
  write_big_endian(&pkt[4], this->ssrc_);
  write_big_endian(&pkt[8], ntp_msw);
  write_big_endian(&pkt[12], ntp_lsw);
  write_big_endian(&pkt[16], rtp_timestamp);
  write_big_endian(&pkt[20], c.rtp_packet_count);
  write_big_endian(&pkt[24], c.rtp_octet_count);

  const ssize_t sent = send(c.udp_rtcp_fd, pkt, sizeof(pkt), 0);
  if (sent != static_cast<ssize_t>(sizeof(pkt))) {
    ESP_LOGD(TAG, "RTCP SR send failed: errno=%d", errno);
    return false;
  }
  c.last_rtcp_sr_us = micros();
  return true;
}

bool H264RtspServer::send_frame_rtp_(RtspClient &c, const uint8_t *au, size_t len,
                                      bool is_key, uint32_t rtp_timestamp) {
  if (c.needs_keyframe && !is_key) {
    return true;
  }

  const uint32_t now_us = micros();
  if (c.last_rtcp_sr_us == 0 || (now_us - c.last_rtcp_sr_us) >= K_RTCP_SENDER_REPORT_INTERVAL_US) {
    this->send_rtcp_sr_(c, rtp_timestamp);
  }

  // For decoder startup/recovery, send parameter sets immediately before the
  // first IDR access unit. SDP sprop-parameter-sets should be enough for RTSP,
  // but Frigate/FFmpeg recovery is more robust if SPS/PPS are also present in
  // the RTP stream at the keyframe boundary. These are tiny stored NALs; the
  // encoded AU payload itself remains zero-copy via writev below.
  if (is_key && c.needs_keyframe && this->sps_len_ != 0 && this->pps_len_ != 0) {
    if (!this->send_rtp_packet_(c, this->sps_, this->sps_len_, nullptr, 0, rtp_timestamp, false) ||
        !this->send_rtp_packet_(c, this->pps_, this->pps_len_, nullptr, 0, rtp_timestamp, false)) {
      c.needs_keyframe = c.transport == RtspTransport::UDP;
      return false;
    }
  }

  const uint8_t *cursor = au;
  const uint8_t *end = au + len;
  H264AnnexBNal nal;
  if (!next_annexb_nal(&cursor, end, &nal)) {
    return c.needs_keyframe;
  }

  while (!nal.empty()) {
    const bool last_nal = (cursor == end);

    if (nal.len <= this->rtp_payload_size_) {
      if (!this->send_rtp_packet_(c, nal.data, nal.len, nullptr, 0, rtp_timestamp, last_nal)) {
        c.needs_keyframe = c.transport == RtspTransport::UDP;
        return false;
      }
    } else {
      const uint8_t fu_indicator = (nal.data[0] & 0xE0) | K_H264_FU_A_NAL_TYPE;
      const uint8_t nal_type = nal.type();
      const uint8_t *fragment_payload = nal.data + K_H264_NAL_HEADER_BYTES;
      size_t remaining_fragment_bytes = nal.len - K_H264_NAL_HEADER_BYTES;
      bool first_fragment = true;
      while (remaining_fragment_bytes > 0) {
        const size_t max_fragment_payload_len = this->rtp_payload_size_ - K_H264_FU_A_HEADER_BYTES;
        const size_t fragment_payload_len = clamp_at_most(remaining_fragment_bytes, max_fragment_payload_len);
        const bool final_fragment = (fragment_payload_len == remaining_fragment_bytes);
        uint8_t fu_header[K_H264_FU_A_HEADER_BYTES] = {
            fu_indicator,
            static_cast<uint8_t>(nal_type | (first_fragment ? K_H264_FU_START_BIT : 0) |
                                 (final_fragment ? K_H264_FU_END_BIT : 0))};
        if (!this->send_rtp_packet_(c, fu_header, sizeof(fu_header), fragment_payload, fragment_payload_len,
                                    rtp_timestamp, last_nal && final_fragment)) {
          c.needs_keyframe = c.transport == RtspTransport::UDP;
          return false;
        }
        fragment_payload += fragment_payload_len;
        remaining_fragment_bytes -= fragment_payload_len;
        first_fragment = false;
      }
    }

    if (!next_annexb_nal(&cursor, end, &nal)) {
      break;
    }
  }

  if (is_key) {
    c.needs_keyframe = false;
  }
  return true;
}

bool H264RtspServer::send_rtp_packet_(RtspClient &c,
                                       const uint8_t *primary_payload, size_t primary_payload_len,
                                       const uint8_t *secondary_payload, size_t secondary_payload_len,
                                       uint32_t rtp_timestamp, bool marker) {
  const size_t rtp_payload_len = primary_payload_len + secondary_payload_len;
  const size_t rtp_len = K_RTP_HEADER_BYTES + rtp_payload_len;
  uint8_t hdr[K_RTP_TCP_PACKET_HEADER_BYTES];
  hdr[0] = '$';
  hdr[1] = K_RTP_TCP_CHANNEL;
  write_big_endian(&hdr[2], static_cast<uint16_t>(rtp_len));
  hdr[4] = K_RTP_VERSION_2;
  hdr[5] = K_RTP_PAYLOAD_TYPE_H264 | (marker ? K_RTP_MARKER_BIT : 0);
  const uint16_t seq = c.rtp_seq;
  write_big_endian(&hdr[6], seq);
  write_big_endian(&hdr[8], rtp_timestamp);
  write_big_endian(&hdr[12], this->ssrc_);

  if (c.transport == RtspTransport::UDP) {
    if (c.udp_rtp_fd < 0 || rtp_payload_len > this->rtp_payload_size_) {
      this->close_client_(c);
      return false;
    }

    struct iovec packet_segments[3];
    int packet_segment_count = 0;
    packet_segments[packet_segment_count].iov_base = hdr + K_RTP_TCP_INTERLEAVED_HEADER_BYTES;
    packet_segments[packet_segment_count++].iov_len = K_RTP_HEADER_BYTES;
    if (primary_payload_len != 0) {
      packet_segments[packet_segment_count].iov_base = const_cast<uint8_t *>(primary_payload);
      packet_segments[packet_segment_count++].iov_len = primary_payload_len;
    }
    if (secondary_payload_len != 0) {
      packet_segments[packet_segment_count].iov_base = const_cast<uint8_t *>(secondary_payload);
      packet_segments[packet_segment_count++].iov_len = secondary_payload_len;
    }

    if (!this->udp_wait_for_packet_budget_(c, rtp_len)) {
      return false;
    }

    for (uint8_t attempt = 0; attempt < K_UDP_SEND_RETRY_COUNT; attempt++) {
      const ssize_t sent = lwip_writev(c.udp_rtp_fd, packet_segments, packet_segment_count);
      if (sent == static_cast<ssize_t>(rtp_len)) {
        c.rtp_seq++;
        c.rtp_packet_count++;
        c.rtp_octet_count += static_cast<uint32_t>(rtp_payload_len);
        return true;
      }

      const int err = errno;
      if (err != ENOMEM && err != ENOBUFS && err != EAGAIN && err != EWOULDBLOCK) {
        ESP_LOGW(TAG, "UDP RTP send failed; dropping current frame: errno=%d", err);
        return false;
      }

      c.udp_bucket_bytes = 0;
      c.udp_bucket_last_us = micros();
      vTaskDelay(pdMS_TO_TICKS(attempt < K_UDP_FAST_RETRY_COUNT ? K_UDP_FAST_RETRY_DELAY_MS : K_UDP_SLOW_RETRY_DELAY_MS));
    }

    ESP_LOGW(TAG, "UDP RTP send failed after retries; dropping current frame: errno=%d", errno);
    return false;
  }

  struct iovec packet_segments[3];
  int packet_segment_count = 0;
  packet_segments[packet_segment_count].iov_base = hdr;
  packet_segments[packet_segment_count++].iov_len = K_RTP_TCP_PACKET_HEADER_BYTES;
  if (primary_payload_len != 0) {
    packet_segments[packet_segment_count].iov_base = const_cast<uint8_t *>(primary_payload);
    packet_segments[packet_segment_count++].iov_len = primary_payload_len;
  }
  if (secondary_payload_len != 0) {
    packet_segments[packet_segment_count].iov_base = const_cast<uint8_t *>(secondary_payload);
    packet_segments[packet_segment_count++].iov_len = secondary_payload_len;
  }

  if (!send_iov_all(c.fd, packet_segments, packet_segment_count)) {
    this->close_client_(c);
    return false;
  }
  c.rtp_seq++;
  c.rtp_packet_count++;
  c.rtp_octet_count += static_cast<uint32_t>(rtp_payload_len);
  return true;
}

uint32_t H264RtspServer::frame_period_ms_() const {
  const uint8_t fps = this->encoder_ != nullptr ? this->encoder_->get_fps() : K_DEFAULT_ENCODER_FPS;
  if (fps == 0) {
    return 0;
  }
  return (one_second_ms() + static_cast<uint32_t>(fps) - 1UL) / static_cast<uint32_t>(fps);
}

uint32_t H264RtspServer::estimate_udp_send_ms_(size_t len) const {
  if (this->transport_ != RtspTransport::UDP || len == 0 || this->udp_bandwidth_kbps_ == 0) {
    return 0;
  }
  const size_t packets = (len + this->rtp_payload_size_ - 1U) / this->rtp_payload_size_;
  // RTP payload plus conservative RTP/UDP/IP overhead. This is a live send
  // budget, not client timing; frames that cannot fit are dropped instead of
  // queued.
  const uint64_t bytes = static_cast<uint64_t>(len) + static_cast<uint64_t>(packets) * K_ESTIMATED_RTP_UDP_IP_OVERHEAD_BYTES;
  return static_cast<uint32_t>((bytes * K_BITS_PER_BYTE + this->udp_bandwidth_kbps_ - 1ULL) /
                               this->udp_bandwidth_kbps_);
}

uint32_t H264RtspServer::udp_burst_bytes_() const {
  // Match FFmpeg's bitrate/burst model conceptually, but keep the embedded
  // burst cap small. Four RTP packets gives lwIP enough batching to avoid
  // per-packet sleep overhead on keyframes without allowing a whole-frame burst.
  return static_cast<uint32_t>((this->rtp_payload_size_ + K_ESTIMATED_RTP_UDP_IP_OVERHEAD_BYTES) * K_UDP_PACING_BURST_RTP_PACKETS);
}

void H264RtspServer::udp_refill_bucket_(RtspClient &c, uint32_t now_us) {
  const uint32_t burst = this->udp_burst_bytes_();
  if (c.udp_bucket_last_us == 0) {
    c.udp_bucket_last_us = now_us;
    c.udp_bucket_bytes = burst;
    return;
  }

  const uint32_t elapsed_us = now_us - c.udp_bucket_last_us;
  if (elapsed_us == 0) {
    return;
  }
  c.udp_bucket_last_us = now_us;

  const uint64_t add = (static_cast<uint64_t>(elapsed_us) *
                        static_cast<uint64_t>(this->udp_bandwidth_kbps_)) / K_UDP_KBPS_US_PER_BYTE;
  const uint64_t tokens = static_cast<uint64_t>(c.udp_bucket_bytes) + add;
  c.udp_bucket_bytes = static_cast<uint32_t>(clamp_at_most(tokens, burst));
}

bool H264RtspServer::udp_wait_for_packet_budget_(RtspClient &c, size_t packet_len) {
  if (this->transport_ != RtspTransport::UDP || this->udp_bandwidth_kbps_ == 0 || packet_len == 0) {
    return true;
  }

  // Include a conservative IP/UDP/RTP overhead estimate in the token cost.
  const uint32_t cost = static_cast<uint32_t>(packet_len + K_UDP_IP_OVERHEAD_BYTES);
  while (true) {
    const uint32_t now = micros();
    this->udp_refill_bucket_(c, now);
    if (c.udp_bucket_bytes >= cost) {
      c.udp_bucket_bytes -= cost;
      return true;
    }

    const uint32_t need = cost - c.udp_bucket_bytes;
    uint32_t wait_us = static_cast<uint32_t>((static_cast<uint64_t>(need) * K_UDP_KBPS_US_PER_BYTE +
                                              this->udp_bandwidth_kbps_ - 1ULL) /
                                             this->udp_bandwidth_kbps_);
    if (wait_us > K_UDP_PACING_TASK_DELAY_THRESHOLD_US) {
      const uint32_t wait_ms =
          micros_to_millis(static_cast<uint64_t>(wait_us) + one_millisecond_us() - 1U);
      vTaskDelay(pdMS_TO_TICKS(wait_ms));
    } else if (wait_us > 0U) {
      esp_rom_delay_us(wait_us);
    } else {
      taskYIELD();
    }
  }
}

}  // namespace esphome::rtsp_server
#endif
