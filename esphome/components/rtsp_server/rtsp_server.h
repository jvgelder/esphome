#pragma once
#ifdef USE_CAMERA_H264

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/camera_h264/camera_h264.h"
#include "esphome/components/camera_h264/h264_units.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <lwip/sockets.h>
#include <atomic>
#include <string>

namespace esphome::rtsp_server {

static constexpr size_t RTSP_MAX_CLIENTS = 2;
static constexpr size_t RTSP_RX_BUF = camera_h264::kibibytes(1);
static constexpr uint16_t RTSP_DEFAULT_PORT = 554;
static constexpr uint8_t RTSP_MIN_FRAME_SLOTS = 1;
static constexpr uint32_t RTSP_DEFAULT_UDP_BANDWIDTH_KBPS = 50000;
static constexpr uint32_t RTSP_DEFAULT_SSRC = 0x22334455;
static constexpr uint16_t RTP_PAYLOAD_MIN = 256;
static constexpr uint16_t RTP_PAYLOAD_MAX = 1400;  // conservative MTU-safe payload size for RTP/UDP
static constexpr uint16_t RTP_PAYLOAD_DEFAULT = RTP_PAYLOAD_MAX;
static constexpr size_t SPS_PPS_MAX = 128;
static constexpr size_t RTSP_AUTH_CREDENTIAL_MAX = 128;
static constexpr size_t RTSP_MAX_FRAME_SLOTS = 2;  // low-RAM cap: one sending, one queued/free

enum class RtspState : uint8_t { INIT, READY, PLAYING };
enum class RtspTransport : uint8_t { TCP, UDP };

struct RtspClient {
  int        fd{-1};
  RtspState  state{RtspState::INIT};
  uint32_t   session_id{0};
  uint16_t   rtp_seq{0};
  RtspTransport transport{RtspTransport::TCP};
  sockaddr_in udp_rtp_addr{};
  uint16_t   udp_rtp_port{0};
  int        udp_rtp_fd{-1};
  uint16_t   udp_server_port{0};
  int        udp_rtcp_fd{-1};
  uint16_t   udp_rtcp_port{0};
  uint32_t   udp_bucket_last_us{0};
  uint32_t   udp_bucket_bytes{0};
  uint32_t   rtp_packet_count{0};
  uint32_t   rtp_octet_count{0};
  uint32_t   last_rtcp_sr_us{0};
  bool       needs_keyframe{false};
  char       rx[RTSP_RX_BUF];
  size_t     rx_len{0};
};

// A frame occupying one pool slot. data points into the slot (no ownership).
struct RtspFrame {
  uint8_t *data;
  size_t   len;
  bool     is_key;
  bool     has_idr;
  uint32_t rtp_timestamp;
  uint32_t capture_us;
};


class H264RtspServer : public Component {
 public:
  void set_encoder(camera_h264::CameraH264Encoder *enc) { encoder_ = enc; }
  void set_port(uint16_t port)     { port_ = port; }
  void set_username(const std::string &u) { username_ = u; }
  void set_password(const std::string &p) { password_ = p; }
  void set_frame_slots(uint8_t frame_slots) {
    frame_slots_ = clamp_at_most(clamp_at_least(frame_slots, RTSP_MIN_FRAME_SLOTS), RTSP_MAX_FRAME_SLOTS);
  }
  void set_transport(RtspTransport v) { transport_ = v; }
  void set_udp_bandwidth_kbps(uint32_t v) { udp_bandwidth_kbps_ = v; }
  void set_force_keyframe_on_play(bool v) { force_keyframe_on_play_ = v; }
  void set_rtp_payload_size(uint16_t rtp_payload_size) {
    rtp_payload_size_ = clamp_at_most(clamp_at_least(rtp_payload_size, RTP_PAYLOAD_MIN), RTP_PAYLOAD_MAX);
  }

  void setup() override;
  void loop() override {}
  void dump_config() override;
  // Must run after encoder setup (AFTER_WIFI-1) so get_out_size() is valid
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI - 2; }

 protected:
  static void server_task_trampoline(void *arg) {
    static_cast<H264RtspServer *>(arg)->server_task_();
    vTaskDelete(nullptr);
  }
  void server_task_();
  bool send_next_frame_(TickType_t wait_ticks);

  // RTSP protocol
  void handle_request_(RtspClient &c);
  bool check_auth_(const char *req);
  void send_response_(int fd, const char *fmt, ...);
  void handle_options_(RtspClient &c, int cseq);
  void handle_describe_(RtspClient &c, int cseq);
  void handle_setup_(RtspClient &c, int cseq, const char *req);
  void handle_play_(RtspClient &c, int cseq);
  void handle_teardown_(RtspClient &c, int cseq);
  void close_client_(RtspClient &c);
  bool open_client_udp_socket_(RtspClient &c);
  void close_client_udp_socket_(RtspClient &c);
  static bool parse_client_rtp_port(const char *req, uint16_t *port);

  // RTP — writev scatter-gather, no payload copies
  bool send_frame_rtp_(RtspClient &c, const uint8_t *au, size_t len, bool is_key, uint32_t rtp_timestamp);
  bool send_rtp_packet_(RtspClient &c, const uint8_t *primary_payload, size_t primary_payload_len,
                        const uint8_t *secondary_payload, size_t secondary_payload_len, uint32_t rtp_timestamp, bool marker);
  bool send_rtcp_sr_(RtspClient &c, uint32_t rtp_timestamp);
  static void ntp_from_us(uint64_t us, uint32_t *msw, uint32_t *lsw);
  uint32_t frame_period_ms_() const;
  uint32_t estimate_udp_send_ms_(size_t len) const;
  uint32_t udp_burst_bytes_() const;
  void udp_refill_bucket_(RtspClient &c, uint32_t now_us);
  bool udp_wait_for_packet_budget_(RtspClient &c, size_t packet_len);

  // Encoder callback path (copy encoded AU into fixed pool slot)
  void on_encoded_frame_(const uint8_t *data, size_t len, bool is_key);
  void queue_or_replace_frame_(RtspFrame frame);
  void purge_queued_frames_();
  bool any_playing_client_waits_for_keyframe_();
  bool should_drop_outside_live_budget_(const RtspFrame &frame, bool client_needs_keyframe) const;
  void extract_sps_pps_(const uint8_t *au, size_t len);

  camera_h264::CameraH264Encoder *encoder_{nullptr};
  uint16_t    port_{RTSP_DEFAULT_PORT};
  std::string username_;
  std::string password_;
  std::string auth_b64_;

  RtspClient clients_[RTSP_MAX_CLIENTS];  // Owned by the RTSP task; encoder callbacks only use frame queues.

  // Pool: slots allocated once in setup (PSRAM, 64B aligned). No runtime malloc.
  uint8_t      *slots_[RTSP_MAX_FRAME_SLOTS]{};
  uint8_t       frame_slots_{RTSP_MIN_FRAME_SLOTS};
  RtspTransport transport_{RtspTransport::TCP};
  uint32_t      udp_bandwidth_kbps_{RTSP_DEFAULT_UDP_BANDWIDTH_KBPS};
  uint16_t      rtp_payload_size_{RTP_PAYLOAD_DEFAULT};
  bool          force_keyframe_on_play_{true};
  size_t        slot_size_{0};
  QueueHandle_t slot_free_{nullptr};    // uint8_t* — empty slots
  QueueHandle_t frame_queue_{nullptr};  // RtspFrame — filled frames

  uint8_t  sps_[SPS_PPS_MAX]; size_t sps_len_{0};
  uint8_t  pps_[SPS_PPS_MAX]; size_t pps_len_{0};
  std::atomic<bool> have_params_{false};
  std::atomic<bool> streaming_{false};

  uint32_t sent_frames_{0};
  uint32_t ssrc_{RTSP_DEFAULT_SSRC};
};

}  // namespace esphome::rtsp_server
#endif
