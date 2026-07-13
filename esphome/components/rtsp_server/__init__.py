import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.camera_h264 import CameraH264Encoder
from esphome.const import CONF_ID, CONF_PORT, CONF_USERNAME, CONF_PASSWORD
from esphome.types import ConfigType

DEPENDENCIES = ["camera_h264", "network"]
CODEOWNERS = ["@jvgelder"]

rtsp_server_ns = cg.esphome_ns.namespace("rtsp_server")
H264RtspServer = rtsp_server_ns.class_("H264RtspServer", cg.Component)
RtspTransport = rtsp_server_ns.enum("RtspTransport", is_class=True)

CONF_ENCODER_ID = "encoder_id"
CONF_FRAME_SLOTS = "frame_slots"
CONF_TRANSPORT = "transport"
CONF_UDP_BANDWIDTH_KBPS = "udp_bandwidth_kbps"
CONF_RTP_PAYLOAD_SIZE = "rtp_payload_size"
CONF_FORCE_KEYFRAME_ON_PLAY = "force_keyframe_on_play"

DEFAULT_RTSP_PORT = 554
DEFAULT_FRAME_SLOTS = 1
MIN_FRAME_SLOTS = 1
MAX_FRAME_SLOTS = 2
DEFAULT_UDP_BANDWIDTH_KBPS = 50_000
MIN_UDP_BANDWIDTH_KBPS = 1_000
MAX_UDP_BANDWIDTH_KBPS = 1_000_000
DEFAULT_RTP_PAYLOAD_SIZE = 1400
MIN_RTP_PAYLOAD_SIZE = 256
MAX_RTP_PAYLOAD_SIZE = 1400
RTSP_MAX_CLIENTS = 2
RTSP_CONTROL_SOCKET_COUNT = RTSP_MAX_CLIENTS
RTSP_LISTEN_SOCKET_COUNT = 1
RTSP_UDP_SOCKETS_PER_CLIENT = 2

TRANSPORTS = {
    "tcp": RtspTransport.TCP,
    "udp": RtspTransport.UDP,
}


def _consume_rtsp_server_sockets(config: ConfigType) -> ConfigType:
    """Register the sockets this RTSP server can hold at runtime."""
    from esphome.components import socket

    # One listening TCP socket plus one RTSP control TCP socket per client.
    socket.consume_sockets(
        RTSP_LISTEN_SOCKET_COUNT, "rtsp_server", socket.SocketType.TCP_LISTEN
    )(config)
    socket.consume_sockets(RTSP_CONTROL_SOCKET_COUNT, "rtsp_server")(config)

    # UDP transport opens one RTP and one RTCP socket per connected client. The
    # socket component accounts descriptor pressure; generic socket accounting is
    # enough here because these are not listening sockets.
    if config[CONF_TRANSPORT] == "udp":
        socket.consume_sockets(
            RTSP_MAX_CLIENTS * RTSP_UDP_SOCKETS_PER_CLIENT, "rtsp_server"
        )(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
        cv.GenerateID(): cv.declare_id(H264RtspServer),
        cv.Required(CONF_ENCODER_ID): cv.use_id(CameraH264Encoder),
        cv.Optional(CONF_PORT, default=DEFAULT_RTSP_PORT): cv.port,
        cv.Optional(CONF_USERNAME, default=""): cv.string,
        cv.Optional(CONF_PASSWORD, default=""): cv.string,
        cv.Optional(CONF_FRAME_SLOTS, default=DEFAULT_FRAME_SLOTS): cv.int_range(MIN_FRAME_SLOTS, MAX_FRAME_SLOTS),
        cv.Optional(CONF_TRANSPORT, default="tcp"): cv.enum(TRANSPORTS, lower=True),
        cv.Optional(CONF_UDP_BANDWIDTH_KBPS, default=DEFAULT_UDP_BANDWIDTH_KBPS): cv.int_range(MIN_UDP_BANDWIDTH_KBPS, MAX_UDP_BANDWIDTH_KBPS),
        cv.Optional(CONF_RTP_PAYLOAD_SIZE, default=DEFAULT_RTP_PAYLOAD_SIZE): cv.int_range(MIN_RTP_PAYLOAD_SIZE, MAX_RTP_PAYLOAD_SIZE),
        cv.Optional(CONF_FORCE_KEYFRAME_ON_PLAY, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _consume_rtsp_server_sockets,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    enc = await cg.get_variable(config[CONF_ENCODER_ID])
    cg.add(var.set_encoder(enc))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_username(config[CONF_USERNAME]))
    cg.add(var.set_password(config[CONF_PASSWORD]))
    cg.add(var.set_frame_slots(config[CONF_FRAME_SLOTS]))
    cg.add(var.set_transport(config[CONF_TRANSPORT]))
    cg.add(var.set_udp_bandwidth_kbps(config[CONF_UDP_BANDWIDTH_KBPS]))
    cg.add(var.set_rtp_payload_size(config[CONF_RTP_PAYLOAD_SIZE]))
    cg.add(var.set_force_keyframe_on_play(config[CONF_FORCE_KEYFRAME_ON_PLAY]))
