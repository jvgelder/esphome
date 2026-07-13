import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import camera
from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option, only_on_variant
from esphome.components.esp32.const import VARIANT_ESP32P4
from esphome.const import CONF_ID, CONF_HEIGHT, CONF_WIDTH

DEPENDENCIES = ["esp32", "camera"]
CODEOWNERS = ["@jvgelder"]

camera_h264_ns = cg.esphome_ns.namespace("camera_h264")
CameraH264Encoder = camera_h264_ns.class_(
    "CameraH264Encoder", cg.Component, camera.CameraListener, camera.Encoder
)
CONF_CAMERA_ID   = "camera_id"
CONF_BITRATE     = "bitrate"
CONF_GOP         = "gop"
CONF_QP_MIN      = "qp_min"
CONF_QP_MAX      = "qp_max"
CONF_FPS         = "fps"
CONF_ALWAYS_ON = "always_on"

DEFAULT_FPS = 25
MAX_FPS = 60
DEFAULT_BITRATE = 2_000_000
DEFAULT_GOP = 15
DEFAULT_QP_MIN = 24
DEFAULT_QP_MAX = 51
MIN_QP = 0
MAX_QP = 51
AUTO_SIZE = 0
MAX_WIDTH = 1920
MAX_HEIGHT = 1080
ESP_H264_COMPONENT_VERSION = "1.3.6"
ESP32P4_DEFAULT_CPU_FREQ_MHZ = 360
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
        cv.GenerateID(): cv.declare_id(CameraH264Encoder),
        cv.Required(CONF_CAMERA_ID): cv.use_id(camera.Camera),
        cv.Optional(CONF_FPS, default=DEFAULT_FPS): cv.int_range(1, MAX_FPS),
        cv.Optional(CONF_BITRATE, default=DEFAULT_BITRATE): cv.positive_int,
        cv.Optional(CONF_GOP, default=DEFAULT_GOP): cv.positive_int,
        cv.Optional(CONF_QP_MIN, default=DEFAULT_QP_MIN): cv.int_range(MIN_QP, MAX_QP),
        cv.Optional(CONF_QP_MAX, default=DEFAULT_QP_MAX): cv.int_range(MIN_QP, MAX_QP),
        cv.Optional(CONF_WIDTH, default=AUTO_SIZE): cv.int_range(AUTO_SIZE, MAX_WIDTH),
        cv.Optional(CONF_HEIGHT, default=AUTO_SIZE): cv.int_range(AUTO_SIZE, MAX_HEIGHT),
        cv.Optional(CONF_ALWAYS_ON, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    only_on_variant(supported=[VARIANT_ESP32P4]),
)


async def to_code(config):
    enc = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(enc, config)

    cam = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(enc.add_camera(cam))
    cg.add(enc.set_fps(config[CONF_FPS]))
    cg.add(enc.set_bitrate(config[CONF_BITRATE]))
    cg.add(enc.set_gop(config[CONF_GOP]))
    cg.add(enc.set_qp_min(config[CONF_QP_MIN]))
    cg.add(enc.set_qp_max(config[CONF_QP_MAX]))
    cg.add(enc.set_width(config.get(CONF_WIDTH, AUTO_SIZE)))
    cg.add(enc.set_height(config.get(CONF_HEIGHT, AUTO_SIZE)))
    cg.add(enc.set_always_on(config[CONF_ALWAYS_ON]))

    cg.add_define("USE_CAMERA_H264")
    add_idf_component(name="espressif/esp_h264", ref=ESP_H264_COMPONENT_VERSION)
    add_idf_sdkconfig_option("CONFIG_H264_HW_ENCODER", True)
    add_idf_sdkconfig_option("CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_360", True)
    add_idf_sdkconfig_option("CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ", ESP32P4_DEFAULT_CPU_FREQ_MHZ)


