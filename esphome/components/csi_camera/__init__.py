from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import camera, i2c
from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option, only_on_variant
from esphome.components.esp32.const import VARIANT_ESP32P4
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_I2C_ID, CONF_ID, CONF_MODE, CONF_SENSOR
from esphome.core.entity_helpers import setup_entity

AUTO_LOAD = ["camera"]
DEPENDENCIES = ["esp32", "i2c"]
CODEOWNERS = ["@jvgelder"]

csi_camera_ns = cg.esphome_ns.namespace("csi_camera")
CsiCamera = csi_camera_ns.class_("CsiCamera", camera.Camera, i2c.I2CDevice)
CsiCameraSetNightModeAction = csi_camera_ns.class_(
    "CsiCameraSetNightModeAction", automation.Action
)

CONF_POWER_DOWN_PIN = "power_down_pin"
CONF_LDO_CHAN = "ldo_chan"
CONF_LDO_MV = "ldo_mv"
CONF_FRAME_BUFFER_COUNT = "frame_buffer_count"
CONF_SENSOR_FORMAT = "sensor_format"
CONF_ENABLE_CCM    = "enable_ccm"
CONF_CCM           = "ccm"
CONF_HUE           = "hue"
CONF_BRIGHTNESS    = "brightness"
CONF_BAYER_ORDER   = "bayer_order"
BAYER_ORDER_AUTO   = "auto"
CONF_SATURATION    = "saturation"
CONF_CONTRAST      = "contrast"
CONF_VERTICAL_FLIP = "vertical_flip"
CONF_HORIZONTAL_MIRROR = "horizontal_mirror"
CONF_TEST_PATTERN  = "test_pattern"
CONF_FORCE_OFFICIAL_SENSOR_DRIVER = "force_official_sensor_driver"
CONF_LANE_BIT_RATE_MBPS = "lane_bit_rate_mbps"
CONF_RAW_FORMAT = "raw_format"
CONF_WB_MODE = "wb_mode"
CONF_AEC_MODE = "aec_mode"
CONF_AE_LEVEL = "ae_level"
CONF_AGC_MODE = "agc_mode"
CONF_SHARPNESS = "sharpness"
CONF_DENOISE = "denoise"
CONF_DEAD_PIXEL_CORRECTION = "dead_pixel_correction"
CONF_BLACK_LEVEL_CORRECTION = "black_level_correction"
CONF_LENS_SHADING_CORRECTION = "lens_shading_correction"
CONF_NIGHT_MODE = "night_mode"

DEFAULT_SENSOR = "ov5647"
SENSOR_OV5647 = "ov5647"
SENSOR_IMX219 = "imx219"
SENSOR_CUSTOM = "custom"
OV5647_DEFAULT_I2C_ADDRESS = 0x36
IMX219_DEFAULT_I2C_ADDRESS = 0x10
DEFAULT_LANE_BIT_RATE_MBPS = 912
DEFAULT_SATURATION = 1.5
CUSTOM_DEFAULT_SATURATION = 1.0
DEFAULT_HUE = 0
DEFAULT_BRIGHTNESS = 0
DEFAULT_CONTRAST = 1.0
DEFAULT_TEST_PATTERN = False
DEFAULT_BAYER_ORDER = "gbrg"
DEFAULT_LDO_CHANNEL = 3
MIN_LDO_CHANNEL = 1
MAX_LDO_CHANNEL = 4
DEFAULT_LDO_MV = 2500
MIN_LDO_MV = 1800
MAX_LDO_MV = 3300
DEFAULT_FRAME_BUFFERS = 3
MIN_FRAME_BUFFERS = 2
MAX_FRAME_BUFFERS = 4
CCM_MATRIX_ELEMENT_COUNT = 9
MIN_HUE = 0
MAX_HUE = 359
MIN_BRIGHTNESS = -128
MAX_BRIGHTNESS = 127
MIN_SATURATION = 0.0
MAX_SATURATION = 1.99
MIN_CONTRAST = 0.0
MAX_CONTRAST = 1.99
MIN_LANE_BIT_RATE_MBPS = 1
MAX_LANE_BIT_RATE_MBPS = 2500
RAW_FORMAT_RAW8 = "raw8"
RAW_FORMAT_RAW10 = "raw10"
MIN_CAMERA_TUNING_LEVEL = -2
MAX_CAMERA_TUNING_LEVEL = 2
WB_MODE_AUTO = "auto"
WB_MODE_SUNNY = "sunny"
WB_MODE_CLOUDY = "cloudy"
WB_MODE_OFFICE = "office"
WB_MODE_HOME = "home"
WB_MODES = {
    WB_MODE_AUTO: 0,
    WB_MODE_SUNNY: 1,
    WB_MODE_CLOUDY: 2,
    WB_MODE_OFFICE: 3,
    WB_MODE_HOME: 4,
}
GAIN_CONTROL_MODES = {
    "AUTO": True,
    "MANUAL": False,
}

MODE_1080P30 = "1080p30"
MODE_720P60 = "720p60"
MODE_VGA60 = "640x480p60"
MODE_VGA90 = "640x480p90"
MODE_800X800P50 = "800x800p50"

MODE_LABELS = {
    MODE_1080P30: "1080P@30",
    MODE_720P60: "720P@60",
    MODE_VGA60: "640x480P@60",
    MODE_VGA90: "640x480P@90",
    MODE_800X800P50: "800x800P@50",
}

FORMAT_RAW10_1080P30 = "MIPI_2lane_24Minput_RAW10_1920x1080_30fps"
FORMAT_RAW10_720P60 = "MIPI_2lane_24Minput_RAW10_1280x720_60fps"
FORMAT_RAW10_VGA60 = "MIPI_2lane_24Minput_RAW10_640x480_60fps"
FORMAT_RAW10_VGA90 = "MIPI_2lane_24Minput_RAW10_640x480_90fps"
FORMAT_RAW8_800X800P50 = "MIPI_2lane_24Minput_RAW8_800x800_50fps"

OV5647_KCONFIG_SENSOR = [
    "CONFIG_CAMERA_OV5647",
    "CONFIG_CAMERA_OV5647_AUTO_DETECT_MIPI_INTERFACE_SENSOR",
]
OV5647_KCONFIG_1080P30 = "CONFIG_CAMERA_OV5647_MIPI_RAW10_1920X1080_30FPS"
OV5647_KCONFIG_720P60 = "CONFIG_CAMERA_OV5647_MIPI_RAW10_1280X720_60FPS"
OV5647_KCONFIG_VGA60 = "CONFIG_CAMERA_OV5647_MIPI_RAW10_640X480_60FPS"
OV5647_KCONFIG_VGA90 = "CONFIG_CAMERA_OV5647_MIPI_RAW10_640X480_90FPS"
OV5647_KCONFIG_800X800P50 = "CONFIG_CAMERA_OV5647_MIPI_RAW8_800X800_50FPS"

IDENTITY_CCM = [
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
]

SENSOR_PROFILES = {
    SENSOR_OV5647: {
        "default_mode": MODE_1080P30,
        "default_address": OV5647_DEFAULT_I2C_ADDRESS,
        CONF_ENABLE_CCM: False,
        CONF_CCM: IDENTITY_CCM,
        CONF_SATURATION: DEFAULT_SATURATION,
        CONF_HUE: DEFAULT_HUE,
        CONF_BRIGHTNESS: DEFAULT_BRIGHTNESS,
        CONF_CONTRAST: DEFAULT_CONTRAST,
        CONF_TEST_PATTERN: DEFAULT_TEST_PATTERN,
        "detect_symbol": "ov5647_detect",
        "sdkconfig": OV5647_KCONFIG_SENSOR,
        "modes": {
            MODE_1080P30: {
                CONF_SENSOR_FORMAT: FORMAT_RAW10_1080P30,
                CONF_RAW_FORMAT: RAW_FORMAT_RAW10,
                CONF_LANE_BIT_RATE_MBPS: DEFAULT_LANE_BIT_RATE_MBPS,
                "sdkconfig": [OV5647_KCONFIG_1080P30],
            },
            MODE_720P60: {
                CONF_SENSOR_FORMAT: FORMAT_RAW10_720P60,
                CONF_RAW_FORMAT: RAW_FORMAT_RAW10,
                CONF_LANE_BIT_RATE_MBPS: DEFAULT_LANE_BIT_RATE_MBPS,
                "sdkconfig": [OV5647_KCONFIG_720P60],
            },
            MODE_VGA60: {
                CONF_SENSOR_FORMAT: FORMAT_RAW10_VGA60,
                CONF_RAW_FORMAT: RAW_FORMAT_RAW10,
                CONF_LANE_BIT_RATE_MBPS: DEFAULT_LANE_BIT_RATE_MBPS,
                "sdkconfig": [OV5647_KCONFIG_VGA60],
            },
            MODE_VGA90: {
                CONF_SENSOR_FORMAT: FORMAT_RAW10_VGA90,
                CONF_RAW_FORMAT: RAW_FORMAT_RAW10,
                CONF_LANE_BIT_RATE_MBPS: DEFAULT_LANE_BIT_RATE_MBPS,
                "sdkconfig": [OV5647_KCONFIG_VGA90],
            },
            MODE_800X800P50: {
                CONF_SENSOR_FORMAT: FORMAT_RAW8_800X800P50,
                CONF_RAW_FORMAT: RAW_FORMAT_RAW8,
                CONF_LANE_BIT_RATE_MBPS: DEFAULT_LANE_BIT_RATE_MBPS,
                "sdkconfig": [OV5647_KCONFIG_800X800P50],
            },
        },
    },
    SENSOR_IMX219: {
        "default_mode": MODE_1080P30,
        "default_address": IMX219_DEFAULT_I2C_ADDRESS,
        CONF_ENABLE_CCM: False,
        CONF_CCM: IDENTITY_CCM,
        CONF_SATURATION: DEFAULT_SATURATION,
        CONF_HUE: DEFAULT_HUE,
        CONF_BRIGHTNESS: DEFAULT_BRIGHTNESS,
        CONF_CONTRAST: DEFAULT_CONTRAST,
        CONF_TEST_PATTERN: DEFAULT_TEST_PATTERN,
        # IMX219 uses the local register-table driver unless the project adds
        # an official esp_cam_sensor detector. The local tables currently drive
        # RAW10 modes only; IMX219 sensors can also expose RAW8 in other drivers.
        "modes": {
            MODE_1080P30: {
                CONF_SENSOR_FORMAT: FORMAT_RAW10_1080P30,
                CONF_RAW_FORMAT: RAW_FORMAT_RAW10,
                CONF_LANE_BIT_RATE_MBPS: DEFAULT_LANE_BIT_RATE_MBPS,
            },
            MODE_720P60: {
                CONF_SENSOR_FORMAT: FORMAT_RAW10_720P60,
                CONF_RAW_FORMAT: RAW_FORMAT_RAW10,
                CONF_LANE_BIT_RATE_MBPS: DEFAULT_LANE_BIT_RATE_MBPS,
            },
        },
    },
    SENSOR_CUSTOM: {
        "default_mode": None,
        "default_address": OV5647_DEFAULT_I2C_ADDRESS,
        CONF_ENABLE_CCM: False,
        CONF_CCM: IDENTITY_CCM,
        CONF_SATURATION: CUSTOM_DEFAULT_SATURATION,
        CONF_HUE: DEFAULT_HUE,
        CONF_BRIGHTNESS: DEFAULT_BRIGHTNESS,
        CONF_CONTRAST: DEFAULT_CONTRAST,
        CONF_TEST_PATTERN: DEFAULT_TEST_PATTERN,
        CONF_LANE_BIT_RATE_MBPS: DEFAULT_LANE_BIT_RATE_MBPS,
    },
}


def _mode_for_format(profile, sensor_format):
    for mode, values in profile.get("modes", {}).items():
        if values.get(CONF_SENSOR_FORMAT) == sensor_format:
            return mode
    return None


def _supported_modes(profile):
    return ", ".join(
        f"{mode} ({MODE_LABELS.get(mode, mode)})" for mode in profile.get("modes", {})
    )


def _supported_formats(profile):
    formats = {values[CONF_SENSOR_FORMAT] for values in profile.get("modes", {}).values()}
    return ", ".join(sorted(formats))


def _supported_raw_formats(profile):
    formats = {values.get(CONF_RAW_FORMAT) for values in profile.get("modes", {}).values()}
    return ", ".join(sorted(format_value for format_value in formats if format_value is not None))


def _mode_for_raw_format(profile, raw_format):
    for mode, values in profile.get("modes", {}).items():
        if values.get(CONF_RAW_FORMAT) == raw_format:
            return mode
    return None


def _supported_modes_for_raw_format(profile, raw_format):
    modes = [
        mode
        for mode, values in profile.get("modes", {}).items()
        if values.get(CONF_RAW_FORMAT) == raw_format
    ]
    return ", ".join(f"{mode} ({MODE_LABELS.get(mode, mode)})" for mode in modes)


def _raw_formats_for_mode(profile, mode):
    values = profile.get("modes", {}).get(mode, {})
    raw_format = values.get(CONF_RAW_FORMAT)
    return raw_format if raw_format is not None else "none"


def _apply_sensor_profile(config):
    config = dict(config)
    config.setdefault(CONF_FRAME_BUFFER_COUNT, DEFAULT_FRAME_BUFFERS)
    sensor = config[CONF_SENSOR]
    profile = SENSOR_PROFILES[sensor]
    config.setdefault(CONF_ADDRESS, profile["default_address"])

    if sensor == SENSOR_CUSTOM:
        if CONF_MODE in config:
            raise cv.Invalid("mode is only supported for built-in CSI camera sensors")
        for key in (CONF_SENSOR_FORMAT, CONF_BAYER_ORDER):
            if key not in config:
                raise cv.Invalid(f"{key} is required when sensor: custom")
        if config[CONF_BAYER_ORDER] == BAYER_ORDER_AUTO:
            raise cv.Invalid("bayer_order: auto is only supported for built-in CSI camera sensors")
    else:
        config.setdefault(CONF_BAYER_ORDER, DEFAULT_BAYER_ORDER)
        modes = profile["modes"]
        mode = config.get(CONF_MODE)
        explicit_format = config.get(CONF_SENSOR_FORMAT)
        raw_format = config.get(CONF_RAW_FORMAT)

        if mode is None and explicit_format is not None:
            mode = _mode_for_format(profile, explicit_format)
            if mode is None:
                raise cv.Invalid(
                    f"Unsupported sensor_format '{explicit_format}' for sensor '{sensor}'. "
                    f"Use one of: {_supported_formats(profile)}"
                )
        elif mode is None and raw_format is not None:
            mode = _mode_for_raw_format(profile, raw_format)
            if mode is None:
                raise cv.Invalid(
                    f"Unsupported raw_format '{raw_format}' for sensor '{sensor}'. "
                    f"Supported raw formats for sensor '{sensor}': {_supported_raw_formats(profile)}"
                )
        if mode is None:
            mode = profile["default_mode"]

        if mode not in modes:
            raise cv.Invalid(
                f"Unsupported mode/profile '{config.get(CONF_MODE)}' for sensor '{sensor}'. "
                f"Supported modes for sensor '{sensor}': {_supported_modes(profile)}"
            )

        mode_values = modes[mode]
        if explicit_format is not None and explicit_format != mode_values[CONF_SENSOR_FORMAT]:
            raise cv.Invalid(
                f"mode '{config.get(CONF_MODE)}' uses sensor_format '{mode_values[CONF_SENSOR_FORMAT]}', "
                f"but sensor_format was set to '{explicit_format}'. Remove one of these options."
            )
        if raw_format is not None and raw_format != mode_values.get(CONF_RAW_FORMAT):
            requested_mode = config.get(CONF_MODE, mode)
            raise cv.Invalid(
                f"mode '{requested_mode}' only supports raw_format "
                f"'{_raw_formats_for_mode(profile, mode)}', but raw_format was set to "
                f"'{raw_format}'. Modes for sensor '{sensor}' with raw_format "
                f"'{raw_format}': {_supported_modes_for_raw_format(profile, raw_format) or 'none'}"
            )

        config[CONF_MODE] = mode
        for key, value in mode_values.items():
            if key != "sdkconfig":
                config.setdefault(key, value)

    for key, value in profile.items():
        if key in ("default_mode", "default_address", "detect_symbol", "sdkconfig", "modes"):
            continue
        config.setdefault(key, value)

    return config

CONFIG_SCHEMA = cv.All(
    cv.ENTITY_BASE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(CsiCamera),
            cv.Optional(CONF_SENSOR, default=DEFAULT_SENSOR): cv.one_of(
                SENSOR_OV5647, SENSOR_IMX219, SENSOR_CUSTOM, lower=True
            ),
            cv.Optional(CONF_MODE): cv.string_strict,
            cv.Optional(CONF_POWER_DOWN_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_LDO_CHAN, default=DEFAULT_LDO_CHANNEL): cv.int_range(MIN_LDO_CHANNEL, MAX_LDO_CHANNEL),
            cv.Optional(CONF_LDO_MV, default=DEFAULT_LDO_MV): cv.int_range(MIN_LDO_MV, MAX_LDO_MV),
            cv.Optional(CONF_FRAME_BUFFER_COUNT): cv.int_range(MIN_FRAME_BUFFERS, MAX_FRAME_BUFFERS),
            cv.Optional(CONF_SENSOR_FORMAT): cv.string_strict,
            cv.Optional(CONF_RAW_FORMAT): cv.one_of(RAW_FORMAT_RAW8, RAW_FORMAT_RAW10, lower=True),
            cv.Optional(CONF_ENABLE_CCM): cv.boolean,
            cv.Optional(CONF_CCM): cv.All([cv.float_], cv.Length(min=CCM_MATRIX_ELEMENT_COUNT, max=CCM_MATRIX_ELEMENT_COUNT)),
            cv.Optional(CONF_HUE): cv.int_range(MIN_HUE, MAX_HUE),
            cv.Optional(CONF_BRIGHTNESS): cv.int_range(MIN_BRIGHTNESS, MAX_BRIGHTNESS),
            cv.Optional(CONF_CONTRAST): cv.float_range(min=MIN_CONTRAST, max=MAX_CONTRAST),
            cv.Optional(CONF_VERTICAL_FLIP): cv.boolean,
            cv.Optional(CONF_HORIZONTAL_MIRROR): cv.boolean,
            cv.Optional(CONF_TEST_PATTERN): cv.boolean,
            cv.Optional(CONF_WB_MODE): cv.enum(WB_MODES, lower=True),
            cv.Optional(CONF_AEC_MODE): cv.enum(GAIN_CONTROL_MODES, upper=True),
            cv.Optional(CONF_AE_LEVEL): cv.int_range(MIN_CAMERA_TUNING_LEVEL, MAX_CAMERA_TUNING_LEVEL),
            cv.Optional(CONF_AGC_MODE): cv.enum(GAIN_CONTROL_MODES, upper=True),
            cv.Optional(CONF_SHARPNESS): cv.int_range(MIN_CAMERA_TUNING_LEVEL, MAX_CAMERA_TUNING_LEVEL),
            cv.Optional(CONF_DENOISE): cv.int_range(MIN_CAMERA_TUNING_LEVEL, MAX_CAMERA_TUNING_LEVEL),
            cv.Optional(CONF_DEAD_PIXEL_CORRECTION): cv.boolean,
            cv.Optional(CONF_BLACK_LEVEL_CORRECTION): cv.boolean,
            cv.Optional(CONF_LENS_SHADING_CORRECTION): cv.boolean,
            cv.Optional(CONF_NIGHT_MODE): cv.boolean,
            cv.Optional(CONF_BAYER_ORDER): cv.one_of(
                BAYER_ORDER_AUTO, "rggb", "grbg", "gbrg", "bggr", lower=True
            ),
            cv.Optional(CONF_SATURATION): cv.float_range(min=MIN_SATURATION, max=MAX_SATURATION),
            cv.Optional(CONF_LANE_BIT_RATE_MBPS): cv.int_range(MIN_LANE_BIT_RATE_MBPS, MAX_LANE_BIT_RATE_MBPS),
            cv.Optional(CONF_FORCE_OFFICIAL_SENSOR_DRIVER, default=True): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(
        cv.Schema(
            {
                cv.GenerateID(CONF_I2C_ID): cv.use_id(i2c.I2CBus),
                cv.Optional(CONF_ADDRESS): cv.i2c_address,
            }
        )
    ),
    only_on_variant(supported=[VARIANT_ESP32P4]),
    _apply_sensor_profile,
)


async def to_code(config):
    cg.add_define("USE_CAMERA")
    cg.add_define("USE_CSI_CAMERA")
    add_idf_component(name="espressif/esp_cam_sensor", ref="*")
    add_idf_component(name="espressif/esp_sccb_intf", ref="*")

    sensor = config[CONF_SENSOR]
    profile = SENSOR_PROFILES[sensor]
    mode = config.get(CONF_MODE)

    if config[CONF_FORCE_OFFICIAL_SENSOR_DRIVER] and sensor != SENSOR_CUSTOM:
        for opt in profile.get("sdkconfig", []):
            add_idf_sdkconfig_option(opt, True)
        if mode is not None:
            for opt in profile.get("modes", {}).get(mode, {}).get("sdkconfig", []):
                add_idf_sdkconfig_option(opt, True)
        if detect_symbol := profile.get("detect_symbol"):
            # Keep esp_cam_sensor's detector from being garbage-collected by the linker.
            cg.add_build_flag(f"-Wl,-u,{detect_symbol}")

    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)

    await i2c.register_i2c_device(var, config)
    cg.add(var.set_sensor_profile(sensor))
    if (power_down_pin := config.get(CONF_POWER_DOWN_PIN)) is not None:
        cg.add(var.set_power_down_pin(power_down_pin))
    cg.add(var.set_ldo_chan(config[CONF_LDO_CHAN]))
    cg.add(var.set_ldo_mv(config[CONF_LDO_MV]))
    cg.add(var.set_frame_buffer_count(config[CONF_FRAME_BUFFER_COUNT]))
    cg.add(var.set_sensor_format(config[CONF_SENSOR_FORMAT]))
    cg.add(var.set_lane_bit_rate_mbps(config[CONF_LANE_BIT_RATE_MBPS]))
    cg.add(var.set_enable_ccm(config[CONF_ENABLE_CCM]))
    ccm = config[CONF_CCM]
    cg.add(var.set_ccm(*ccm))
    cg.add(var.set_hue(config[CONF_HUE]))
    cg.add(var.set_brightness(config[CONF_BRIGHTNESS]))
    cg.add(var.set_contrast(config[CONF_CONTRAST]))
    if CONF_VERTICAL_FLIP in config:
        cg.add(var.set_vertical_flip(config[CONF_VERTICAL_FLIP]))
    if CONF_HORIZONTAL_MIRROR in config:
        cg.add(var.set_horizontal_mirror(config[CONF_HORIZONTAL_MIRROR]))
    cg.add(var.set_test_pattern(config[CONF_TEST_PATTERN]))
    if CONF_WB_MODE in config:
        cg.add(var.set_wb_mode(config[CONF_WB_MODE]))
    if CONF_AEC_MODE in config:
        cg.add(var.set_aec_mode(config[CONF_AEC_MODE]))
    if CONF_AE_LEVEL in config:
        cg.add(var.set_ae_level(config[CONF_AE_LEVEL]))
    if CONF_AGC_MODE in config:
        cg.add(var.set_agc_mode(config[CONF_AGC_MODE]))
    if CONF_SHARPNESS in config:
        cg.add(var.set_sharpness(config[CONF_SHARPNESS]))
    if CONF_DENOISE in config:
        cg.add(var.set_denoise(config[CONF_DENOISE]))
    if CONF_DEAD_PIXEL_CORRECTION in config:
        cg.add(var.set_dead_pixel_correction(config[CONF_DEAD_PIXEL_CORRECTION]))
    if CONF_BLACK_LEVEL_CORRECTION in config:
        cg.add(var.set_black_level_correction(config[CONF_BLACK_LEVEL_CORRECTION]))
    if CONF_LENS_SHADING_CORRECTION in config:
        cg.add(var.set_lens_shading_correction(config[CONF_LENS_SHADING_CORRECTION]))
    if CONF_NIGHT_MODE in config:
        cg.add(var.set_night_mode(config[CONF_NIGHT_MODE]))
    cg.add(var.set_bayer_order(config[CONF_BAYER_ORDER]))
    cg.add(var.set_saturation(config[CONF_SATURATION]))


@automation.register_action(
    "csi_camera.set_night_mode",
    CsiCameraSetNightModeAction,
    cv.maybe_simple_value(
        {
            cv.Required(CONF_ID): cv.use_id(CsiCamera),
            cv.Required(CONF_NIGHT_MODE): cv.templatable(cv.boolean),
        },
        key=CONF_NIGHT_MODE,
    ),
)
async def csi_camera_set_night_mode_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    template_ = await cg.templatable(config[CONF_NIGHT_MODE], args, bool)
    cg.add(var.set_night_mode(template_))
    return var
