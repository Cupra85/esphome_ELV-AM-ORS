import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
)

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

elv_am_ors_ns = cg.esphome_ns.namespace("elv_am_ors")
ELVAMORS = elv_am_ors_ns.class_("ELVAMORS", cg.PollingComponent, i2c.I2CDevice)

UNIT_W_M2 = "W/m²"

# YAML keys
CONF_AS7331_ADDRESS = "as7331_address"
CONF_OPT3001_ADDRESS = "opt3001_address"
CONF_GAIN = "as7331_gain"
CONF_INTEGRATION_TIME = "as7331_integration_time"

CONF_IRRA_ADC_PIN = "irra_adc_pin"
CONF_IRRA_ADC_REF_VOLTAGE = "irra_adc_ref_voltage"
CONF_IRRA_ADC_RESOLUTION = "irra_adc_resolution"
CONF_IRRA_SLOPE = "irra_slope_w_per_m2_per_v"
CONF_IRRA_OFFSET = "irra_offset_v"

CONF_UVA = "uva"
CONF_UVB = "uvb"
CONF_UVC = "uvc"
CONF_UVI = "uvi"
CONF_ILLUMINANCE = "illuminance"
CONF_IRRADIANCE = "irradiance"

GAIN_MAP = {
    2048: 0, 1024: 1, 512: 2, 256: 3, 128: 4, 64: 5,
    32: 6, 16: 7, 8: 8, 4: 9, 2: 10, 1: 11
}

INTEGRATION_TIME_MS_TO_CODE = {
    1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6,
    128: 7, 256: 8, 512: 9, 1024: 10, 2048: 11,
    4096: 12, 8192: 13, 16384: 14,
}

def _validate_gain(value):
    value = cv.int_(value)
    if value not in GAIN_MAP:
        raise cv.Invalid(
            "as7331_gain must be one of: "
            + ", ".join(str(k) for k in sorted(GAIN_MAP.keys(), reverse=True))
        )
    return value

def _validate_integration_time(value):
    value = cv.int_(value)
    if value not in INTEGRATION_TIME_MS_TO_CODE:
        raise cv.Invalid(
            "as7331_integration_time must be one of: "
            + ", ".join(str(k) for k in sorted(INTEGRATION_TIME_MS_TO_CODE.keys()))
        )
    return value

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ELVAMORS),

            cv.Optional(CONF_AS7331_ADDRESS, default=0x77): cv.hex_int,
            cv.Optional(CONF_OPT3001_ADDRESS, default=0x45): cv.hex_int,

            cv.Optional(CONF_GAIN, default=2048): _validate_gain,
            cv.Optional(CONF_INTEGRATION_TIME, default=64): _validate_integration_time,

            cv.Optional(CONF_IRRA_ADC_PIN, default=4): cv.int_range(min=0, max=48),
            cv.Optional(CONF_IRRA_ADC_REF_VOLTAGE, default=3.3): cv.float_range(min=1.0, max=5.5),
            cv.Optional(CONF_IRRA_ADC_RESOLUTION, default=4095): cv.int_range(min=255, max=65535),
            cv.Optional(CONF_IRRA_OFFSET, default=0.115): cv.float_,
            cv.Optional(CONF_IRRA_SLOPE, default=400.0): cv.float_,

            cv.Optional(CONF_UVA): sensor.sensor_schema(
                unit_of_measurement=UNIT_W_M2,
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UVB): sensor.sensor_schema(
                unit_of_measurement=UNIT_W_M2,
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UVC): sensor.sensor_schema(
                unit_of_measurement=UNIT_W_M2,
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UVI): sensor.sensor_schema(
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ILLUMINANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_LUX,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_IRRADIANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_W_M2,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x77))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_as7331_address(config[CONF_AS7331_ADDRESS]))
    cg.add(var.set_opt3001_address(config[CONF_OPT3001_ADDRESS]))

    cg.add(var.set_as7331_gain_code(GAIN_MAP[config[CONF_GAIN]]))
    cg.add(var.set_as7331_time_code(INTEGRATION_TIME_MS_TO_CODE[config[CONF_INTEGRATION_TIME]]))

    cg.add(var.set_irra_adc_pin(config[CONF_IRRA_ADC_PIN]))
    cg.add(var.set_irra_adc_ref_voltage(config[CONF_IRRA_ADC_REF_VOLTAGE]))
    cg.add(var.set_irra_adc_resolution(config[CONF_IRRA_ADC_RESOLUTION]))
    cg.add(var.set_irra_calibration(
        config[CONF_IRRA_SLOPE],
        config[CONF_IRRA_OFFSET],
    ))

    for key, setter in (
        (CONF_UVA, var.set_uva_sensor),
        (CONF_UVB, var.set_uvb_sensor),
        (CONF_UVC, var.set_uvc_sensor),
        (CONF_UVI, var.set_uvi_sensor),
        (CONF_ILLUMINANCE, var.set_illuminance_sensor),
        (CONF_IRRADIANCE, var.set_irradiance_sensor),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(setter(sens))
