import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import STATE_CLASS_MEASUREMENT
from esphome import pins

CONF_UVA = "uva"
CONF_UVB = "uvb"
CONF_UVC = "uvc"
CONF_UV_INDEX = "uv_index"
CONF_ILLUMINANCE = "illuminance"
CONF_IRRADIANCE = "irradiance"
CONF_IRRA_ADC_PIN = "irra_adc_pin"

elv_ns = cg.esphome_ns.namespace("elv_am_ors")
ELVAMORS = elv_ns.class_("ELVAMORS", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ELVAMORS),

            cv.Optional(CONF_UVA): sensor.sensor_schema(
                unit_of_measurement="W/m²",
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UVB): sensor.sensor_schema(
                unit_of_measurement="W/m²",
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UVC): sensor.sensor_schema(
                unit_of_measurement="W/m²",
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UV_INDEX): sensor.sensor_schema(
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ILLUMINANCE): sensor.sensor_schema(
                unit_of_measurement="lx",
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_IRRADIANCE): sensor.sensor_schema(
                unit_of_measurement="W/m²",
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),

            cv.Optional(
                CONF_IRRA_ADC_PIN,
                default="GPIO3"
            ): pins.internal_gpio_input_pin_schema,
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(i2c.i2c_device_schema(0x77))
)

async def to_code(config):
    var = cg.new_Pvariable(config[cg.CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_IRRA_ADC_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_IRRA_ADC_PIN])
        cg.add(var.set_irra_adc_pin(pin))

    for key in (
        CONF_UVA,
        CONF_UVB,
        CONF_UVC,
        CONF_UV_INDEX,
        CONF_ILLUMINANCE,
        CONF_IRRADIANCE,
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))
