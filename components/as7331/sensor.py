import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import UNIT_WATT_PER_SQUARE_METER

as7331_ns = cg.esphome_ns.namespace("as7331")
AS7331Component = as7331_ns.class_(
    "AS7331Component", cg.Component, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS7331Component),
            cv.Optional("uva"): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_PER_SQUARE_METER
            ),
            cv.Optional("uvb"): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_PER_SQUARE_METER
            ),
            cv.Optional("uvc"): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_PER_SQUARE_METER
            ),
            cv.Optional("gain", default=5): cv.int_range(min=0, max=11),
            cv.Optional("integration_time", default=8): cv.int_range(min=0, max=15),
        }
    ).extend(i2c.i2c_device_schema(0x77))
)

async def to_code(config):
    var = cg.new_Pvariable(config[cg.CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_gain(config["gain"]))
    cg.add(var.set_integration_time(config["integration_time"]))

    if "uva" in config:
        sens = await sensor.new_sensor(config["uva"])
        cg.add(var.set_uva_sensor(sens))
    if "uvb" in config:
        sens = await sensor.new_sensor(config["uvb"])
        cg.add(var.set_uvb_sensor(sens))
    if "uvc" in config:
        sens = await sensor.new_sensor(config["uvc"])
        cg.add(var.set_uvc_sensor(sens))
