import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_RESET_PIN, CONF_BUSY_PIN

CODEOWNERS = ["@ssieb"]

MULTI_CONF = True

segmented_epaper_ns = cg.esphome_ns.namespace("segmented_epaper")

Segmented_ePaper = segmented_epaper_ns.class_("Segmented_ePaper", cg.Component, i2c.I2CDevice)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Segmented_ePaper),
        cv.Required(CONF_RESET_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_BUSY_PIN): pins.internal_gpio_input_pin_schema,
    }
).extend(i2c.i2c_device_schema(0x3C))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await i2c.register_i2c_device(var, config)
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_Reset_pin_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    cg.add(var.set_Busy_pin_pin(pin))
