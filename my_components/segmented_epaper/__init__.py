import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_RESET_PIN, CONF_BUSY_PIN

CODEOWNERS = ["@ssieb"]

MULTI_CONF = True

segmented_epaper_ns = cg.esphome_ns.namespace("segmented_epaper")

Segmented_ePaper = segmented_epaper_ns.class_("Segmented_ePaper", cg.Component, i2c.I2CDevice)

"""
        bool show_temp_symbol = false;
        bool temp_is_fahrenheit = false;
        bool show_lower_decimal_point = false;
        bool show_upper_decimal_point = false;
        bool show_humidity_symbol = false;
"""

CONFIG_SHOW_TEMP_SYMBOL = "show_temp_symbol"
CONFIG_TEMP_SYMBOL_FAHRENHEIT = "temp_is_fahrenheit"
CONFIG_SHOW_LOWER_DECIMAL_POINT = "show_lower_decimal_point"
CONFIG_SHOW_UPPER_DECIMAL_POINT = "show_upper_decimal_point"
CONFIG_SHOW_HUMIDITY_SYMBOL = "show_humidity_symbol"


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Segmented_ePaper),
        cv.Required(CONF_RESET_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_BUSY_PIN): pins.internal_gpio_input_pin_schema,

        cv.Optional(CONFIG_SHOW_TEMP_SYMBOL, default=False): cv.boolean,
        cv.Optional(CONFIG_TEMP_SYMBOL_FAHRENHEIT, default=False): cv.boolean,
        cv.Optional(CONFIG_SHOW_LOWER_DECIMAL_POINT, default=False): cv.boolean,
        cv.Optional(CONFIG_SHOW_UPPER_DECIMAL_POINT, default=False): cv.boolean,
        cv.Optional(CONFIG_SHOW_HUMIDITY_SYMBOL, default=False): cv.boolean,
    }
).extend(i2c.i2c_device_schema(0x3C))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await i2c.register_i2c_device(var, config)
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_Reset_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    cg.add(var.set_Busy_pin(pin))

    # displayVals = {False, False, False, False, False}
    # if CONFIG_SHOW_TEMP_SYMBOL in config:

    # cg.add(var.SetDisplayFurnishings(False))
