import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID, CONF_RESET_PIN, CONF_BUSY_PIN

CODEOWNERS = ["@ssieb"]

MULTI_CONF = True

pic_programer_ns = cg.esphome_ns.namespace("pic_programer")

Pic_Programer = pic_programer_ns.class_("Pic_Programer", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Pic_Programer),
        cv.Required(CONF_RESET_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_BUSY_PIN): pins.internal_gpio_input_pin_schema,
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_Reset_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    cg.add(var.set_Busy_pin(pin))

    # displayVals = {False, False, False, False, False}
    # if CONFIG_SHOW_TEMP_SYMBOL in config:

    # cg.add(var.SetDisplayFurnishings(False))
