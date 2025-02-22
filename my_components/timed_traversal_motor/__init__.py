import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID, CONF_RESET_PIN, CONF_BUSY_PIN
from esphome.components import voltage_sampler, servo

# from esphome.types import char_ptr

AUTO_LOAD = ["output", "hbridge"]
CODEOWNERS = ["@jacob"]

MULTI_CONF = True

timed_traversal_motor_ns = cg.esphome_ns.namespace("timed_traversal_motor")

TimedTraversalMotor = timed_traversal_motor_ns.class_("TimedTraversalMotor", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TimedTraversalMotor),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)



    # pin = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    # cg.add(var.set_Busy_pin(pin))