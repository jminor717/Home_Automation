import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID, CONF_RESET_PIN, CONF_BUSY_PIN
from esphome.components import voltage_sampler, servo

# from esphome.types import char_ptr


CODEOWNERS = ["@jacob"]

MULTI_CONF = True

CONF_POSITION_SENSOR = "servo_position_sensor"
CONF_SERVO = "servo"
CONF_OPEN_AT_CENTER = "open_at_center"
CONF_FLIP_OPEN = "switch_open_and_close"
CONF_OPEN_OFFSET = "open_offset"
CONF_CLOSE_OFFSET = "close_offset"

hard_stop_damper_ns = cg.esphome_ns.namespace("hard_stop_damper")

HardStopDamper = hard_stop_damper_ns.class_("HardStopDamper", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HardStopDamper),
        cv.Required(CONF_POSITION_SENSOR): cv.use_id(voltage_sampler.VoltageSampler),
        cv.Required(CONF_SERVO): cv.use_id(servo.Servo),
        cv.Optional(CONF_OPEN_AT_CENTER, default=False): cv.boolean,
        cv.Optional(CONF_FLIP_OPEN, default=False): cv.boolean,
        cv.Optional(CONF_OPEN_OFFSET, default=0.0): cv.float_range(min=-1.0, max=1.0),
        cv.Optional(CONF_CLOSE_OFFSET, default=0.0): cv.float_range(min=-1.0, max=1.0),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_open_at_center(config[CONF_OPEN_AT_CENTER]))
    cg.add(var.set_switch_open_and_close(config[CONF_FLIP_OPEN]))
    cg.add(var.set_open_offset(config[CONF_OPEN_OFFSET]))
    cg.add(var.set_close_offset(config[CONF_CLOSE_OFFSET]))

    sensor = await cg.get_variable(config[CONF_POSITION_SENSOR])
    cg.add(var.set_v_servo_sensor(sensor))
    servo = await cg.get_variable(config[CONF_SERVO])
    cg.add(var.set_servo(servo))


    # pin = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    # cg.add(var.set_Busy_pin(pin))