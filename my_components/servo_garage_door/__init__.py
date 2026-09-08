from esphome.components import sensor, voltage_sampler, switch, output  #, i2c, LEDCOutput
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_ENABLE_PIN,
    CONF_DIR_PIN,
    CONF_STEP_PIN,


    CONF_SENSOR,
    CONF_RESET_PIN, 
    CONF_BUSY_PIN,
    CONF_INPUT,
    CONF_POWER,
    CONF_CURRENT,
    CONF_VOLTAGE,
    CONF_TRIGGER_ID,
    CONF_VOLTAGE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_VOLT,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_NONE,
    CONF_MAX_CURRENT
)


CODEOWNERS = ["@jacob"]
# AUTO_LOAD = ["LEDCOutput"]


CONF_MOTORS = "motors"
CONF_STEPS_PER_ROTATION = "steps_per_rotation"
CONF_ALARM_PIN = "alarm_pin"
CONF_IN_PLACE_PIN = "in_place_pin"

MULTI_CONF = False

servo_garage_door_ns = cg.esphome_ns.namespace("servo_garage_door")

Servo_Garage_Door = servo_garage_door_ns.class_("ServoGarageDoor", cg.Component)

ServoMotor = servo_garage_door_ns.class_("ServoMotor")


SCHEMA_MOTOR = {
    cv.GenerateID(): cv.declare_id(ServoMotor),
    cv.Required(CONF_STEP_PIN): pins.gpio_output_pin_schema,
    cv.Required(CONF_DIR_PIN): pins.gpio_output_pin_schema,

    cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_ALARM_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_IN_PLACE_PIN): pins.gpio_output_pin_schema,

    cv.Optional(CONF_STEPS_PER_ROTATION, default=400): cv.positive_int,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Servo_Garage_Door),
        
            cv.Required(CONF_MOTORS): cv.ensure_list(SCHEMA_MOTOR),

        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    motors = []
    for motor_config in config[CONF_MOTORS]:
        motor_var = cg.new_Pvariable(motor_config[CONF_ID], ServoMotor())
        
        pin = await cg.gpio_pin_expression(motor_config[CONF_STEP_PIN])
        cg.add(motor_var.set_Step_pin(pin))

        pin = await cg.gpio_pin_expression(motor_config[CONF_DIR_PIN])
        cg.add(motor_var.set_Dir_pin(pin))

        if CONF_ENABLE_PIN in motor_config:
            pin = await cg.gpio_pin_expression(motor_config[CONF_ENABLE_PIN])
            cg.add(motor_var.set_Enable_pin(pin))

        if CONF_ALARM_PIN in motor_config:
            pin = await cg.gpio_pin_expression(motor_config[CONF_ALARM_PIN])
            cg.add(motor_var.set_Alarm_pin(pin))

        if CONF_IN_PLACE_PIN in motor_config:
            pin = await cg.gpio_pin_expression(motor_config[CONF_IN_PLACE_PIN])
            cg.add(motor_var.set_In_Place_pin(pin))

        cg.add(motor_var.set_Steps_Per_Rotation(motor_config[CONF_STEPS_PER_ROTATION]))


        motors.append(motor_var)
    cg.add(var.set_motors(motors))
