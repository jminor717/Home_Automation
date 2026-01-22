from esphome.components import sensor, voltage_sampler, switch, output  #, i2c, LEDCOutput
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_SENSOR,
    CONF_RESET_PIN, 
    CONF_BUSY_PIN,
    CONF_ENABLE_PIN,
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


CONF_CIRCUITS = "circuits"
CONF_V_IN_SENSOR = "v_in_sensor"
CONF_V_OUT_SENSOR = "v_out_sensor"
CONF_CURRENT_SENSOR = "current_sensor"
CONF_SHORT_CIRCUIT_TEST_CHANEL = "short_circuit_test_chanel"
CONF_UVLO = "uvlo"
CONF_OCV = "open_circuit_voltage"
CONF_VOLTAGE_RATIO = "voltage_divider_ratio"
CONF_CURRENT_ZERO_POINT = "current_zero_point_voltage"
CONF_CURRENT_GAIN = "current_gain"
CONF_SHORT_CIRCUIT_TEST_PIN = "short_circuit_test_pin"
CONF_ENABLE_CIRCUIT = "enable_circuit"
CONF_TEST_CURRENT = "short_circuit_test_current"
CONF_V_IN_DISPLAY = "v_in"

MULTI_CONF = True

battery_charge_controller_ns = cg.esphome_ns.namespace("charge_controller")

Charge_Controller = battery_charge_controller_ns.class_("Charge_Controller", cg.Component)


EnableCircuitSwitch = battery_charge_controller_ns.class_("CircuitEnable", switch.Switch, cg.Component)

# LEDCOutput = battery_charge_controller_ns.class_("customLEDCOutput", output.FloatOutput, cg.Component)
ledc_ns = cg.esphome_ns.namespace("ledc")
LEDCOutput = ledc_ns.class_("LEDCOutput", output.FloatOutput, cg.Component)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Charge_Controller),
            cv.Required(CONF_V_IN_SENSOR): cv.use_id(voltage_sampler.VoltageSampler),
            cv.Optional(CONF_UVLO, default=0): cv.positive_float,
            cv.Optional(CONF_OCV, default=12): cv.positive_float,
            cv.Optional(CONF_VOLTAGE_RATIO, default=1): cv.positive_float,
            cv.Optional(CONF_CURRENT_ZERO_POINT, default=0.100): cv.positive_float,
            cv.Optional(CONF_CURRENT_GAIN, default=124): cv.positive_float,
            # cv.Optional(CONF_SHORT_CIRCUIT_TEST_CHANEL): cv.use_id(LEDCOutput),
            
            cv.Optional(CONF_V_IN_DISPLAY): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
                accuracy_decimals=1,
            ),
        }
    )
    .extend(cv.polling_component_schema("500ms")),
    # .extend(i2c.i2c_device_schema(0x64)),
    cv.only_with_esp_idf,
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_UVLO(config[CONF_UVLO]))
    cg.add(var.set_OCV(config[CONF_OCV]))
    cg.add(var.set_Voltage_Divider_Ratio(config[CONF_VOLTAGE_RATIO]))
    cg.add(var.set_current_zero_point_voltage(config[CONF_CURRENT_ZERO_POINT]))
    cg.add(var.set_current_gain(config[CONF_CURRENT_GAIN]))

    vin_sens = await cg.get_variable(config[CONF_V_IN_SENSOR])
    cg.add(var.set_Vin_Sensor(vin_sens))



    if CONF_V_IN_DISPLAY in config:
        voltage_sensor = await sensor.new_sensor(config[CONF_V_IN_DISPLAY])
        cg.add(var.set_voltage_sensor(voltage_sensor))


