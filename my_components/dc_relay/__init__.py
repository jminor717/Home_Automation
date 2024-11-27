from esphome.components import sensor, voltage_sampler, switch, output#, i2c
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
    CONF_TRIGGER_ID,
    CONF_VOLTAGE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_CURRENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_VOLT,
    CONF_NAME,
    DEVICE_CLASS_RESTART,
    ENTITY_CATEGORY_CONFIG,
    ICON_RESTART_ALERT,
)

CODEOWNERS = ["@ssieb"]
CONF_CIRCUITS = "circuits"
CONF_V_IN_SENSOR = "v_in_sensor"
CONF_V_OUT_SENSOR = "v_out_sensor"
CONF_CURRENT_SENSOR = "current_sensor"
CONF_SHORT_CIRCUIT_TEST_CHANEL = "short_circuit_test_chanel"
CONF_UVLO = "uvlo"
CONF_VOLTAGE_RATIO = "voltage_divider_ratio"
CONF_CURRENT_RATIO = "current_calibration"
CONF_SHORT_CIRCUIT_TEST_PIN = "short_circuit_test_pin"
CONF_ENABLE_CIRCUIT = "enable_circuit"

MULTI_CONF = True

dc_relay_ns = cg.esphome_ns.namespace("dc_relay")

Dc_Relay = dc_relay_ns.class_("Dc_Relay", cg.Component)

CircuitConfig = dc_relay_ns.class_("CircuitConfig")

EnableCircuitSwitch = dc_relay_ns.class_("CircuitEnable", switch.Switch)

# LEDCOutput = dc_relay_ns.class_("customLEDCOutput", output.FloatOutput, cg.Component)
ledc_ns = cg.esphome_ns.namespace("ledc")
LEDCOutput = ledc_ns.class_("LEDCOutput", output.FloatOutput, cg.Component)

SCHEMA_CIRCUIT = {
    cv.GenerateID(): cv.declare_id(CircuitConfig),
    cv.Required(CONF_ENABLE_CIRCUIT): switch.switch_schema(
        EnableCircuitSwitch,
        entity_category=ENTITY_CATEGORY_CONFIG,
        default_restore_mode="RESTORE_DEFAULT_OFF",
    ),
    cv.Required(CONF_V_OUT_SENSOR): cv.use_id(voltage_sampler.VoltageSampler),
    cv.Required(CONF_CURRENT_SENSOR): cv.use_id(voltage_sampler.VoltageSampler),
    cv.Required(CONF_ENABLE_PIN): pins.internal_gpio_input_pin_schema,
    cv.Optional(CONF_SHORT_CIRCUIT_TEST_PIN): pins.internal_gpio_input_pin_schema,

    cv.Optional(CONF_POWER): sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=1,
    ),
    cv.Optional(CONF_CURRENT): sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=2,
    ),
}



CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Dc_Relay),
            cv.Required(CONF_V_IN_SENSOR): cv.use_id(voltage_sampler.VoltageSampler),
            cv.Optional(CONF_UVLO, default=0): cv.positive_float,
            cv.Optional(CONF_VOLTAGE_RATIO, default=1): cv.positive_float,
            cv.Optional(CONF_CURRENT_RATIO, default=1): cv.positive_float,
            cv.Required(CONF_CIRCUITS): cv.ensure_list(SCHEMA_CIRCUIT),
            cv.Optional(CONF_SHORT_CIRCUIT_TEST_CHANEL): cv.use_id(LEDCOutput),
            
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
    cg.add(var.set_Voltage_Divider_Ratio(config[CONF_VOLTAGE_RATIO]))
    cg.add(var.set_Current_Calibration(config[CONF_CURRENT_RATIO]))

    vin_sens = await cg.get_variable(config[CONF_V_IN_SENSOR])
    cg.add(var.set_Vin_Sensor(vin_sens))

    chan = await cg.get_variable(config[CONF_SHORT_CIRCUIT_TEST_CHANEL])
    cg.add(var.set_Short_Circuit_Test_Chanel(chan))

    circuits = []
    for circuit_config in config[CONF_CIRCUITS]:
        circuit_var = cg.new_Pvariable(circuit_config[CONF_ID], CircuitConfig())
        
        if enable_config := config.get(CONF_ENABLE_CIRCUIT):
            b = await switch.new_switch(enable_config)
            await cg.register_parented(b, config[CONF_ID])
            cg.add(circuit_var.set_Enable_Circuit_Switch(b))
        
        # phase_var = await cg.get_variable(circuit_config[CONF_PHASE_ID])
        v_out_sens = await cg.get_variable(circuit_config[CONF_V_OUT_SENSOR])
        cg.add(circuit_var.set_V_out_sensor(v_out_sens))
        i_out_sens = await cg.get_variable(circuit_config[CONF_CURRENT_SENSOR])
        cg.add(circuit_var.set_I_out_sensor(i_out_sens))

        pin = await cg.gpio_pin_expression(circuit_config[CONF_ENABLE_PIN])
        cg.add(circuit_var.set_Enable_pin(pin))

        if CONF_SHORT_CIRCUIT_TEST_PIN in circuit_config:
            pin = await cg.gpio_pin_expression(circuit_config[CONF_SHORT_CIRCUIT_TEST_PIN])
            cg.add(circuit_var.set_Short_Circuit_Test_pin(pin))

        if CONF_POWER in circuit_config:
            power_sensor = await sensor.new_sensor(circuit_config[CONF_POWER])
            cg.add(circuit_var.set_power_sensor(power_sensor))

        if CONF_CURRENT in circuit_config:
            current_sensor = await sensor.new_sensor(circuit_config[CONF_CURRENT])
            cg.add(circuit_var.set_current_sensor(current_sensor))

        # if CONF_CURRENT in circuit_config:
        #     current_sensor = await sensor.new_sensor(circuit_config[CONF_CURRENT])
        #     cg.add(circuit_var.set_current_sensor(current_sensor))

        circuits.append(circuit_var)
    cg.add(var.set_circuits(circuits))

    # displayVals = {False, False, False, False, False}
    # if CONFIG_SHOW_TEMP_SYMBOL in config:

    # cg.add(var.SetDisplayFurnishings(False))
