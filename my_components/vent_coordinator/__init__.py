from esphome.components import cover, sensor, packet_transport
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
    CONF_MAX_CURRENT,
)


CODEOWNERS = ["@jacob"]
# AUTO_LOAD = ["LEDCOutput"]


CONF_VENTS = "vents"
CONF_COVER_POSITION_SENSOR = "cover_position_sensor"
CONF_COVER = "cover"
CONF_REMOTE_PROVIDER = "remote_provider"
CONF_REMOTE_VENT_ID = "remote_id"
CONF_LOCAL_VENT_ID = "control_id"
CONF_TRANSPORT = "transport"

MULTI_CONF = True
AUTO_LOAD = ["cover", "sensor", "packet_transport"]

vent_coordinator_ns = cg.esphome_ns.namespace("vent_coordinator")
Vent_Coordinator = vent_coordinator_ns.class_("Vent_Coordinator", cg.Component)

VentConfig = vent_coordinator_ns.class_("VentConfig")


def validate_vent(config):
    isLocal = CONF_COVER in config
    isRemote = CONF_REMOTE_PROVIDER in config
    if isLocal == isRemote:
        raise cv.Invalid("Specify exactly one of 'cover' or 'remote_provider'")
    if isLocal and CONF_LOCAL_VENT_ID not in config:
        raise cv.Invalid("Local vent needs control_id")
    if isRemote and CONF_REMOTE_VENT_ID not in config:
        raise cv.Invalid("Remote vent needs remote_id")
    if not (isLocal or isRemote):
        raise cv.Invalid("Must specify either 'cover' or 'remote_provider'")
    return config


SCHEMA_VENT = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VentConfig),
            cv.Required(CONF_COVER_POSITION_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_COVER): cv.use_id(cover.Cover),
            cv.Optional(CONF_REMOTE_PROVIDER): cv.valid_name,
            cv.Optional(CONF_REMOTE_VENT_ID): cv.uint16_t,
            cv.Optional(CONF_LOCAL_VENT_ID): cv.uint16_t,
        }
    ),
    validate_vent,
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Vent_Coordinator),
            cv.Required(CONF_VENTS): cv.ensure_list(SCHEMA_VENT),
            cv.Optional(CONF_TRANSPORT): cv.use_id(packet_transport.PacketTransport),
        }
    )
    .extend(cv.polling_component_schema("10s")),
    # cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    vents = []
    for vent_config in config[CONF_VENTS]:
        vent_var = cg.new_Pvariable(vent_config[CONF_ID])
        position_sensor = await cg.get_variable(vent_config[CONF_COVER_POSITION_SENSOR])
        cg.add(vent_var.set_Cover_Position_Sensor(position_sensor))

        if CONF_COVER in vent_config:
            cover_var = await cg.get_variable(vent_config[CONF_COVER])
            cg.add(vent_var.set_Cover(cover_var))
            cg.add(vent_var.set_Local_Vent_ID(vent_config[CONF_LOCAL_VENT_ID]))
        else:
            cg.add(vent_var.set_Remote_Provider(vent_config[CONF_REMOTE_PROVIDER]))
            cg.add(vent_var.set_Remote_ID(vent_config[CONF_REMOTE_VENT_ID]))
        vents.append(vent_var)
    cg.add(var.set_vents(vents))

    if CONF_TRANSPORT in config:
        transport = await cg.get_variable(config[CONF_TRANSPORT])
        cg.add(var.set_transport(transport))
