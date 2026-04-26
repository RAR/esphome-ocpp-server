"""OCPP 1.6J Charge Point component for ESPHome (via MicroOcpp).

Top-level YAML surface:

    ocpp:
      id: ocpp_cp
      csms_url: ws://host:8180/steve/websocket/CentralSystemService/my-cp
      charge_point_id: my-cp
      vendor: MyVendor
      model: MyModel
      firmware_version: "1.0.0"
      meter_values:
        voltage: voltage_sensor_id
        current: current_sensor_id
        power: power_sensor_id
        energy: energy_sensor_id
      status_from: status_text_sensor_id
      status_mapping:
        "local-state-A": Charging
        "local-state-B": SuspendedEV
      on_remote_start:
        - lambda: id(my_charger)->start_charge(id_tag);
      on_remote_stop:
        - lambda: id(my_charger)->stop_charge();
      on_reset:
        - button.press: restart_btn
      on_unlock_connector:
        - lambda: id(my_charger)->unlock(connector_id);
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor, text_sensor, binary_sensor, number
from esphome.const import CONF_ID, CONF_TRIGGER_ID

CODEOWNERS = ["@andrewrankin"]
DEPENDENCIES = ["wifi"]

ocpp_ns = cg.esphome_ns.namespace("ocpp")
OcppCp = ocpp_ns.class_("OcppCp", cg.Component)

RemoteStartTrigger = ocpp_ns.class_("RemoteStartTrigger", automation.Trigger.template(cg.std_string))
RemoteStopTrigger = ocpp_ns.class_("RemoteStopTrigger", automation.Trigger.template(cg.int_))
ResetTrigger = ocpp_ns.class_("ResetTrigger", automation.Trigger.template(cg.std_string))
UnlockConnectorTrigger = ocpp_ns.class_("UnlockConnectorTrigger", automation.Trigger.template(cg.int_))
ChargingProfileChangeTrigger = ocpp_ns.class_(
    "ChargingProfileChangeTrigger",
    automation.Trigger.template(cg.float_, cg.float_, cg.int_),
)

MeterValueField = ocpp_ns.enum("MeterValueField", is_class=True)
_FIELDS = {
    "voltage": MeterValueField.VOLTAGE,
    "current": MeterValueField.CURRENT,
    "power": MeterValueField.POWER,
    "energy": MeterValueField.ENERGY,
    "temperature": MeterValueField.TEMPERATURE,
    "soc": MeterValueField.SOC,
    "frequency": MeterValueField.FREQUENCY,
    "power_factor": MeterValueField.POWER_FACTOR,
}

CONF_CSMS_URL = "csms_url"
CONF_CHARGE_POINT_ID = "charge_point_id"
CONF_VENDOR = "vendor"
CONF_MODEL = "model"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_PHASE = "phase"
CONF_NOMINAL_VOLTAGE = "nominal_voltage"
CONF_PHASE_SWITCHING_SUPPORTED = "phase_switching_supported"
CONF_METER_VALUES = "meter_values"
CONF_STATUS_FROM = "status_from"
CONF_STATUS_MAPPING = "status_mapping"
CONF_PLUGGED_FROM = "plugged_from"
CONF_SOC_PLUGGED_FROM = "soc_plugged_from"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_ON_REMOTE_START = "on_remote_start"
CONF_ON_REMOTE_STOP = "on_remote_stop"
CONF_ON_RESET = "on_reset"
CONF_ON_UNLOCK_CONNECTOR = "on_unlock_connector"
CONF_ON_CHARGING_PROFILE_CHANGE = "on_charging_profile_change"


CONF_CURRENT_OFFERED = "current_offered"

# `current_offered` accepts a number.Number (e.g. a `Max Charge Current` number
# entity) — that's the value the EVSE is offering on the J1772 PWM, which
# evcc / OCPP CSMSes commonly want as `Current.Offered` in MeterValues. Power
# Offered is auto-computed from current_offered × voltage when both are bound;
# we don't expose a separate `power_offered` slot.
METER_VALUES_SCHEMA = cv.Schema(
    {
        **{cv.Optional(key): cv.use_id(sensor.Sensor) for key in _FIELDS},
        cv.Optional(CONF_CURRENT_OFFERED): cv.use_id(number.Number),
    }
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OcppCp),
        cv.Required(CONF_CSMS_URL): cv.string_strict,
        cv.Required(CONF_CHARGE_POINT_ID): cv.string_strict,
        cv.Optional(CONF_VENDOR, default="ESPHome"): cv.string_strict,
        cv.Optional(CONF_MODEL, default="OCPP Charger"): cv.string_strict,
        cv.Optional(CONF_FIRMWARE_VERSION, default=""): cv.string_strict,
        # OCPP phase tag for Voltage / Current.Import measurands. evcc's
        # PhaseCurrents/PhaseVoltages interfaces only resolve when the
        # SampledValue carries `phase: L1|L2|L3` — without it those rows go
        # into evcc's generic measurement map and per-phase displays stay 0.
        # Single-phase EVSEs typically set this to "L1"; three-phase EVSEs
        # would need three separate addMeterValueInput calls (not yet
        # supported here).
        cv.Optional(CONF_PHASE): cv.one_of(
            "L1", "L2", "L3", "L1-N", "L2-N", "L3-N", upper=True
        ),
        # Country / regional knobs. nominal_voltage is the EVSE's expected
        # line voltage — US split-phase 240, US outlet 120, EU single-phase
        # 230, EU 3-phase line-to-line 400. Used as a Power.Offered fallback
        # when no live voltage sensor is bound, and as a one-source-of-truth
        # constant for YAML W→A conversion lambdas.
        cv.Optional(CONF_NOMINAL_VOLTAGE, default=230.0): cv.positive_float,
        # True iff this EVSE has a 1p/3p switchable contactor. Default false
        # (single-phase). EU 3-phase switchable EVSEs set true; this drives
        # the OCPP ConnectorSwitch3to1PhaseSupported declaration that evcc
        # reads to override its "Phase switch: yes" UI heuristic.
        cv.Optional(CONF_PHASE_SWITCHING_SUPPORTED, default=False): cv.boolean,
        cv.Optional(CONF_METER_VALUES): METER_VALUES_SCHEMA,
        cv.Optional(CONF_STATUS_FROM): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_STATUS_MAPPING, default={}): cv.Schema(
            {cv.string: cv.string_strict}
        ),
        cv.Optional(CONF_PLUGGED_FROM): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_SOC_PLUGGED_FROM): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_HEARTBEAT_INTERVAL): cv.positive_time_period_seconds,
        cv.Optional(CONF_ON_REMOTE_START): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RemoteStartTrigger)}
        ),
        cv.Optional(CONF_ON_REMOTE_STOP): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RemoteStopTrigger)}
        ),
        cv.Optional(CONF_ON_RESET): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ResetTrigger)}
        ),
        cv.Optional(CONF_ON_UNLOCK_CONNECTOR): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(UnlockConnectorTrigger)}
        ),
        cv.Optional(CONF_ON_CHARGING_PROFILE_CHANGE): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ChargingProfileChangeTrigger)}
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # PlatformIO deps + build flags — sourced here so consumers don't have to
    # repeat them in their YAML. MicroOcpp + ArduinoJson are pulled from PIO
    # registry at build time.
    cg.add_library("bblanchon/ArduinoJson", "6.21.5")
    # PIO treats `name=url` as an alias; some PIO versions mis-parse the `=`
    # (git sees it as a protocol). Pass the VCS URL as the library name and
    # skip the alias form.
    cg.add_library("https://github.com/matth-x/MicroOcpp.git#v1.2.0", None)
    # MicroOcpp's library.json declares links2004/WebSockets as a transitive
    # dep even when MO_CUSTOM_WS is defined. That lib's WebSockets.h drags
    # in <Ethernet.h> which LibreTiny's Arduino core doesn't ship. Ignore it.
    cg.add_platformio_option("lib_ignore", ["WebSockets", "arduinoWebSockets"])
    cg.add_build_flag("-DMO_PLATFORM=MO_PLATFORM_ARDUINO")
    cg.add_build_flag("-DMO_CUSTOM_WS")
    # LibreTiny's Arduino Serial lacks printf_P (no PROGMEM distinction); route
    # MicroOcpp logs through a plain callback instead. Wired in OcppCp::setup().
    cg.add_build_flag("-DMO_CUSTOM_CONSOLE")
    cg.add_build_flag("-DMO_USE_FILEAPI=DISABLE_FS")
    cg.add_build_flag("-DMO_ENABLE_MBEDTLS=0")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_csms_url(config[CONF_CSMS_URL]))
    cg.add(var.set_charge_point_id(config[CONF_CHARGE_POINT_ID]))
    cg.add(var.set_vendor(config[CONF_VENDOR]))
    cg.add(var.set_model(config[CONF_MODEL]))
    if config[CONF_FIRMWARE_VERSION]:
        cg.add(var.set_firmware_version(config[CONF_FIRMWARE_VERSION]))

    if CONF_PHASE in config:
        cg.add(var.set_phase(config[CONF_PHASE]))

    cg.add(var.set_nominal_voltage(config[CONF_NOMINAL_VOLTAGE]))
    cg.add(var.set_phase_switching_supported(config[CONF_PHASE_SWITCHING_SUPPORTED]))

    if CONF_METER_VALUES in config:
        for key, enum_val in _FIELDS.items():
            if key in config[CONF_METER_VALUES]:
                s = await cg.get_variable(config[CONF_METER_VALUES][key])
                cg.add(var.set_meter_value_sensor(enum_val, s))
        if CONF_CURRENT_OFFERED in config[CONF_METER_VALUES]:
            n = await cg.get_variable(config[CONF_METER_VALUES][CONF_CURRENT_OFFERED])
            cg.add(var.set_current_offered_number(n))

    if CONF_STATUS_FROM in config:
        ts = await cg.get_variable(config[CONF_STATUS_FROM])
        cg.add(var.set_status_text_sensor(ts))

    if CONF_PLUGGED_FROM in config:
        bs = await cg.get_variable(config[CONF_PLUGGED_FROM])
        cg.add(var.set_plugged_binary_sensor(bs))

    if CONF_SOC_PLUGGED_FROM in config:
        bs = await cg.get_variable(config[CONF_SOC_PLUGGED_FROM])
        cg.add(var.set_soc_plugged_binary_sensor(bs))

    for k, v in config[CONF_STATUS_MAPPING].items():
        cg.add(var.add_status_mapping(k, v))

    if CONF_HEARTBEAT_INTERVAL in config:
        cg.add(var.set_heartbeat_interval(int(config[CONF_HEARTBEAT_INTERVAL].total_seconds)))

    for conf in config.get(CONF_ON_REMOTE_START, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trig, [(cg.std_string, "id_tag")], conf)
    for conf in config.get(CONF_ON_REMOTE_STOP, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trig, [(cg.int_, "transaction_id")], conf)
    for conf in config.get(CONF_ON_RESET, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trig, [(cg.std_string, "type")], conf)
    for conf in config.get(CONF_ON_UNLOCK_CONNECTOR, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trig, [(cg.int_, "connector_id")], conf)
    for conf in config.get(CONF_ON_CHARGING_PROFILE_CHANGE, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trig,
            [
                (cg.float_, "current_limit_a"),
                (cg.float_, "power_limit_w"),
                (cg.int_, "n_phases"),
            ],
            conf,
        )
