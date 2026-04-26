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
TriggerMessageTrigger = ocpp_ns.class_(
    "TriggerMessageTrigger",
    automation.Trigger.template(cg.std_string, cg.int_),
)
DataTransferTrigger = ocpp_ns.class_(
    "DataTransferTrigger",
    automation.Trigger.template(cg.std_string, cg.std_string, cg.std_string),
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
CONF_PHASES = "phases"
CONF_L1 = "l1"
CONF_L2 = "l2"
CONF_L3 = "l3"
CONF_NOMINAL_VOLTAGE = "nominal_voltage"
CONF_PHASE_SWITCHING_SUPPORTED = "phase_switching_supported"
CONF_LOCK_OFFERED_CURRENT_DURING_TRANSACTION = "lock_offered_current_during_transaction"
CONF_METER_VALUES = "meter_values"
CONF_STATUS_FROM = "status_from"
CONF_STATUS_MAPPING = "status_mapping"
CONF_PLUGGED_FROM = "plugged_from"
CONF_SOC_PLUGGED_FROM = "soc_plugged_from"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_METER_VALUE_SAMPLE_INTERVAL = "meter_value_sample_interval"
CONF_STOP_TXN_SAMPLED_DATA = "stop_txn_sampled_data"
CONF_ON_REMOTE_START = "on_remote_start"
CONF_ON_REMOTE_STOP = "on_remote_stop"
CONF_ON_RESET = "on_reset"
CONF_ON_UNLOCK_CONNECTOR = "on_unlock_connector"
CONF_ON_CHARGING_PROFILE_CHANGE = "on_charging_profile_change"
CONF_ON_TRIGGER_MESSAGE = "on_trigger_message"
CONF_ON_DATA_TRANSFER = "on_data_transfer"


CONF_CURRENT_OFFERED = "current_offered"

# `current_offered` accepts a number.Number (e.g. a `Max Charge Current` number
# entity) — that's the value the EVSE is offering on the J1772 PWM, which
# evcc / OCPP CSMSes commonly want as `Current.Offered` in MeterValues. Power
# Offered is auto-computed from current_offered × voltage when both are bound;
# we don't expose a separate `power_offered` slot.
#
# `voltage` and `current` accept either a single sensor (single-phase EVSE)
# OR a `{l1, l2, l3}` dict (3-phase EVSE — each leg becomes a separately
# tagged addMeterValueInput call so evcc's PhaseCurrents/PhaseVoltages
# getters resolve "<measurand>.L<n>" rows for each leg). Other measurands
# stay scalar — Power.Active.Import / Energy.Active.Import.Register are
# already sums in the OCPP 1.6 measurand semantics.
PHASE_DICT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_L1): cv.use_id(sensor.Sensor),
        cv.Required(CONF_L2): cv.use_id(sensor.Sensor),
        cv.Required(CONF_L3): cv.use_id(sensor.Sensor),
    }
)


def _voltage_or_current(value):
    # Polymorphic validator: accept either a single sensor ID (legacy
    # single-phase) or a {l1,l2,l3} dict (3-phase). Dict beats id-string in
    # cv ambiguity: cv.use_id always wins on a string, dict_form must be
    # tried first. ESPHome's id parser doesn't accept dicts so this is
    # unambiguous in practice.
    if isinstance(value, dict):
        return PHASE_DICT_SCHEMA(value)
    return cv.use_id(sensor.Sensor)(value)


_SCALAR_FIELDS = {k: v for k, v in _FIELDS.items() if k not in ("voltage", "current")}

METER_VALUES_SCHEMA = cv.Schema(
    {
        cv.Optional("voltage"): _voltage_or_current,
        cv.Optional("current"): _voltage_or_current,
        **{cv.Optional(key): cv.use_id(sensor.Sensor) for key in _SCALAR_FIELDS},
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
        # 3-phase opt-in. Presence of a `phases:` list switches the
        # component into multi-leg mode: voltage / current under
        # meter_values become {l1,l2,l3} dicts, and each phase tag listed
        # here is paired with the matching leg's sensor on a separate
        # addMeterValueInput call. Order matters: phases[0] tags l1,
        # phases[1] tags l2, phases[2] tags l3. Single-phase YAML keeps
        # using the scalar `phase:` key — the two are mutually exclusive
        # but we don't enforce that here; whichever fires first in
        # to_code wins (phases takes precedence below).
        cv.Optional(CONF_PHASES): cv.All(
            cv.ensure_list(
                cv.one_of("L1", "L2", "L3", "L1-N", "L2-N", "L3-N", upper=True)
            ),
            cv.Length(min=2, max=3),
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
        # Set true on EVSEs whose hardware can't change the offered current
        # mid-transaction (e.g. Rippleon ROC001 — writing the 0x1002 register
        # mid-session faults the MCU). When the connector is in an active
        # OCPP transaction we snapshot Current.Offered to whatever
        # current_offered (the bound Number) had at StartTx and lock the
        # MeterValues sample to that value until StopTx; the user-bound
        # Number can still change (HA UI / evcc reflect intent) but evcc's
        # measured-vs-offered comparison sees the immutable hardware truth
        # instead of a setpoint the EVSE silently ignored. Without this,
        # evcc logs `current mismatch` because the post-SetChargingProfile
        # readback shows the new setpoint but the actual current draw still
        # reflects the session-start cap. Default false: most EVSEs can
        # honour mid-session current changes and want Current.Offered to
        # track intent.
        cv.Optional(
            CONF_LOCK_OFFERED_CURRENT_DURING_TRANSACTION, default=False
        ): cv.boolean,
        cv.Optional(CONF_METER_VALUES): METER_VALUES_SCHEMA,
        cv.Optional(CONF_STATUS_FROM): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_STATUS_MAPPING, default={}): cv.Schema(
            {cv.string: cv.string_strict}
        ),
        cv.Optional(CONF_PLUGGED_FROM): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_SOC_PLUGGED_FROM): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_HEARTBEAT_INTERVAL): cv.positive_time_period_seconds,
        # Pin OCPP `MeterValueSampleInterval` (seconds between MeterValues
        # frames during a transaction). Same re-pin pattern as
        # heartbeat_interval — we override the value MicroOcpp / the CSMS
        # would otherwise pick on every loop, so a CSMS-side
        # ChangeConfiguration won't drift it back. evcc's "current
        # mismatch" remediation suggests ≥30s; some hardware-locked EVSEs
        # benefit from 60s+.
        cv.Optional(
            CONF_METER_VALUE_SAMPLE_INTERVAL
        ): cv.positive_time_period_seconds,
        # Separate measurand list shipped *only* with StopTransaction.req
        # (the per-transaction transcript), distinct from the periodic
        # MeterValues feed driven by MeterValuesSampledData. CSMSes that
        # bill from the StopTx transcript benefit from a tighter,
        # billing-only set (typically just energy + SoC). Empty / absent
        # leaves MO's default (= no sampled data in the StopTx).
        # Re-pinned every 5 s in loop() so a CSMS ChangeConfiguration
        # can't drift it back. Each entry must match an OCPP 1.6
        # measurand string and have a corresponding sensor under
        # meter_values:.
        cv.Optional(CONF_STOP_TXN_SAMPLED_DATA): cv.ensure_list(
            cv.string_strict
        ),
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
        cv.Optional(CONF_ON_TRIGGER_MESSAGE): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(TriggerMessageTrigger)}
        ),
        cv.Optional(CONF_ON_DATA_TRANSFER): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(DataTransferTrigger)}
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
    # Use our fork of MicroOcpp (branch `ocpp-server`, based on the v1.2.0
    # tag — our component targets that API surface; upstream main has since
    # refactored `Notification.h` and others). Carries two patches we
    # depend on:
    #
    #   1. MO_CONFIG_MAX_VALSTRSIZE made override-able via -D so the
    #      MeterValuesSampledData advertised list can exceed the upstream
    #      128-byte ceiling. Upstream PR: matth-x/MicroOcpp#450
    #
    #   2. Clock::now() guard against implausible time deltas (skips the
    #      update if delta > 1 h instead of applying a ~12 h jump caused
    #      by 32-bit tick-counter overflow). Defensive on Arduino too —
    #      millis() wraps at ~49.7 days. Cherry-picked from upstream PR
    #      matth-x/MicroOcpp#447 (just the Time.cpp commits — the espidf
    #      Platform.cpp fix doesn't apply to our MO_PLATFORM_ARDUINO
    #      build).
    #
    # Once both PRs land + tag, switch back to a tagged upstream URL and
    # drop the fork. Pin the SHA rather than the branch so future
    # force-pushes to the fork can't break us.
    cg.add_library(
        "https://github.com/RAR/MicroOcpp.git#0ed6ba1ce7ece21665bb1bec571fa2568e388e69",
        None,
    )
    # Raise the ceiling so the full 6-mandatory + 4-optional measurand
    # advertised list (138 chars at peak) survives setString. 256 bumps
    # static RAM cost by 128 B per Configuration<const char*> instance —
    # immaterial relative to the 50 KB total budget on BK7231N.
    cg.add_build_flag("-DMO_CONFIG_MAX_VALSTRSIZE=256")
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
    if CONF_PHASES in config:
        for tag in config[CONF_PHASES]:
            cg.add(var.add_phase_tag(tag))

    cg.add(var.set_nominal_voltage(config[CONF_NOMINAL_VOLTAGE]))
    cg.add(var.set_phase_switching_supported(config[CONF_PHASE_SWITCHING_SUPPORTED]))
    cg.add(
        var.set_lock_offered_current_during_transaction(
            config[CONF_LOCK_OFFERED_CURRENT_DURING_TRANSACTION]
        )
    )

    if CONF_METER_VALUES in config:
        mv = config[CONF_METER_VALUES]
        for key, enum_val in _FIELDS.items():
            if key not in mv:
                continue
            val = mv[key]
            if isinstance(val, dict):
                # Per-phase voltage/current. The C++ side stores up to three
                # sensors per measurand; tag/leg pairing is done via the
                # phases list at register_callbacks_ time.
                for leg, leg_key in enumerate((CONF_L1, CONF_L2, CONF_L3)):
                    if leg_key in val:
                        s = await cg.get_variable(val[leg_key])
                        cg.add(var.set_meter_value_sensor_phase(enum_val, leg, s))
            else:
                s = await cg.get_variable(val)
                cg.add(var.set_meter_value_sensor(enum_val, s))
        if CONF_CURRENT_OFFERED in mv:
            n = await cg.get_variable(mv[CONF_CURRENT_OFFERED])
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
    if CONF_METER_VALUE_SAMPLE_INTERVAL in config:
        cg.add(
            var.set_meter_value_sample_interval(
                int(config[CONF_METER_VALUE_SAMPLE_INTERVAL].total_seconds)
            )
        )
    if CONF_STOP_TXN_SAMPLED_DATA in config:
        cg.add(
            var.set_stop_txn_sampled_data(
                ",".join(config[CONF_STOP_TXN_SAMPLED_DATA])
            )
        )

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
    for conf in config.get(CONF_ON_TRIGGER_MESSAGE, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trig,
            [(cg.std_string, "requested_message"), (cg.int_, "connector_id")],
            conf,
        )
    for conf in config.get(CONF_ON_DATA_TRANSFER, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trig,
            [
                (cg.std_string, "vendor_id"),
                (cg.std_string, "message_id"),
                (cg.std_string, "data"),
            ],
            conf,
        )
