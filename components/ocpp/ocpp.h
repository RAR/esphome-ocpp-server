#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <array>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace esphome {

namespace sensor { class Sensor; }
namespace text_sensor { class TextSensor; }
namespace binary_sensor { class BinarySensor; }
namespace number { class Number; }

namespace ocpp {

class WsClient;
class MoConnection;

enum class MeterValueField : uint8_t {
  VOLTAGE,
  CURRENT,
  POWER,
  ENERGY,
  // Optional extras — bound via the same `meter_values:` block. Each maps to
  // a single OCPP measurand string and a fixed unit. `temperature` defaults
  // to Location=Body (charger interior) since that's where most EVSEs put
  // their thermal sensor; `soc` ships Location=EV.
  TEMPERATURE,
  SOC,
  FREQUENCY,
  POWER_FACTOR,
};

class OcppCp : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_csms_url(const std::string &u) { csms_url_ = u; }
  void set_charge_point_id(const std::string &id) { cp_id_ = id; }
  void set_vendor(const std::string &v) { vendor_ = v; }
  void set_model(const std::string &m) { model_ = m; }
  void set_firmware_version(const std::string &v) { firmware_version_ = v; }
  void set_phase(const std::string &p) { phase_ = p; }
  // Country / regional knobs. nominal_voltage is the EVSE's expected line
  // voltage (US split-phase 240, US outlet 120, EU single-phase 230,
  // EU 3-phase line-to-line 400). Used as a Power.Offered fallback when no
  // live voltage sensor is bound, and exposed via get_nominal_voltage() so
  // YAML lambdas (e.g. W→A conversion in on_charging_profile_change) can
  // reference one source of truth instead of hardcoding 240.
  void set_nominal_voltage(float v) { nominal_voltage_ = v; }
  float get_nominal_voltage() const { return nominal_voltage_; }
  // True iff this EVSE has a 1p/3p switchable contactor. Drives the OCPP
  // ConnectorSwitch3to1PhaseSupported declaration that suppresses evcc's
  // "Phase switch: yes" UI heuristic. Single-phase EVSEs (most US) leave
  // this false; EU 3-phase switchable EVSEs set true.
  void set_phase_switching_supported(bool b) { phase_switching_supported_ = b; }
  // True for EVSEs whose hardware can't reprogram the offered current
  // mid-transaction. While a transaction is active, Current.Offered is
  // pinned to the snapshot taken at StartTx (instead of tracking the
  // user-bound Number). evcc reads the snapshot and stops logging
  // "current mismatch" when it tries — and fails — to derate mid-session.
  void set_lock_offered_current_during_transaction(bool b) {
    lock_offered_current_during_transaction_ = b;
  }

  void set_meter_value_sensor(MeterValueField f, sensor::Sensor *s) { meter_sensors_[f] = s; }
  // 3-phase mode: per-leg sensors keyed by (measurand, leg-index 0..2).
  // Only Voltage and Current.Import are split per-leg; Power / Energy stay
  // scalar (OCPP 1.6 measurands are already cross-phase totals). Pairs
  // with the phase_tags_ vector at register_callbacks_ time.
  void set_meter_value_sensor_phase(MeterValueField f, int leg,
                                    sensor::Sensor *s) {
    if (leg < 0 || leg > 2) return;
    meter_sensors_phase_[f][leg] = s;
  }
  void add_phase_tag(const std::string &tag) { phase_tags_.push_back(tag); }
  void set_status_text_sensor(text_sensor::TextSensor *s) { status_sensor_ = s; }
  void add_status_mapping(const std::string &from, const std::string &to) { status_mapping_[from] = to; }
  void set_plugged_binary_sensor(binary_sensor::BinarySensor *s) { plugged_sensor_ = s; }
  // Optional second plug confirmation, specifically for SoC. Use when the
  // SoC source is a particular EV (e.g. R1T) and you want to suppress SoC
  // unless that EV's own "charger connected" entity also reports true —
  // otherwise a different car plugged in would get a phantom Rivian SoC.
  void set_soc_plugged_binary_sensor(binary_sensor::BinarySensor *s) { soc_plugged_sensor_ = s; }
  void set_heartbeat_interval(int seconds) { heartbeat_interval_s_ = seconds; }
  void set_connection_state_text_sensor(text_sensor::TextSensor *s) { connection_state_sensor_ = s; }
  void set_current_offered_number(number::Number *n) { current_offered_number_ = n; }

  void add_on_remote_start_callback(std::function<void(const std::string &)> cb) {
    remote_start_callbacks_.push_back(std::move(cb));
  }
  void add_on_remote_stop_callback(std::function<void(int)> cb) {
    remote_stop_callbacks_.push_back(std::move(cb));
  }
  void add_on_reset_callback(std::function<void(const std::string &)> cb) {
    reset_callbacks_.push_back(std::move(cb));
  }
  void add_on_unlock_callback(std::function<void(int)> cb) {
    unlock_callbacks_.push_back(std::move(cb));
  }
  void add_on_charging_profile_change_callback(
      std::function<void(float, float, int)> cb) {
    charging_profile_callbacks_.push_back(std::move(cb));
  }

  // YAML-callable: end the active OCPP transaction (sends StopTransaction.req
  // with the given reason). Used when the CSMS asks for a profile-level
  // disable (e.g. evcc's `SetChargingProfile{limit:0,unit:W}` pause) — without
  // closing the transaction, evcc with `remotestart: true` keys Enabled() off
  // transaction state and never sees the disable take effect. `reason` should
  // match OCPP 1.6 Reason enum: Local / Other / Remote / EVDisconnected /
  // PowerLoss / Reboot / SoftReset / HardReset / DeAuthorized.
  void end_transaction(const std::string &reason = "Local");

 protected:
  void init_microocpp_();
  void register_callbacks_();
  void poll_status_();

  // Connector input derivation: MicroOcpp's state machine wants three booleans
  // (plugged, EV-ready, EVSE-ready). We compute them from the cached, already-
  // mapped OCPP status string so a single status_mapping is enough to drive the
  // entire StatusNotification + Start/StopTransaction lifecycle.
  bool is_plugged_() const;
  bool is_ev_ready_() const;
  bool is_evse_ready_() const;
  void enforce_heartbeat_interval_();
  void publish_connection_state_();
  // Builds the comma-separated MeterValuesSampledData string from the bound
  // sensors. HA-mirrored extras (Temperature, SoC, Frequency, Power.Factor)
  // are only included when their sensor currently has_state() — that lets us
  // drop SoC from the wire frame entirely while a vehicle integration is
  // unavailable, instead of reporting 0 or some sentinel.
  std::string build_mvsd_list_() const;
  void refresh_mvsd_();

  std::string csms_url_;
  std::string cp_id_;
  std::string vendor_;
  std::string model_;
  std::string firmware_version_;
  // Optional phase tag (e.g. "L1") attached to Voltage and Current.Import
  // measurands so evcc's PhaseCurrents / PhaseVoltages getters resolve them
  // — getPhaseKey() in evcc's connector.go expects "<measurand>.L<n>" rows.
  // Empty = no phase attribute, current behavior.
  std::string phase_;
  float nominal_voltage_{230.0f};
  bool phase_switching_supported_{false};
  // Offered-current lock state. lock_offered_current_during_transaction_
  // is the static config switch; offered_current_locked_ is true between
  // a StartTx and a StopTx while the switch is on; locked_offered_value_
  // captures Number->state at StartTx and is what Current.Offered reports
  // until the lock releases.
  bool lock_offered_current_during_transaction_{false};
  bool offered_current_locked_{false};
  float locked_offered_value_{0.0f};

  std::map<MeterValueField, sensor::Sensor *> meter_sensors_;
  // Per-phase voltage/current sensors. Outer key is the measurand
  // (Voltage / Current.Import); inner array indexes legs 0..2 corresponding
  // to phase_tags_[0..2]. Empty when single-phase mode is in effect.
  std::map<MeterValueField, std::array<sensor::Sensor *, 3>> meter_sensors_phase_;
  // Phase tags for 3-phase mode. Order pairs 1:1 with the legs configured
  // above. Empty when single-phase mode is in effect (the scalar `phase_`
  // field carries the single tag instead).
  std::vector<std::string> phase_tags_;
  text_sensor::TextSensor *status_sensor_{nullptr};
  text_sensor::TextSensor *connection_state_sensor_{nullptr};
  std::string last_connection_state_;
  binary_sensor::BinarySensor *plugged_sensor_{nullptr};
  binary_sensor::BinarySensor *soc_plugged_sensor_{nullptr};
  number::Number *current_offered_number_{nullptr};
  // True when the CSMS has installed a profile whose first period limit is 0
  // (i.e. evcc's `Enable(false)` → SetChargingProfile{limit:0}). MicroOcpp's
  // SmartChargingService interprets `limit:0` as "no constraint" and emits a
  // -1 sentinel from setSmartChargingPowerOutput, so we can't rely on the
  // output callback to detect a CSMS-imposed pause. Instead we set this flag
  // directly from the SetChargingProfile/ClearChargingProfile observers.
  // When true, Current.Offered / Power.Offered are reported as 0 so evcc's
  // Enabled()-via-offered-measurand fallback agrees with its own Enable(false)
  // call (otherwise it logs `charger out of sync`).
  bool csms_disabled_{false};
  std::map<std::string, std::string> status_mapping_;
  std::string last_source_status_;
  std::string mapped_status_{"Available"};
  int heartbeat_interval_s_{0};  // 0 = let CSMS decide
  uint32_t last_hb_check_ms_{0};
  uint32_t last_mvsd_check_ms_{0};

  std::vector<std::function<void(const std::string &)>> remote_start_callbacks_;
  std::vector<std::function<void(int)>> remote_stop_callbacks_;
  std::vector<std::function<void(const std::string &)>> reset_callbacks_;
  std::vector<std::function<void(int)>> unlock_callbacks_;
  std::vector<std::function<void(float, float, int)>> charging_profile_callbacks_;

  WsClient *ws_{nullptr};
  MoConnection *mo_conn_{nullptr};
  bool initialized_{false};
};

}  // namespace ocpp
}  // namespace esphome
