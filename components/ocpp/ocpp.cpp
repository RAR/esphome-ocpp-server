#include "ocpp.h"

#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/api/api_server.h"

#include <cmath>

#include <MicroOcpp.h>
#include <MicroOcpp/Core/Configuration.h>
#include <MicroOcpp/Model/ConnectorBase/Notification.h>
#include <MicroOcpp/Model/Transactions/Transaction.h>

#include "ws_client.h"
#include "mo_connection.h"

namespace esphome {
namespace ocpp {

static const char *const TAG = "ocpp";

void OcppCp::setup() {
  ESP_LOGI(TAG, "Initializing OCPP CP id=%s url=%s", cp_id_.c_str(), csms_url_.c_str());

  // Route MicroOcpp's console output through ESPHome's logger. Must happen
  // before any MO call (internal asserts may call the console).
  mocpp_set_console_out([](const char *msg) { ESP_LOGD("microocpp", "%s", msg); });

  ws_ = new WsClient();
  ws_->set_url(csms_url_);
  ws_->set_subprotocol("ocpp1.6");

  mo_conn_ = new MoConnection(ws_);

  // Kick off WebSocket connect. MicroOcpp will start sending BootNotification
  // once the socket is open.
  ws_->begin();

  // Seed MicroOcpp. Filesystem disabled — state doesn't persist across reboots
  // on this port; MicroOcpp will re-issue BootNotification after every boot.
  ChargerCredentials creds(model_.c_str(), vendor_.c_str(),
                           firmware_version_.empty() ? nullptr : firmware_version_.c_str());
  mocpp_initialize(*mo_conn_, creds, nullptr);

  // OCPP 1.6 optional standard key. evcc and other CSMSes try to set this on
  // every CP; declaring it makes ChangeConfiguration return Accepted. We don't
  // honour the value — the WsClient pings at a fixed 20 s cadence regardless.
  // Must live in CONFIGURATION_VOLATILE (in-memory) since we run with
  // MO_USE_FILEAPI=DISABLE_FS — the default file-backed store silently drops
  // the declaration when the FS adapter isn't available.
  MicroOcpp::declareConfiguration<int>("WebSocketPingInterval", 20,
                                       CONFIGURATION_VOLATILE);

  // Pre-seed MeterValueSampleInterval before MO's MeteringConnector
  // declares its default. enforce_meter_value_sample_interval_() in loop()
  // would re-pin anyway, but seeding here means the first MeterValues
  // batch after BootNotification fires at the configured cadence.
  if (meter_value_sample_interval_s_ > 0) {
    MicroOcpp::declareConfiguration<int>("MeterValueSampleInterval",
                                         meter_value_sample_interval_s_);
  }

  // Tell evcc whether this is a 1p/3p switchable charger. evcc's PhaseSwitching
  // heuristic in cp_setup.go flips true when the CP advertises W-mode
  // SetChargingProfile support — which we do, so without this declaration
  // evcc shows "Phase switch: yes" in the UI. Declaring
  // ConnectorSwitch3to1PhaseSupported explicitly, read-only, lets evcc's
  // second-case match override the heuristic. Default false (single-phase
  // EVSE); EU 3-phase EVSEs with switchable contactors would set true.
  // CONFIGURATION_VOLATILE because we have no FS, read-only flag (4th arg)
  // so CSMS-side ChangeConfiguration can't flip it.
  MicroOcpp::declareConfiguration<bool>("ConnectorSwitch3to1PhaseSupported",
                                        phase_switching_supported_,
                                        CONFIGURATION_VOLATILE, true);

  // MO's MeteringService default for MeterValuesSampledData is just
  // "Energy.Active.Import.Register,Power.Active.Import" — voltage and current
  // are dropped from MeterValues even though we feed them in through
  // addMeterValueInput(). build_mvsd_list_ assembles the full advertised set,
  // and refresh_mvsd_ keeps it in sync at runtime as transient sensors (HA
  // mirrors of vehicle SoC, grid frequency, etc.) come and go.
  refresh_mvsd_();

  register_callbacks_();
  initialized_ = true;
}

void OcppCp::register_callbacks_() {
  // Meter value inputs — each is a lambda that reads the bound ESPHome sensor.
  auto energy_it = meter_sensors_.find(MeterValueField::ENERGY);
  if (energy_it != meter_sensors_.end() && energy_it->second != nullptr) {
    auto *s = energy_it->second;
    setEnergyMeterInput([s]() -> int {
      // OCPP wants cumulative energy in Wh; ESPHome sensors typically expose kWh.
      if (!s->has_state()) return 0;
      return static_cast<int>(s->state * 1000.0f);
    });
  }

  auto power_it = meter_sensors_.find(MeterValueField::POWER);
  if (power_it != meter_sensors_.end() && power_it->second != nullptr) {
    auto *s = power_it->second;
    setPowerMeterInput([s]() -> float {
      if (!s->has_state()) return 0.0f;
      // ESPHome power is typically in kW; OCPP wants W.
      return s->state * 1000.0f;
    });
  }

  // phase_.c_str() returns "" for unset; addMeterValueInput treats nullptr
  // and "" the same per MO source, but we pass nullptr explicitly when
  // unset for clarity.
  const char *phase_tag = phase_.empty() ? nullptr : phase_.c_str();
  // Voltage and Current.Import each support two registration paths:
  //   1) 3-phase mode — meter_sensors_phase_[F] has per-leg sensors paired
  //      with phase_tags_. Emit one addMeterValueInput per leg, each
  //      tagged "L1"/"L2"/"L3" so evcc's getPhaseKey() resolves
  //      Voltage.L1, Voltage.L2, Voltage.L3 (same for Current.Import).
  //   2) single-phase mode — fall back to the scalar meter_sensors_ slot
  //      tagged with the lone phase_ string (or nullptr if unset).
  // 3-phase mode is detected by a non-empty phase_tags_; legs without a
  // bound sensor are silently skipped.
  auto register_axis = [this, phase_tag](MeterValueField f,
                                         const char *measurand,
                                         const char *unit) {
    auto pit = meter_sensors_phase_.find(f);
    if (!phase_tags_.empty() && pit != meter_sensors_phase_.end()) {
      for (size_t leg = 0; leg < phase_tags_.size() && leg < 3; ++leg) {
        auto *s = pit->second[leg];
        if (s == nullptr) continue;
        const char *tag = phase_tags_[leg].c_str();
        addMeterValueInput(
            [s]() -> float { return s->has_state() ? s->state : 0.0f; },
            measurand, unit, nullptr, tag);
      }
      return;
    }
    auto it = meter_sensors_.find(f);
    if (it != meter_sensors_.end() && it->second != nullptr) {
      auto *s = it->second;
      addMeterValueInput(
          [s]() -> float { return s->has_state() ? s->state : 0.0f; },
          measurand, unit, nullptr, phase_tag);
    }
  };
  register_axis(MeterValueField::VOLTAGE, "Voltage", "V");
  register_axis(MeterValueField::CURRENT, "Current.Import", "A");

  // Current.Offered / Power.Offered. evcc requests both when configured with
  // remotestart + currentLimit so it can reflect the EVSE-offered cap in its
  // UI. Without registering them, MO emits "could not find metering device"
  // warnings on every ChangeConfiguration. Bind a number::Number that holds
  // the offered current (e.g. a Max Charge Current entity) and we publish
  // both Current.Offered = number.state and (if voltage is bound) the
  // computed Power.Offered = current_offered × voltage_rms.
  // Optional extras: Temperature, SoC, Frequency, Power.Factor.
  auto bind_extra = [this](MeterValueField f, const char *measurand,
                           const char *unit, const char *location) {
    auto it = meter_sensors_.find(f);
    if (it == meter_sensors_.end() || it->second == nullptr) return;
    auto *s = it->second;
    // Sanitize NaN to 0. Go's encoding/json can't marshal NaN floats; if
    // evcc's Battery / Phase-getter decorators are active because the
    // charger advertised the measurand at cp_setup time, dev.Soc() (etc.)
    // returning NaN crashes the JSON marshal of evcc's
    // /api/config/devices/charger/<id>/status — the response is 200 with an
    // empty body and the Configuration UI shows blank. Combined with the
    // dynamic-MVSD logic that excludes the measurand entirely when stale,
    // emitting 0 instead of NaN means evcc's last-known cached value stays
    // a real number.
    addMeterValueInput([s]() -> float {
      if (!s->has_state() || std::isnan(s->state)) return 0.0f;
      return s->state;
    }, measurand, unit, location);
  };
  // Temperature → charger body. Switch the location string at the call site
  // if you bind a different sensor (e.g. gun_temp would warrant Location=Cable).
  bind_extra(MeterValueField::TEMPERATURE, "Temperature", "Celsius", "Body");
  bind_extra(MeterValueField::SOC, "SoC", "Percent", "EV");
  // OCPP 1.6's unitOfMeasure enum doesn't include Hertz (added in 2.0.1) —
  // strict CSMSes (evcc) reject the whole MeterValues frame as a
  // GenericError when they see `unit: Hertz`. Pass nullptr so MO omits the
  // unit attribute entirely; the Frequency measurand still goes through.
  bind_extra(MeterValueField::FREQUENCY, "Frequency", nullptr, nullptr);
  bind_extra(MeterValueField::POWER_FACTOR, "Power.Factor", nullptr, nullptr);

  if (current_offered_number_ != nullptr) {
    auto *n = current_offered_number_;
    addMeterValueInput(
        [this, n]() -> float {
          if (csms_disabled_) return 0.0f;
          // Hardware-locked-during-transaction: report the snapshot taken
          // at StartTx, not the live Number state. The Number can still
          // move (HA UI / evcc reflect intent), but the wire value evcc
          // verifies against tracks what the EVSE is actually doing.
          if (offered_current_locked_) return locked_offered_value_;
          return std::isnan(n->state) ? 0.0f : n->state;
        },
        "Current.Offered", "A");
    // Power.Offered = Current.Offered × voltage × n_phases. The J1772 PWM
    // (and IEC 61851 Type 2) advertises a per-phase current cap; with N
    // legs each carrying that current at their per-leg voltage, the
    // installed-power offer is the sum across legs. Country knob:
    // 230 EU, 240 US split-phase, 120 US outlet, 400 EU 3-phase line-to-line.
    //   single-phase scalar sensor → V × I
    //   3-phase per-leg sensors    → (V_l1 + V_l2 + V_l3) × I (legs that
    //                                aren't reporting fall back to
    //                                nominal_voltage individually)
    //   no sensor bound at all     → nominal_voltage × I × max(1, n_legs)
    auto volt_it2 = meter_sensors_.find(MeterValueField::VOLTAGE);
    sensor::Sensor *v_scalar =
        (volt_it2 != meter_sensors_.end()) ? volt_it2->second : nullptr;
    auto volt_pit = meter_sensors_phase_.find(MeterValueField::VOLTAGE);
    bool three_phase = !phase_tags_.empty();
    std::array<sensor::Sensor *, 3> v_legs = {nullptr, nullptr, nullptr};
    if (three_phase && volt_pit != meter_sensors_phase_.end()) {
      v_legs = volt_pit->second;
    }
    int n_legs = three_phase ? static_cast<int>(phase_tags_.size()) : 1;
    addMeterValueInput(
        [this, n, v_scalar, v_legs, three_phase, n_legs]() -> float {
          if (csms_disabled_) return 0.0f;
          float i = std::isnan(n->state) ? 0.0f : n->state;
          float vsum = 0.0f;
          if (three_phase) {
            for (int leg = 0; leg < n_legs; ++leg) {
              auto *vs = v_legs[leg];
              vsum += (vs != nullptr && vs->has_state() && !std::isnan(vs->state))
                          ? vs->state
                          : nominal_voltage_;
            }
          } else {
            vsum = (v_scalar != nullptr && v_scalar->has_state() &&
                    !std::isnan(v_scalar->state))
                       ? v_scalar->state
                       : nominal_voltage_;
          }
          return i * vsum;
        },
        "Power.Offered", "W");
  }

  // Connector inputs — drive MicroOcpp's connector state machine. Without these
  // the connector stays "Available" forever and no StartTransaction ever fires.
  // We only register them if the user bound at least a status text_sensor
  // OR an explicit plugged binary_sensor; otherwise MO would believe the
  // cable is unplugged forever.
  if (status_sensor_ != nullptr || plugged_sensor_ != nullptr) {
    setConnectorPluggedInput([this]() -> bool { return this->is_plugged_(); });
    setEvReadyInput([this]() -> bool { return this->is_ev_ready_(); });
    setEvseReadyInput([this]() -> bool { return this->is_evse_ready_(); });
  }

  // SmartCharging output registration MUST come before the SetChargingProfile /
  // ClearChargingProfile observers below — MicroOcpp lazily creates
  // SmartChargingService inside the SmartCharging registration call, and that's
  // what registers those two operations in OperationRegistry.
  //
  // Use the unified setSmartChargingOutput rather than the per-axis
  // setSmartChargingCurrentOutput / setSmartChargingPowerOutput pair: both of
  // those internally write the same wrapper slot (the second call clobbers
  // the first), and the survivor sets ChargingScheduleAllowedChargingRateUnit
  // to "Power" — which is exactly the value evcc's PhaseSwitching heuristic
  // matches (`if v == "Power" || v == "W"` in cp_setup.go), causing it to
  // flip "Phase switch: yes" in the UI for any single-phase EVSE that
  // accepts W-mode profiles. Calling setSmartChargingOutput sets the unit
  // string to "Current,Power" instead — evcc's exact-string match fails,
  // PhaseSwitching stays false, *and* we still see both A- and W-mode
  // profiles via a single callback.
  //
  // Sentinels: power == -1.0f / current == -1.0f / nphases == -1 mean
  // "not set" (pass through hardware default). limit==0 from the engine
  // never fires for evcc-style `SetChargingProfile{limit:0}` because MO
  // interprets that as "no constraint" — see the SetChargingProfile
  // observer for the real disable-detection path.
  setSmartChargingOutput([this](float power, float current, int nphases) {
    ESP_LOGD(TAG, "SmartCharging output: %.1f W / %.1f A / %d phases", power,
             current, nphases);
    for (auto &cb : charging_profile_callbacks_) cb(current, power, nphases);
  });

  // RemoteStart/RemoteStopTransaction observers — purely for INFO-level
  // diagnostic logging on the request edge. The user-facing on_remote_start /
  // on_remote_stop YAML triggers fire later from setTxNotificationOutput()
  // below, on the *transaction lifecycle* (StartTx/StopTx) rather than on
  // request receipt — see comment there.
  setOnReceiveRequest("RemoteStartTransaction", [](JsonObject payload) {
    std::string id_tag = payload["idTag"] | "";
    int connector_id = payload["connectorId"] | 1;
    ESP_LOGI(TAG, "RemoteStartTransaction received: idTag='%s' connector=%d",
             id_tag.c_str(), connector_id);
  });
  setOnReceiveRequest("RemoteStopTransaction", [](JsonObject payload) {
    int tx_id = payload["transactionId"] | 0;
    ESP_LOGI(TAG, "RemoteStopTransaction received: transactionId=%d", tx_id);
  });

  // Log the raw profile from each SetChargingProfile so we can see exactly
  // what the CSMS is asking for (purpose / unit / per-period limits). MO's
  // SmartCharging engine still computes the effective limit and feeds it to
  // setSmartChargingCurrentOutput / setSmartChargingPowerOutput below — this
  // observer is purely diagnostic.
  setOnReceiveRequest("SetChargingProfile", [this](JsonObject payload) {
    int connector_id = payload["connectorId"] | -1;
    JsonObject prof = payload["csChargingProfiles"];
    if (prof.isNull()) {
      ESP_LOGW(TAG, "SetChargingProfile: missing csChargingProfiles");
      return;
    }
    int profile_id = prof["chargingProfileId"] | -1;
    const char *purpose = prof["chargingProfilePurpose"] | "?";
    const char *kind = prof["chargingProfileKind"] | "?";
    int stack_level = prof["stackLevel"] | -1;
    int tx_id = prof["transactionId"] | -1;
    JsonObject sched = prof["chargingSchedule"];
    const char *unit = sched["chargingRateUnit"] | "?";
    int duration = sched["duration"] | -1;
    ESP_LOGI(TAG, "SetChargingProfile: connector=%d id=%d purpose=%s kind=%s "
                  "stack=%d tx=%d unit=%s duration=%ds",
             connector_id, profile_id, purpose, kind, stack_level, tx_id, unit,
             duration);
    JsonArray periods = sched["chargingSchedulePeriod"];
    size_t i = 0;
    bool first_period_zero = false;
    bool first_period_positive = false;
    for (JsonObject p : periods) {
      int start = p["startPeriod"] | 0;
      float limit = p["limit"] | -1.0f;
      int phases = p["numberPhases"] | -1;
      if (i == 0 && start == 0) {
        if (limit == 0.0f) first_period_zero = true;
        else if (limit > 0.0f) first_period_positive = true;
      }
      if (phases >= 0) {
        ESP_LOGI(TAG, "  period[%u] start=+%ds limit=%.1f%s phases=%d",
                 (unsigned) i, start, limit, unit, phases);
      } else {
        ESP_LOGI(TAG, "  period[%u] start=+%ds limit=%.1f%s", (unsigned) i,
                 start, limit, unit);
      }
      i++;
    }
    // Only TxDefaultProfile / TxProfile actually gate charging. Skip
    // ChargePointMaxProfile (a global cap, not an enable/disable signal).
    bool is_tx_profile = strcmp(purpose, "TxDefaultProfile") == 0 ||
                         strcmp(purpose, "TxProfile") == 0;
    if (is_tx_profile) {
      if (first_period_zero) {
        if (!csms_disabled_) {
          ESP_LOGI(TAG, "  -> CSMS disable: forcing offered measurands to 0");
        }
        csms_disabled_ = true;
        // If MO has an active transaction (e.g. left over from evcc's
        // remotestart=true auto-RemoteStart on boot), end it so the connector
        // status moves to Finishing → Available and evcc stops reading
        // status=Charging. endTransaction is a no-op if no transaction is
        // running, so it's safe to call unconditionally.
        if (::isTransactionActive(1)) {
          ESP_LOGI(TAG, "  -> ending active OCPP transaction");
          ::endTransaction(nullptr, "Local", 1);
        }
      } else if (first_period_positive) {
        if (csms_disabled_) {
          ESP_LOGI(TAG, "  -> CSMS re-enable: offered measurands track Number");
        }
        csms_disabled_ = false;
      }
    }
  });

  // Same diagnostic pattern for ClearChargingProfile, plus a re-enable: when
  // the CSMS clears a profile we drop any in-effect disable so Current.Offered
  // tracks the user's Number again.
  setOnReceiveRequest("ClearChargingProfile", [this](JsonObject payload) {
    int profile_id = payload["id"] | -1;
    int connector_id = payload["connectorId"] | -1;
    const char *purpose = payload["chargingProfilePurpose"] | "?";
    int stack_level = payload["stackLevel"] | -1;
    ESP_LOGI(TAG,
             "ClearChargingProfile: id=%d connector=%d purpose=%s stack=%d",
             profile_id, connector_id, purpose, stack_level);
    if (csms_disabled_) {
      ESP_LOGI(TAG, "  -> CSMS re-enable (profile cleared)");
      csms_disabled_ = false;
    }
  });

  // Transaction lifecycle hook. setOnReceiveRequest fires the moment the
  // request hits the wire, *regardless of whether MO accepts it* — so a
  // RemoteStart for a Faulted/Unavailable connector would still trip the
  // user's lambda and tell the MCU to start charging into a dead state.
  // setTxNotificationOutput fires only on real transaction transitions:
  //   StartTx → connector has all conditions (plugged + EV ready + EVSE ready
  //             + authorized + operative) and is sending StartTransaction.req
  //   StopTx  → connector is ending the active transaction
  // RemoteStart/RemoteStop (capitalised) just mean "the request was accepted
  // and a transaction was begun/ended at the MO level"; we log them at INFO
  // for visibility but they don't drive the hardware.
  setTxNotificationOutput([this](MicroOcpp::Transaction *tx,
                                 MicroOcpp::TxNotification notification) {
    using N = MicroOcpp::TxNotification;
    int tx_id = tx ? tx->getTransactionId() : -1;
    const char *id_tag = (tx && tx->getIdTag()) ? tx->getIdTag() : "";
    switch (notification) {
      case N::RemoteStart:
        ESP_LOGI(TAG, "RemoteStart accepted: idTag='%s' tx=%d (waiting for "
                       "plug + EV/EVSE ready before StartTransaction)",
                 id_tag, tx_id);
        break;
      case N::RemoteStop:
        ESP_LOGI(TAG, "RemoteStop accepted: tx=%d", tx_id);
        break;
      case N::StartTx:
        ESP_LOGI(TAG, "StartTransaction triggered: idTag='%s' tx=%d", id_tag,
                 tx_id);
        // Snapshot the current_offered Number state for hardware-locked
        // EVSEs. Subsequent SetChargingProfile messages will still call
        // the user's on_charging_profile_change lambda (which usually
        // updates the Number for HA UI), but Current.Offered on the wire
        // stays at this snapshot until StopTx — so evcc's measured-vs-
        // offered comparison sees the actual hardware setpoint.
        if (lock_offered_current_during_transaction_ &&
            current_offered_number_ != nullptr) {
          float v = current_offered_number_->state;
          locked_offered_value_ = std::isnan(v) ? 0.0f : v;
          offered_current_locked_ = true;
          ESP_LOGI(TAG, "  -> Current.Offered locked at %.1f A for duration of "
                         "transaction (hardware can't change mid-session)",
                   locked_offered_value_);
        }
        for (auto &cb : remote_start_callbacks_) cb(std::string(id_tag));
        break;
      case N::StopTx:
        ESP_LOGI(TAG, "StopTransaction triggered: tx=%d", tx_id);
        if (offered_current_locked_) {
          ESP_LOGI(TAG, "  -> Current.Offered lock released");
          offered_current_locked_ = false;
        }
        for (auto &cb : remote_stop_callbacks_) cb(tx_id);
        break;
      case N::ConnectionTimeout:
        ESP_LOGW(TAG, "Transaction aborted: ConnectionTimeout (no plug "
                       "detected within ConnectionTimeOut s)");
        break;
      case N::AuthorizationRejected:
        ESP_LOGW(TAG, "Transaction aborted: AuthorizationRejected (idTag '%s')",
                 id_tag);
        break;
      case N::AuthorizationTimeout:
        ESP_LOGW(TAG, "Transaction aborted: AuthorizationTimeout (CSMS offline)");
        break;
      case N::DeAuthorized:
        ESP_LOGW(TAG, "Transaction aborted: DeAuthorized (CSMS rejected StartTransaction)");
        break;
      case N::ReservationConflict:
        ESP_LOGW(TAG, "Transaction blocked: ReservationConflict");
        break;
      default:
        break;
    }
  });
  // Reset: MicroOcpp's internal handler aborts with "No reset handler set" if
  // we don't register an executor, so use setOnResetExecute (not the observer-
  // only setOnReceiveRequest). MO accepts the reset, sends Reset.conf, and
  // then invokes us to actually perform it. The user's automation does the
  // hardware-side reset (sending a command to an MCU, rebooting the chip,
  // etc.) — we just translate the bool to a "Soft"/"Hard" string.
  setOnResetExecute([this](bool is_hard) {
    std::string type = is_hard ? "Hard" : "Soft";
    ESP_LOGI(TAG, "Reset.execute (%s)", type.c_str());
    for (auto &cb : reset_callbacks_) cb(type);
  });
  setOnReceiveRequest("UnlockConnector", [this](JsonObject payload) {
    int connector_id = payload["connectorId"] | 1;
    for (auto &cb : unlock_callbacks_) cb(connector_id);
  });

  // TriggerMessage diagnostic observer. MicroOcpp's built-in
  // Operations/TriggerMessage handler does the actual work — it queues
  // StatusNotification / MeterValues / etc. via takeTriggeredMeterValues
  // and triggerStatusNotification. This observer just gives YAML a hook
  // to log or count CSMS-initiated polls. CSMS test buttons (SteVe's
  // Operations tab, evcc's debug panel) all funnel through here.
  setOnReceiveRequest("TriggerMessage", [this](JsonObject payload) {
    std::string requested = payload["requestedMessage"] | "?";
    int connector_id = payload["connectorId"] | -1;
    ESP_LOGI(TAG, "TriggerMessage: requested=%s connector=%d",
             requested.c_str(), connector_id);
    for (auto &cb : trigger_message_callbacks_) cb(requested, connector_id);
  });

  // DataTransfer custom request handler. setRequestHandler *replaces* MO's
  // built-in DataTransfer (which always returns Rejected); we capture the
  // request, fire the YAML callbacks, and reply Accepted with no data. If
  // there are no DataTransfer callbacks bound, leave MO's stock handler in
  // place — it returns the OCPP-default Rejected, which is the right
  // answer for a CP that doesn't speak any vendor extensions.
  if (!data_transfer_callbacks_.empty()) {
    setRequestHandler("DataTransfer",
        [this](JsonObject request) {
          std::string vendor = request["vendorId"] | "";
          std::string message = request["messageId"] | "";
          std::string data;
          // The data field is OCPP-typed as `string` but vendors often ship
          // structured JSON inside. Serialize back to a string so YAML can
          // parse it (or just log it). serializeJson with the raw variant
          // copes with strings, objects, arrays, numbers identically.
          if (request.containsKey("data")) {
            JsonVariant d = request["data"];
            if (d.is<const char *>()) {
              data = d.as<const char *>();
            } else {
              serializeJson(d, data);
            }
          }
          ESP_LOGI(TAG,
                   "DataTransfer: vendor='%s' message='%s' data='%s'",
                   vendor.c_str(), message.c_str(), data.c_str());
          for (auto &cb : data_transfer_callbacks_) cb(vendor, message, data);
        },
        []() -> std::unique_ptr<MicroOcpp::JsonDoc> {
          // Fixed Accepted reply, no data. Matches "the message was
          // received and processed" semantics — sufficient for the
          // observer/automation use case.
          auto res = std::unique_ptr<MicroOcpp::JsonDoc>(
              new MicroOcpp::JsonDoc(JSON_OBJECT_SIZE(1)));
          JsonObject response = res->to<JsonObject>();
          response["status"] = "Accepted";
          return res;
        });
  }
}

void OcppCp::loop() {
  if (!initialized_) return;
  mocpp_loop();
  poll_status_();
  enforce_heartbeat_interval_();
  enforce_meter_value_sample_interval_();
  enforce_stop_txn_sampled_data_();
  publish_connection_state_();
  refresh_mvsd_();
}

std::string OcppCp::build_mvsd_list_() const {
  // MicroOcpp's Configuration::setString rejects strings longer than
  // MO_CONFIG_MAX_VALSTRSIZE (default 128 chars including null terminator)
  // and silently returns false. Our fork raises the override-via-D ceiling;
  // we set it to 256 in __init__.py so the full 138-char advertised list
  // fits with margin. Keep the length-pack as a defence-in-depth: if a
  // future build disables the override, MO would silently reject the
  // longer string and the CSMS-side default would override ours. 255 chars
  // matches the configured ceiling minus the null terminator.
  static constexpr size_t kMaxLen = 255;
  std::string list;
  auto add_if = [&](bool cond, const char *m) {
    if (!cond) return;
    size_t add = strlen(m) + (list.empty() ? 0 : 1);
    if (list.size() + add > kMaxLen) {
      ESP_LOGW(TAG, "MVSD: dropping %s (would exceed %u-char MO limit)", m,
               (unsigned) kMaxLen);
      return;
    }
    if (!list.empty()) list += ",";
    list += m;
  };
  // Mandatory base — these always fit (102 chars).
  add_if(true, "Energy.Active.Import.Register");
  add_if(true, "Power.Active.Import");
  // The per-row phase tag handles which leg each sample belongs to, so
  // the measurand string is "Voltage" / "Current.Import" once regardless
  // of single- vs 3-phase mode.
  add_if(meter_sensors_.count(MeterValueField::VOLTAGE) > 0 ||
             meter_sensors_phase_.count(MeterValueField::VOLTAGE) > 0,
         "Voltage");
  add_if(meter_sensors_.count(MeterValueField::CURRENT) > 0 ||
             meter_sensors_phase_.count(MeterValueField::CURRENT) > 0,
         "Current.Import");
  // Current/Power.Offered come from the user-bound Number. Drop them while
  // CSMS has imposed a 0-limit profile (see csms_disabled_) — otherwise
  // they'd swing between 0 and the Number value depending on the CSMS state,
  // which is exactly the lookup evcc's Enabled() does.
  add_if(current_offered_number_ != nullptr, "Current.Offered");
  add_if(current_offered_number_ != nullptr &&
             meter_sensors_.count(MeterValueField::VOLTAGE) > 0,
         "Power.Offered");
  // HA-mirrored extras: include only when the source is actually serving
  // fresh data. Three conditions stack:
  //   - has_state() AND !isnan: the homeassistant: platform marks
  //     `unavailable` HA entities with state=NaN but leaves
  //     has_state()=true, so the isnan guard drops e.g. SoC while the
  //     vehicle integration upstream is offline.
  //   - api_ok (when require_api): if the ESPHome→HA WebSocket itself is
  //     down, every HA-mirrored sensor is stale; drop them all from MVSD
  //     until the API reconnects. Local sensors (rippleon's own
  //     system_temp) ignore this gate.
  //   - require_plugged: SoC is only meaningful with an EV connected;
  //     don't report a phantom Rivian SoC while the cable is unplugged
  //     from the charger.
  bool api_ok = api::global_api_server != nullptr &&
                api::global_api_server->is_connected();
  bool plugged = plugged_sensor_ != nullptr && plugged_sensor_->has_state() &&
                 plugged_sensor_->state;
  auto bound_and_ready = [&](MeterValueField f, bool require_api,
                             bool require_plugged) {
    auto it = meter_sensors_.find(f);
    if (it == meter_sensors_.end() || it->second == nullptr) return false;
    auto *s = it->second;
    if (!s->has_state() || std::isnan(s->state)) return false;
    if (require_api && !api_ok) return false;
    if (require_plugged && !plugged) return false;
    return true;
  };
  // Temperature defaults to a local sensor (rippleon's own system_temp);
  // not gated on the HA API. SoC/Frequency/Power.Factor are conventionally
  // HA mirrors — if the user binds a local sensor instead the gate's a
  // false negative but only while HA is offline, which is acceptable.
  add_if(bound_and_ready(MeterValueField::TEMPERATURE, false, false),
         "Temperature");
  // SoC has an extra layer: when soc_plugged_from is bound (e.g. an HA mirror
  // of the Rivian's own `binary_sensor.r1t_charger_connection`), require it
  // to also report true. That way SoC is only emitted when the cable is in
  // the *same* car we're pulling SoC from — a different EV plugged in
  // doesn't get a phantom Rivian SoC reading.
  bool soc_ok = bound_and_ready(MeterValueField::SOC, true, true);
  if (soc_ok && soc_plugged_sensor_ != nullptr) {
    soc_ok = soc_plugged_sensor_->has_state() && soc_plugged_sensor_->state;
  }
  add_if(soc_ok, "SoC");
  add_if(bound_and_ready(MeterValueField::FREQUENCY, true, false),
         "Frequency");
  add_if(bound_and_ready(MeterValueField::POWER_FACTOR, true, false),
         "Power.Factor");
  return list;
}

void OcppCp::refresh_mvsd_() {
  // Debounce: only check every 5 s. has_state() flips are rare and a stale
  // measurand for a few seconds during a vehicle-disconnect transition is
  // harmless.
  uint32_t now = millis();
  if (last_mvsd_check_ms_ != 0 && now - last_mvsd_check_ms_ < 5000) return;
  last_mvsd_check_ms_ = now;
  auto mvsd = MicroOcpp::declareConfiguration<const char *>(
      "MeterValuesSampledData", "");
  if (!mvsd) {
    ESP_LOGD(TAG, "MVSD slot unreachable");
    return;
  }
  std::string desired = build_mvsd_list_();
  // Compare to MO's actual stored value, not our last write — evcc / SteVe
  // can ChangeConfiguration MVSD between our refreshes (e.g. inject SoC),
  // and we need to overwrite that, not skip because our private last_mvsd_
  // shadow says we already set it to the same string.
  const char *cur = mvsd->getString();
  // Always log at DEBUG so we can see whether MO's stored value diverges
  // from what we want, even when no setString happens.
  ESP_LOGD(TAG, "MVSD tick: cur=[%s] desired=[%s]",
           cur ? cur : "(null)", desired.c_str());
  if (cur != nullptr && desired == cur) return;
  mvsd->setString(desired.c_str());
  ESP_LOGI(TAG, "MeterValuesSampledData: %s", desired.c_str());
}

void OcppCp::end_transaction(const std::string &reason) {
  if (!initialized_) return;
  ESP_LOGI(TAG, "end_transaction (reason=%s)", reason.c_str());
  ::endTransaction(nullptr, reason.c_str(), 1);
}

bool OcppCp::start_transaction(const std::string &id_tag) {
  if (!initialized_) return false;
  if (id_tag.empty()) {
    ESP_LOGW(TAG, "start_transaction: empty idTag");
    return false;
  }
  ESP_LOGI(TAG, "start_transaction (idTag=%s)", id_tag.c_str());
  // beginTransaction returns nullptr if another transaction is already
  // active or it refuses for any reason (e.g. connector unavailable).
  auto tx = ::beginTransaction(id_tag.c_str(), 1);
  return tx != nullptr;
}

bool OcppCp::end_transaction_with_idtag(const std::string &id_tag,
                                        const std::string &reason) {
  if (!initialized_) return false;
  ESP_LOGI(TAG, "end_transaction (idTag=%s reason=%s)",
           id_tag.c_str(), reason.c_str());
  return ::endTransaction(id_tag.empty() ? nullptr : id_tag.c_str(),
                          reason.c_str(), 1);
}

void OcppCp::publish_connection_state_() {
  if (connection_state_sensor_ == nullptr || ws_ == nullptr) return;
  const char *s;
  switch (ws_->state()) {
    case WsClient::STATE_DISCONNECTED:
      s = "disconnected";
      break;
    case WsClient::STATE_CONNECTING:
      s = "connecting";
      break;
    case WsClient::STATE_HANDSHAKING:
      s = "handshaking";
      break;
    case WsClient::STATE_OPEN:
      // "ready" implies BootNotification accepted *and* the connector isn't
      // Faulted/Unavailable. ::isOperative(1) covers both — MO's
      // chargePoint->isOperative() flips true on RegistrationStatus::Accepted.
      s = ::isOperative(1) ? "ready" : "connected";
      break;
    case WsClient::STATE_CLOSING:
      s = "closing";
      break;
    default:
      s = "unknown";
      break;
  }
  if (last_connection_state_ != s) {
    last_connection_state_ = s;
    connection_state_sensor_->publish_state(s);
  }
}

void OcppCp::enforce_heartbeat_interval_() {
  // BootNotification.conf overwrites HeartbeatInterval with whatever the CSMS
  // returns (SteVe defaults to 14400 s, evcc to 86400 s). If the user pinned
  // an interval in YAML, force the configuration back every 5 s — covers the
  // post-boot window where MO has just stomped our value.
  if (heartbeat_interval_s_ <= 0) return;
  uint32_t now = millis();
  if (now - last_hb_check_ms_ < 5000) return;
  last_hb_check_ms_ = now;
  auto cfg =
      MicroOcpp::declareConfiguration<int>("HeartbeatInterval", heartbeat_interval_s_);
  if (cfg && cfg->getInt() != heartbeat_interval_s_) {
    cfg->setInt(heartbeat_interval_s_);
    ESP_LOGD(TAG, "HeartbeatInterval forced to %d s", heartbeat_interval_s_);
  }
}

void OcppCp::enforce_stop_txn_sampled_data_() {
  // OCPP 1.6 StopTxnSampledData: measurands shipped only with the
  // StopTransaction.req transcript (separate from the periodic
  // MeterValuesSampledData feed). Re-pin every 5 s like the other config
  // overrides — MO's validateSelectString rejects entries without a
  // registered sampler, so this can't fire before register_callbacks_()
  // has wired all addMeterValueInput calls. Empty user value means
  // "leave MO's default" — no override.
  if (stop_txn_sampled_data_.empty()) return;
  uint32_t now = millis();
  if (now - last_stop_txn_check_ms_ < 5000) return;
  last_stop_txn_check_ms_ = now;
  auto cfg =
      MicroOcpp::declareConfiguration<const char *>("StopTxnSampledData", "");
  if (cfg == nullptr) return;
  const char *cur = cfg->getString();
  if (cur != nullptr && stop_txn_sampled_data_ == cur) return;
  if (!cfg->setString(stop_txn_sampled_data_.c_str())) {
    ESP_LOGW(TAG,
             "StopTxnSampledData rejected (likely an entry without a bound "
             "sensor): %s",
             stop_txn_sampled_data_.c_str());
    return;
  }
  ESP_LOGI(TAG, "StopTxnSampledData: %s", stop_txn_sampled_data_.c_str());
}

void OcppCp::enforce_meter_value_sample_interval_() {
  // MicroOcpp defaults MeterValueSampleInterval to 60 s but exposes it as a
  // standard ChangeConfiguration key — evcc / SteVe / Monta will all rewrite
  // it on the post-boot config push. Same enforce-every-5-s pattern as
  // HeartbeatInterval so the YAML-pinned value always wins after at most one
  // 5-second window.
  if (meter_value_sample_interval_s_ <= 0) return;
  uint32_t now = millis();
  if (now - last_mvsi_check_ms_ < 5000) return;
  last_mvsi_check_ms_ = now;
  auto cfg = MicroOcpp::declareConfiguration<int>(
      "MeterValueSampleInterval", meter_value_sample_interval_s_);
  if (cfg && cfg->getInt() != meter_value_sample_interval_s_) {
    cfg->setInt(meter_value_sample_interval_s_);
    ESP_LOGD(TAG, "MeterValueSampleInterval forced to %d s",
             meter_value_sample_interval_s_);
  }
}

void OcppCp::poll_status_() {
  if (status_sensor_ == nullptr || !status_sensor_->has_state()) return;
  const std::string &cur = status_sensor_->state;
  if (cur == last_source_status_) return;
  last_source_status_ = cur;
  // If a mapping is configured, resolve; otherwise pass through (assumes the
  // source already speaks OCPP vocabulary: Available / Preparing / Charging /
  // SuspendedEV / SuspendedEVSE / Finishing / Reserved / Unavailable / Faulted).
  auto it = status_mapping_.find(cur);
  mapped_status_ = it == status_mapping_.end() ? cur : it->second;
  ESP_LOGD(TAG, "Status change: %s -> %s", cur.c_str(), mapped_status_.c_str());
  // Connector input lambdas read mapped_status_ on next mocpp_loop() and MO
  // emits StatusNotification when the derived connector state crosses a
  // boundary.
}

bool OcppCp::is_plugged_() const {
  // Prefer an explicit plugged binary_sensor when bound — many chargers (e.g.
  // the rippleon) report a "ready"/"idle" status text that doesn't mean an EV
  // is physically connected, so deriving plugged from status alone causes
  // MicroOcpp to spam Preparing → SetChargingProfile → unnecessary MCU writes.
  if (plugged_sensor_ != nullptr) {
    return plugged_sensor_->has_state() && plugged_sensor_->state;
  }
  // Fallback: derive from status. Plugged whenever the connector is past
  // Available and not in a non-charging terminal state.
  return mapped_status_ != "Available" && mapped_status_ != "Reserved" &&
         mapped_status_ != "Unavailable" && mapped_status_ != "Faulted";
}

bool OcppCp::is_ev_ready_() const {
  // EV signals it's drawing (or willing to draw) current. SuspendedEV means
  // the EV explicitly stopped the request, so EvReady=false.
  return mapped_status_ == "Preparing" || mapped_status_ == "Charging" ||
         mapped_status_ == "SuspendedEVSE" || mapped_status_ == "Finishing";
}

bool OcppCp::is_evse_ready_() const {
  // EVSE-side gate. SuspendedEVSE means the station is throttling/blocking
  // (auth, schedule, derate); everything else with a plugged cable is ready.
  return mapped_status_ != "SuspendedEVSE" && mapped_status_ != "Faulted" &&
         mapped_status_ != "Unavailable";
}

void OcppCp::dump_config() {
  ESP_LOGCONFIG(TAG, "OCPP Charge Point:");
  ESP_LOGCONFIG(TAG, "  CSMS URL: %s", csms_url_.c_str());
  ESP_LOGCONFIG(TAG, "  Charge Point ID: %s", cp_id_.c_str());
  ESP_LOGCONFIG(TAG, "  Vendor/Model: %s / %s", vendor_.c_str(),
                model_.c_str());
  if (!firmware_version_.empty())
    ESP_LOGCONFIG(TAG, "  Firmware Version: %s", firmware_version_.c_str());

  ESP_LOGCONFIG(TAG, "  Region:");
  ESP_LOGCONFIG(TAG, "    Nominal Voltage: %.1f V", nominal_voltage_);
  if (!phase_tags_.empty()) {
    std::string tags;
    for (size_t i = 0; i < phase_tags_.size(); ++i) {
      if (i) tags += ",";
      tags += phase_tags_[i];
    }
    ESP_LOGCONFIG(TAG, "    Phases: [%s] (3-phase mode)", tags.c_str());
  } else if (!phase_.empty()) {
    ESP_LOGCONFIG(TAG, "    Phase tag: %s (single-phase)", phase_.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "    Phase tag: <none> (untagged single-phase)");
  }
  ESP_LOGCONFIG(TAG, "    1p/3p switchable: %s",
                phase_switching_supported_ ? "yes" : "no");

  // Meter input report. Show each measurand with its bound source — scalar
  // sensor name, or per-leg list for 3-phase voltage/current.
  ESP_LOGCONFIG(TAG, "  Meter inputs:");
  auto report_axis = [this](MeterValueField f, const char *label) {
    auto pit = meter_sensors_phase_.find(f);
    if (!phase_tags_.empty() && pit != meter_sensors_phase_.end()) {
      for (size_t leg = 0; leg < phase_tags_.size() && leg < 3; ++leg) {
        auto *s = pit->second[leg];
        if (s != nullptr)
          ESP_LOGCONFIG(TAG, "    %s.%s: %s", label,
                        phase_tags_[leg].c_str(), s->get_name().c_str());
      }
      return;
    }
    auto it = meter_sensors_.find(f);
    if (it != meter_sensors_.end() && it->second != nullptr)
      ESP_LOGCONFIG(TAG, "    %s: %s", label, it->second->get_name().c_str());
  };
  report_axis(MeterValueField::VOLTAGE, "Voltage");
  report_axis(MeterValueField::CURRENT, "Current.Import");
  auto report_scalar = [this](MeterValueField f, const char *label) {
    auto it = meter_sensors_.find(f);
    if (it != meter_sensors_.end() && it->second != nullptr)
      ESP_LOGCONFIG(TAG, "    %s: %s", label, it->second->get_name().c_str());
  };
  report_scalar(MeterValueField::POWER, "Power.Active.Import");
  report_scalar(MeterValueField::ENERGY, "Energy.Active.Import.Register");
  report_scalar(MeterValueField::TEMPERATURE, "Temperature");
  report_scalar(MeterValueField::SOC, "SoC");
  report_scalar(MeterValueField::FREQUENCY, "Frequency");
  report_scalar(MeterValueField::POWER_FACTOR, "Power.Factor");
  if (current_offered_number_ != nullptr) {
    ESP_LOGCONFIG(TAG, "    Current.Offered: %s",
                  current_offered_number_->get_name().c_str());
    ESP_LOGCONFIG(TAG, "    Power.Offered: computed (Current.Offered × V)");
    ESP_LOGCONFIG(TAG, "      Lock during transaction: %s",
                  lock_offered_current_during_transaction_ ? "yes" : "no");
  }

  // Connector inputs.
  ESP_LOGCONFIG(TAG, "  Connector inputs:");
  if (status_sensor_ != nullptr)
    ESP_LOGCONFIG(TAG, "    Status text_sensor: %s",
                  status_sensor_->get_name().c_str());
  if (plugged_sensor_ != nullptr)
    ESP_LOGCONFIG(TAG, "    Plugged binary_sensor: %s",
                  plugged_sensor_->get_name().c_str());
  if (soc_plugged_sensor_ != nullptr)
    ESP_LOGCONFIG(TAG, "    SoC-plugged gate: %s",
                  soc_plugged_sensor_->get_name().c_str());
  if (!status_mapping_.empty()) {
    ESP_LOGCONFIG(TAG, "    Status mapping:");
    for (auto &kv : status_mapping_)
      ESP_LOGCONFIG(TAG, "      %s -> %s", kv.first.c_str(),
                    kv.second.c_str());
  }

  // Diagnostic outputs.
  ESP_LOGCONFIG(TAG, "  Outputs:");
  if (connection_state_sensor_ != nullptr)
    ESP_LOGCONFIG(TAG, "    Connection state text_sensor: %s",
                  connection_state_sensor_->get_name().c_str());
  if (heartbeat_interval_s_ > 0)
    ESP_LOGCONFIG(TAG, "    Heartbeat interval: %d s", heartbeat_interval_s_);
  else
    ESP_LOGCONFIG(TAG, "    Heartbeat interval: <CSMS-decided>");
  if (meter_value_sample_interval_s_ > 0)
    ESP_LOGCONFIG(TAG, "    MeterValueSampleInterval: %d s",
                  meter_value_sample_interval_s_);
  if (!stop_txn_sampled_data_.empty())
    ESP_LOGCONFIG(TAG, "    StopTxnSampledData: %s",
                  stop_txn_sampled_data_.c_str());

  // LocalAuthList. MicroOcpp auto-handles SendLocalList /
  // GetLocalListVersion as long as MO_ENABLE_LOCAL_AUTH=1 (default).
  // With MO_USE_FILEAPI=DISABLE_FS the list is in-memory only — survives
  // CSMS-pushed updates at runtime but resets on every boot.
  ESP_LOGCONFIG(TAG, "  Local auth list: in-memory (no FS, no persistence)");

  // Automation hookup counts.
  ESP_LOGCONFIG(TAG, "  Automations bound:");
  ESP_LOGCONFIG(TAG, "    on_remote_start: %u",
                (unsigned) remote_start_callbacks_.size());
  ESP_LOGCONFIG(TAG, "    on_remote_stop: %u",
                (unsigned) remote_stop_callbacks_.size());
  ESP_LOGCONFIG(TAG, "    on_reset: %u", (unsigned) reset_callbacks_.size());
  ESP_LOGCONFIG(TAG, "    on_unlock_connector: %u",
                (unsigned) unlock_callbacks_.size());
  ESP_LOGCONFIG(TAG, "    on_charging_profile_change: %u",
                (unsigned) charging_profile_callbacks_.size());
}

}  // namespace ocpp
}  // namespace esphome
