#include "ocpp.h"

#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"

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

  // MO's MeteringService default for MeterValuesSampledData is just
  // "Energy.Active.Import.Register,Power.Active.Import" — voltage and current
  // are dropped from MeterValues even though we feed them in through
  // addMeterValueInput(). Force the full set so evcc / other CSMSes see V/I/P/E
  // per sample without having to ChangeConfiguration on every connect.
  // declareConfiguration is idempotent (returns the existing slot if MO has
  // already declared it earlier in init), so we setString() to actually move
  // the runtime value off MO's stingier default.
  if (auto mvsd = MicroOcpp::declareConfiguration<const char *>(
          "MeterValuesSampledData",
          "Energy.Active.Import.Register,Power.Active.Import,Voltage,Current.Import")) {
    mvsd->setString(
        "Energy.Active.Import.Register,Power.Active.Import,Voltage,Current.Import");
  }

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

  auto volt_it = meter_sensors_.find(MeterValueField::VOLTAGE);
  if (volt_it != meter_sensors_.end() && volt_it->second != nullptr) {
    auto *s = volt_it->second;
    addMeterValueInput([s]() -> float { return s->has_state() ? s->state : 0.0f; },
                       "Voltage", "V");
  }

  auto curr_it = meter_sensors_.find(MeterValueField::CURRENT);
  if (curr_it != meter_sensors_.end() && curr_it->second != nullptr) {
    auto *s = curr_it->second;
    addMeterValueInput([s]() -> float { return s->has_state() ? s->state : 0.0f; },
                       "Current.Import", "A");
  }

  // Current.Offered / Power.Offered. evcc requests both when configured with
  // remotestart + currentLimit so it can reflect the EVSE-offered cap in its
  // UI. Without registering them, MO emits "could not find metering device"
  // warnings on every ChangeConfiguration. Bind a number::Number that holds
  // the offered current (e.g. a Max Charge Current entity) and we publish
  // both Current.Offered = number.state and (if voltage is bound) the
  // computed Power.Offered = current_offered × voltage_rms.
  if (current_offered_number_ != nullptr) {
    auto *n = current_offered_number_;
    addMeterValueInput(
        [this, n]() -> float {
          // CSMS-imposed pause overrides the user's Number — must report 0 so
          // evcc's Enabled()-via-Current.Offered fallback agrees with its
          // own `Enable(false)` call.
          if (csms_limit_a_ == 0.0f || csms_limit_w_ == 0.0f) return 0.0f;
          return std::isnan(n->state) ? 0.0f : n->state;
        },
        "Current.Offered", "A");
    auto volt_it2 = meter_sensors_.find(MeterValueField::VOLTAGE);
    if (volt_it2 != meter_sensors_.end() && volt_it2->second != nullptr) {
      auto *v = volt_it2->second;
      addMeterValueInput(
          [this, n, v]() -> float {
            if (csms_limit_a_ == 0.0f || csms_limit_w_ == 0.0f) return 0.0f;
            float i = std::isnan(n->state) ? 0.0f : n->state;
            float vv = v->has_state() ? v->state : 0.0f;
            return i * vv;
          },
          "Power.Offered", "W");
    }
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
  setOnReceiveRequest("SetChargingProfile", [](JsonObject payload) {
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
    for (JsonObject p : periods) {
      int start = p["startPeriod"] | 0;
      float limit = p["limit"] | 0.0f;
      int phases = p["numberPhases"] | -1;
      if (phases >= 0) {
        ESP_LOGI(TAG, "  period[%u] start=+%ds limit=%.1f%s phases=%d",
                 (unsigned) i, start, limit, unit, phases);
      } else {
        ESP_LOGI(TAG, "  period[%u] start=+%ds limit=%.1f%s", (unsigned) i,
                 start, limit, unit);
      }
      i++;
    }
  });

  // Same diagnostic pattern for ClearChargingProfile so a "stop charging"
  // edge from the CSMS is visible whether it comes via RemoteStop or via
  // profile-level pause.
  setOnReceiveRequest("ClearChargingProfile", [](JsonObject payload) {
    int profile_id = payload["id"] | -1;
    int connector_id = payload["connectorId"] | -1;
    const char *purpose = payload["chargingProfilePurpose"] | "?";
    int stack_level = payload["stackLevel"] | -1;
    ESP_LOGI(TAG,
             "ClearChargingProfile: id=%d connector=%d purpose=%s stack=%d",
             profile_id, connector_id, purpose, stack_level);
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
        for (auto &cb : remote_start_callbacks_) cb(std::string(id_tag));
        break;
      case N::StopTx:
        ESP_LOGI(TAG, "StopTransaction triggered: tx=%d", tx_id);
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

  // SmartCharging — MO computes an effective limit from the active stack of
  // ChargingProfiles and invokes this callback whenever the value changes.
  // current_limit_A == -1.0f means "no limit" (pass through hardware default).
  setSmartChargingCurrentOutput([this](float current_limit_a) {
    ESP_LOGD(TAG, "SmartCharging current limit: %.1f A", current_limit_a);
    csms_limit_a_ = current_limit_a;
    csms_limit_w_ = -1.0f;
    for (auto &cb : charging_profile_callbacks_) cb(current_limit_a, -1.0f, -1);
  });
  setSmartChargingPowerOutput([this](float power_limit_w) {
    ESP_LOGD(TAG, "SmartCharging power limit: %.1f W", power_limit_w);
    csms_limit_w_ = power_limit_w;
    csms_limit_a_ = -1.0f;
    for (auto &cb : charging_profile_callbacks_) cb(-1.0f, power_limit_w, -1);
  });
}

void OcppCp::loop() {
  if (!initialized_) return;
  mocpp_loop();
  poll_status_();
  enforce_heartbeat_interval_();
  publish_connection_state_();
}

void OcppCp::end_transaction(const std::string &reason) {
  if (!initialized_) return;
  ESP_LOGI(TAG, "end_transaction (reason=%s)", reason.c_str());
  ::endTransaction(nullptr, reason.c_str(), 1);
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
  ESP_LOGCONFIG(TAG, "  Vendor/Model: %s / %s", vendor_.c_str(), model_.c_str());
  if (!firmware_version_.empty())
    ESP_LOGCONFIG(TAG, "  Firmware Version: %s", firmware_version_.c_str());
  ESP_LOGCONFIG(TAG, "  Meter inputs: %s%s%s%s",
                meter_sensors_.count(MeterValueField::VOLTAGE) ? "V " : "",
                meter_sensors_.count(MeterValueField::CURRENT) ? "I " : "",
                meter_sensors_.count(MeterValueField::POWER) ? "P " : "",
                meter_sensors_.count(MeterValueField::ENERGY) ? "E" : "");
  if (status_sensor_ != nullptr)
    ESP_LOGCONFIG(TAG, "  Status source: bound");
}

}  // namespace ocpp
}  // namespace esphome
