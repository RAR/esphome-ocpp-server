#include "ocpp.h"

#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <MicroOcpp.h>
#include <MicroOcpp/Core/Configuration.h>

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
  MicroOcpp::declareConfiguration<int>("WebSocketPingInterval", 20);

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

  // Automation hooks — wired to the `on_remote_start` / `on_remote_stop` / `on_reset`
  // / `on_unlock_connector` triggers in the YAML. These run as observers; MO still
  // handles the protocol-level response and transaction begin/end internally, so
  // the user's lambda is purely for hardware-side actions (relay, restart).
  setOnReceiveRequest("RemoteStartTransaction", [this](JsonObject payload) {
    std::string id_tag = payload["idTag"] | "";
    for (auto &cb : remote_start_callbacks_) cb(id_tag);
  });
  setOnReceiveRequest("RemoteStopTransaction", [this](JsonObject payload) {
    int tx_id = payload["transactionId"] | 0;
    for (auto &cb : remote_stop_callbacks_) cb(tx_id);
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
    for (auto &cb : charging_profile_callbacks_) cb(current_limit_a, -1.0f, -1);
  });
  setSmartChargingPowerOutput([this](float power_limit_w) {
    ESP_LOGD(TAG, "SmartCharging power limit: %.1f W", power_limit_w);
    for (auto &cb : charging_profile_callbacks_) cb(-1.0f, power_limit_w, -1);
  });
}

void OcppCp::loop() {
  if (!initialized_) return;
  mocpp_loop();
  poll_status_();
  enforce_heartbeat_interval_();
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
