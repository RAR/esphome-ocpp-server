#include "ocpp.h"

#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <MicroOcpp.h>

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

  // Automation hooks — wired to the `on_remote_start` / `on_remote_stop` / `on_reset`
  // / `on_unlock_connector` triggers in the YAML. Users implement the actual hardware
  // call in their automation body.
  setOnReceiveRequest("RemoteStartTransaction", [this](JsonObject payload) {
    std::string id_tag = payload["idTag"] | "";
    for (auto &cb : remote_start_callbacks_) cb(id_tag);
  });
  setOnReceiveRequest("RemoteStopTransaction", [this](JsonObject payload) {
    int tx_id = payload["transactionId"] | 0;
    for (auto &cb : remote_stop_callbacks_) cb(tx_id);
  });
  setOnReceiveRequest("Reset", [this](JsonObject payload) {
    std::string type = payload["type"] | "Soft";
    for (auto &cb : reset_callbacks_) cb(type);
  });
  setOnReceiveRequest("UnlockConnector", [this](JsonObject payload) {
    int connector_id = payload["connectorId"] | 1;
    for (auto &cb : unlock_callbacks_) cb(connector_id);
  });
}

void OcppCp::loop() {
  if (!initialized_) return;
  mocpp_loop();
  poll_status_();
}

void OcppCp::poll_status_() {
  if (status_sensor_ == nullptr || !status_sensor_->has_state()) return;
  const std::string &cur = status_sensor_->state;
  if (cur == last_status_) return;
  last_status_ = cur;
  // Mapping: source status text -> OCPP status (handled in user YAML by choosing
  // the already-OCPP-compliant vocabulary: Available / Preparing / Charging /
  // SuspendedEV / SuspendedEVSE / Finishing / Reserved / Unavailable / Faulted).
  // If a mapping is configured, resolve; otherwise pass through.
  auto it = status_mapping_.find(cur);
  const std::string &mapped = it == status_mapping_.end() ? cur : it->second;
  ESP_LOGD(TAG, "Status change: %s -> %s", cur.c_str(), mapped.c_str());
  // MicroOcpp auto-publishes StatusNotification from connector state; for now
  // this is just logging. TODO: drive MicroOcpp's connector inputs so
  // transitions fire StatusNotification.
  (void) mapped;
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
