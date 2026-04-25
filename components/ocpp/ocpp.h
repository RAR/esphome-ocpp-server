#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace esphome {

namespace sensor { class Sensor; }
namespace text_sensor { class TextSensor; }

namespace ocpp {

class WsClient;
class MoConnection;

enum class MeterValueField : uint8_t {
  VOLTAGE,
  CURRENT,
  POWER,
  ENERGY,
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

  void set_meter_value_sensor(MeterValueField f, sensor::Sensor *s) { meter_sensors_[f] = s; }
  void set_status_text_sensor(text_sensor::TextSensor *s) { status_sensor_ = s; }
  void add_status_mapping(const std::string &from, const std::string &to) { status_mapping_[from] = to; }

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

 protected:
  void init_microocpp_();
  void register_callbacks_();
  void poll_status_();

  std::string csms_url_;
  std::string cp_id_;
  std::string vendor_;
  std::string model_;
  std::string firmware_version_;

  std::map<MeterValueField, sensor::Sensor *> meter_sensors_;
  text_sensor::TextSensor *status_sensor_{nullptr};
  std::map<std::string, std::string> status_mapping_;
  std::string last_status_;

  std::vector<std::function<void(const std::string &)>> remote_start_callbacks_;
  std::vector<std::function<void(int)>> remote_stop_callbacks_;
  std::vector<std::function<void(const std::string &)>> reset_callbacks_;
  std::vector<std::function<void(int)>> unlock_callbacks_;

  WsClient *ws_{nullptr};
  MoConnection *mo_conn_{nullptr};
  bool initialized_{false};
};

}  // namespace ocpp
}  // namespace esphome
