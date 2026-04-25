#pragma once

#include "esphome/core/automation.h"
#include "ocpp.h"

namespace esphome {
namespace ocpp {

// Fired when the CSMS asks the CP to start a transaction (RemoteStartTransaction).
// The id_tag the CSMS provided is passed to the automation as a string.
class RemoteStartTrigger : public Trigger<std::string> {
 public:
  explicit RemoteStartTrigger(OcppCp *parent) {
    parent->add_on_remote_start_callback([this](const std::string &id_tag) { this->trigger(id_tag); });
  }
};

class RemoteStopTrigger : public Trigger<int> {
 public:
  explicit RemoteStopTrigger(OcppCp *parent) {
    parent->add_on_remote_stop_callback([this](int transaction_id) { this->trigger(transaction_id); });
  }
};

class ResetTrigger : public Trigger<std::string> {
 public:
  explicit ResetTrigger(OcppCp *parent) {
    parent->add_on_reset_callback([this](const std::string &type) { this->trigger(type); });
  }
};

class UnlockConnectorTrigger : public Trigger<int> {
 public:
  explicit UnlockConnectorTrigger(OcppCp *parent) {
    parent->add_on_unlock_callback([this](int connector_id) { this->trigger(connector_id); });
  }
};

}  // namespace ocpp
}  // namespace esphome
