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

// Fired when MicroOcpp recomputes the connector's effective charging limit —
// either from a SetChargingProfile from CSMS, the expiry of a profile window,
// or a ChangeAvailability. The user's automation is expected to push the
// limit to whatever hardware knob actually controls current (typically a
// number entity bound to the charger's MAX_CURRENT DP).
//
// `current_limit_a` is in Amps; `power_limit_w` in Watts; `n_phases` is the
// number of phases the limit applies to (1 or 3). Negative values mean "no
// limit" — pass through whatever the hardware default is.
class ChargingProfileChangeTrigger : public Trigger<float, float, int> {
 public:
  explicit ChargingProfileChangeTrigger(OcppCp *parent) {
    parent->add_on_charging_profile_change_callback(
        [this](float current_limit_a, float power_limit_w, int n_phases) {
          this->trigger(current_limit_a, power_limit_w, n_phases);
        });
  }
};

// CSMS sent a TriggerMessage. MicroOcpp itself satisfies the request
// (queues the requested StatusNotification / MeterValues / etc. — see
// MO's Operations/TriggerMessage.cpp); this trigger is purely diagnostic
// so YAML can log it or count it. `requested_message` is the exact string
// from the request ("StatusNotification", "MeterValues", ...);
// `connector_id` is -1 when the CSMS didn't scope it.
class TriggerMessageTrigger : public Trigger<std::string, int> {
 public:
  explicit TriggerMessageTrigger(OcppCp *parent) {
    parent->add_on_trigger_message_callback(
        [this](const std::string &requested_message, int connector_id) {
          this->trigger(requested_message, connector_id);
        });
  }
};

// CSMS sent a DataTransfer.req — vendor-specific extension messages.
// `vendor_id` and `message_id` come straight from the request; `data` is
// the JSON-serialized contents of the request's `data` field (or "" if
// absent). The component always replies Accepted with no data; if you
// need richer responses, change the API to take a return value.
class DataTransferTrigger
    : public Trigger<std::string, std::string, std::string> {
 public:
  explicit DataTransferTrigger(OcppCp *parent) {
    parent->add_on_data_transfer_callback(
        [this](const std::string &vendor_id, const std::string &message_id,
               const std::string &data) {
          this->trigger(vendor_id, message_id, data);
        });
  }
};

}  // namespace ocpp
}  // namespace esphome
