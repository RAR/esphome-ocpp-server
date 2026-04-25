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

}  // namespace ocpp
}  // namespace esphome
