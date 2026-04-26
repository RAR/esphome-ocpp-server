# esphome-ocpp-server

On-device **OCPP 1.6J Charge Point** external component for [ESPHome](https://esphome.io),
built on top of [matth-x/MicroOcpp](https://github.com/matth-x/MicroOcpp).

Lets any ESPHome-controlled EV charger register with a Central System (CSMS) —
SteVe, CitrineOS, evcc, Monta, ev.energy, etc. — and expose itself as a standard
OCPP Charge Point over WebSocket. No always-on Home Assistant needed for the
OCPP link itself.

## Status

In production use against [evcc](https://evcc.io) on a Quectel FC41D
(BK7231N) inside a reverse-engineered Rippleon ROC001 EV charger. Compiles
under LibreTiny (`bk72xx`); should compile on ESP32/ESP8266 since MicroOcpp
supports those upstream.

## Repository layout

| Path | Contents |
|---|---|
| `components/ocpp/` | The ESPHome external component. Import via `external_components:`. |
| `server/` | Docker-compose kit for a local SteVe dev CSMS. |
| `example/` | Reference YAML configs. |

## Quick start (dev CSMS)

```bash
cd server
docker-compose up
```

SteVe web UI: <http://localhost:8180/steve/manager/home>. The `init.sql` seeds
a Charge Point identifier `rippleon-charger-01` so you can point firmware at
`ws://<docker-host>:8180/steve/websocket/CentralSystemService/rippleon-charger-01`
without first registering it in the UI.

## Using the component

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/RAR/esphome-ocpp-server
      ref: main
    refresh: always
    components: [ocpp]

ocpp:
  id: ocpp_cp
  csms_url: ws://192.168.1.10:8887/my-charger
  charge_point_id: my-charger
  vendor: My Company
  model: My Charger Model
  firmware_version: "1.0.0"
  phase: L1                          # OCPP phase tag for V/I (single-phase: L1)
  nominal_voltage: 240.0             # 120 US outlet / 230 EU 1ϕ / 240 US split / 400 EU 3ϕ L-L
  phase_switching_supported: false   # true only for EU 3-phase 1p3p switchable EVSEs
  lock_offered_current_during_transaction: false
                                     # true on EVSEs whose hardware can't change
                                     # offered current mid-session — pins the
                                     # Current.Offered measurand to its StartTx
                                     # value so evcc stops logging "current
                                     # mismatch" on attempted mid-session derates
  # 3-phase EVSEs swap the `phase:` scalar above for a `phases:` list and
  # use {l1,l2,l3} dicts under voltage / current in meter_values:
  #   phases: [L1, L2, L3]
  #   meter_values:
  #     voltage: {l1: v_l1, l2: v_l2, l3: v_l3}
  #     current: {l1: i_l1, l2: i_l2, l3: i_l3}
  #     power: total_power
  #     energy: total_energy
  heartbeat_interval: 60s            # pin Heartbeat (CSMSes often default to hours)
  meter_value_sample_interval: 30s   # pin MeterValueSampleInterval (≥30s for evcc)
  meter_values:
    voltage: voltage_a_sensor        # → Voltage (V)
    current: current_a_sensor        # → Current.Import (A)
    power: power_sensor              # → Power.Active.Import (kW input → W)
    energy: charge_energy_sensor     # → Energy.Active.Import.Register (kWh input → Wh)
    current_offered: max_current_n   # number::Number → Current.Offered (A)
                                     #   Power.Offered auto-computed from Voltage*Number
    temperature: system_temp_sensor  # → Temperature, Location=Body
    soc: ev_soc_sensor               # → SoC, Location=EV (HA-mirrored sensor)
    frequency: grid_freq_sensor      # → Frequency (Hz, HA-mirrored)
    power_factor: grid_pf_sensor     # → Power.Factor (HA-mirrored)
  status_from: status_text_sensor
  status_mapping:                    # local-state → OCPP-status
    "Not Connected": Available
    "Ready to Charge": Preparing
    "Charging": Charging
    "Stopping": Finishing
    "Complete": Finishing
    "Fault": Faulted
  plugged_from: plugged_binary       # authoritative cable-plug signal
  soc_plugged_from: ev_connected_bs  # EV-identity gate for SoC (optional)
  on_remote_start:                   # fires on real StartTx (not just request)
    - lambda: id(my_charger)->start();
  on_remote_stop:                    # fires on real StopTx
    - lambda: id(my_charger)->stop();
  on_reset:
    - lambda: id(my_charger)->reset();
  on_charging_profile_change:        # SmartCharging effective limit
    - lambda: |-
        if (current_limit_a == 0.0f || power_limit_w == 0.0f) {
          id(my_charger)->stop();
        } else if (current_limit_a > 0.0f) {
          // forward to charger's max-current control
        }
  on_trigger_message:                # CSMS-initiated diagnostic poll
    - lambda: |-
        ESP_LOGI("evcc", "TriggerMessage: %s connector=%d",
                 requested_message.c_str(), connector_id);
  on_data_transfer:                  # CSMS-side vendor extension messages
    - lambda: |-
        ESP_LOGI("vendor", "DataTransfer: %s/%s data=%s",
                 vendor_id.c_str(), message_id.c_str(), data.c_str());

text_sensor:
  - platform: ocpp
    ocpp_id: ocpp_cp
    connection_state:                # disconnected / connecting / handshaking
      name: "OCPP State"             # / connected / ready / closing
```

See [`example/rippleon.yaml`](example/rippleon.yaml) for a complete
production config including HA-mirrored measurands and SmartCharging
hookup.

## What it does

**Connector / Transaction lifecycle.** Drives MicroOcpp's connector
state machine from the bound `plugged` binary sensor and a status text
sensor. `on_remote_start` / `on_remote_stop` triggers fire on real
`StartTransaction` / `StopTransaction` events (not just request
receipt), so they only run after MO has accepted the request and the
connector is actually entering/leaving a transaction.

**SmartCharging.** `on_charging_profile_change` fires whenever the
CSMS-effective current/power limit changes. The component also detects
profile-level disable (`SetChargingProfile{limit:0}`) at the
request-parse layer and force-zeroes `Current.Offered` /
`Power.Offered` until the CSMS lifts the disable — which is what evcc
keys its `Enabled()` check off when status isn't `Charging` /
`SuspendedEVSE`.

**Dynamic `MeterValuesSampledData`.** The advertised measurand list is
rebuilt every 5 s from currently-bound and currently-fresh sensors.
HA-mirrored measurands drop out of MeterValues automatically when the
underlying entity goes `unavailable`, when the ESPHome→HA API link is
down, or (for `SoC`) when the EV-identity gate fails. Reasserts over
CSMS-side `ChangeConfiguration` writes.

**Heartbeat enforcement.** Pinning `heartbeat_interval` re-applies the
configured value every 5 s — covers the post-`BootNotification` window
where some CSMSes (SteVe, evcc) reset HeartbeatInterval to their own
default.

**WebSocket keepalive.** Custom WS client with 20 s ping cadence and a
60 s pong watchdog. Reconnects 5 s after any drop.

## License

GPLv3. Inherited from [MicroOcpp](https://github.com/matth-x/MicroOcpp/blob/main/LICENSE).
If you distribute firmware built against this component, you must offer source.
