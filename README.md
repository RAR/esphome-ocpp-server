# esphome-ocpp-server

On-device **OCPP 1.6J Charge Point** external component for [ESPHome](https://esphome.io),
built on top of [matth-x/MicroOcpp](https://github.com/matth-x/MicroOcpp).

Lets any ESPHome-controlled EV charger register with a Central System (CSMS) —
SteVe, CitrineOS, Monta, ev.energy, etc. — and expose itself as a standard
OCPP Charge Point over WebSocket. No always-on Home Assistant needed.

## Status

Early development. First target: LibreTiny `bk72xx` (BK7231N) against a local
SteVe instance. Should also compile on ESP32/ESP8266 since MicroOcpp supports
those out of the box; untested.

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
    components: [ocpp]

ocpp:
  id: ocpp_cp
  csms_url: ws://192.168.1.10:8180/steve/websocket/CentralSystemService/my-charger
  charge_point_id: my-charger
  vendor: My Company
  model: My Charger Model
  meter_values:
    voltage: voltage_a_sensor
    current: current_a_sensor
    power: power_sensor
    energy: session_energy_sensor
  status_from: status_text_sensor
  on_remote_start:
    - lambda: id(my_charger)->start();
  on_remote_stop:
    - lambda: id(my_charger)->stop();
```

See [`example/`](example/) for complete configs, including the reference
Rippleon ROC001 integration.

## License

GPLv3. Inherited from [MicroOcpp](https://github.com/matth-x/MicroOcpp/blob/main/LICENSE).
If you distribute firmware built against this component, you must offer source.
