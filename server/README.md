# Local SteVe dev CSMS

SteVe is maintained upstream at <https://github.com/steve-community/steve> and
ships its own `Dockerfile` + `docker-compose.yml`. We don't fork it. This
directory just documents the one-line tweak needed to use SteVe as a dev CSMS
for `esphome-ocpp-server` and how to wire firmware against it.

## First-time setup

```bash
# Clone SteVe somewhere convenient (sibling of this repo works):
git clone https://github.com/steve-community/steve.git ~/dev/steve
cd ~/dev/steve

# Auto-register unknown chargers so firmware can connect without
# pre-creating a CP row in the SteVe UI:
sed -i 's/^auto.register.unknown.stations = false/auto.register.unknown.stations = true/' \
    src/main/resources/application-docker.properties

# Build + run (first build pulls Maven and an OpenJDK image; ~5–10 min):
docker compose up --build
```

When ready, Jetty logs:

```
Jetty started on port 8180 (http/1.1) with context path '/steve'
```

Web UI: <http://localhost:8180/steve/manager/home>
Default creds: `admin` / `1234` (from `application-docker.properties`).

## Firmware connection URL

```
ws://<host-ip>:8180/steve/websocket/CentralSystemService/<charge_point_id>
```

Replace `<host-ip>` with the LAN IP the firmware can reach the docker host at
(use `ip -4 addr` on the host — not `localhost`, since the chip is on the LAN
not loopback). `<charge_point_id>` is whatever you set in your firmware's
`ocpp.charge_point_id`. With `auto.register.unknown.stations = true`, the CP
appears under <http://localhost:8180/steve/manager/chargepoints> after the
first BootNotification.

## Tailing OCPP traffic

- SteVe web UI → "Operations" → pick CP → live request/response log.
- `docker compose logs -f app` for raw container output.

## Tearing down

```bash
docker compose down -v   # -v also wipes the mariadb volume
```
