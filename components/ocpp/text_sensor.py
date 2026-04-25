"""Output text_sensors emitted by the OCPP component.

Currently exposes:

  text_sensor:
    - platform: ocpp
      ocpp_id: ocpp_cp
      connection_state:
        name: "OCPP Connection State"

`connection_state` is one of: disconnected | connecting | handshaking |
connected | ready | closing. "ready" means WebSocket open AND
BootNotification accepted AND the connector is operative; "connected"
means the WS is up but boot is still pending or the connector is
Faulted/Unavailable.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import OcppCp, ocpp_ns

CONF_OCPP_ID = "ocpp_id"
CONF_CONNECTION_STATE = "connection_state"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_OCPP_ID): cv.use_id(OcppCp),
        cv.Optional(CONF_CONNECTION_STATE): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_OCPP_ID])
    if CONF_CONNECTION_STATE in config:
        ts = await text_sensor.new_text_sensor(config[CONF_CONNECTION_STATE])
        cg.add(parent.set_connection_state_text_sensor(ts))
