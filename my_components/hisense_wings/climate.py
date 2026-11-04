"""Climate platform for hisense_wings."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart, climate, sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

from . import hisense_wings_ns, HisenseWings

AUTO_LOAD = ["sensor"]

CONF_FLOW_CONTROL_PIN = "flow_control_pin"
CONF_INDOOR_TEMPERATURE = "indoor_temperature"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_INDOOR_HUMIDITY = "indoor_humidity"
CONF_INDOOR_PIPE_TEMPERATURE = "indoor_pipe_temperature"

if hasattr(climate, "climate_schema"):
    _BASE = climate.climate_schema(HisenseWings)
else:
    _BASE = climate.CLIMATE_SCHEMA.extend(
        {cv.GenerateID(): cv.declare_id(HisenseWings)}
    )

CONFIG_SCHEMA = _BASE.extend(
    {
        cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_INDOOR_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_INDOOR_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_INDOOR_PIPE_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await climate.register_climate(var, config)

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))

    for key, setter in (
        (CONF_INDOOR_TEMPERATURE, "set_indoor_temperature_sensor"),
        (CONF_OUTDOOR_TEMPERATURE, "set_outdoor_temperature_sensor"),
        (CONF_INDOOR_HUMIDITY, "set_indoor_humidity_sensor"),
        (CONF_INDOOR_PIPE_TEMPERATURE, "set_indoor_pipe_temperature_sensor"),
    ):
        if key in config:
            s = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(s))
