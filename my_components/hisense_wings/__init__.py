"""Hisense Wings AC component for AEH-W4G2 modules on RTL8710BN."""

import esphome.codegen as cg
from esphome.components import uart, climate

CODEOWNERS = ["@you"]

hisense_wings_ns = cg.esphome_ns.namespace("hisense_wings")
HisenseWings = hisense_wings_ns.class_(
    "HisenseWings", cg.Component, uart.UARTDevice, climate.Climate
)
