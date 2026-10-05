"""Switch platform for hisense_wings — exposes feature toggles as ESPHome switches."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from . import hisense_wings_ns, HisenseWings

CONF_HISENSE_WINGS_ID = "hisense_wings_id"

FEATURES = ["display", "boost", "eco", "quiet", "sleep", "mute_beep"]

FeatureSwitch = hisense_wings_ns.class_(
    "FeatureSwitch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_HISENSE_WINGS_ID): cv.use_id(HisenseWings),
    }
).extend(
    {
        cv.Optional(feat): switch.switch_schema(FeatureSwitch).extend(
            cv.COMPONENT_SCHEMA
        )
        for feat in FEATURES
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_HISENSE_WINGS_ID])
    for feat in FEATURES:
        if feat not in config:
            continue
        var = await switch.new_switch(config[feat])
        await cg.register_component(var, config[feat])
        cg.add(var.set_parent(parent))
        cg.add(var.set_feature_id(FEATURES.index(feat)))
