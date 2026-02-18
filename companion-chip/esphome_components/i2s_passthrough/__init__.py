"""
ESPHome component registration for i2s_passthrough.

Declares the I2SPassthrough C++ class so that ESPHome's code-generation
pipeline can instantiate it from the YAML configuration.  The component
has no configurable parameters beyond the standard component ID.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@local"]
AUTO_LOAD = []
MULTI_CONF = False

i2s_passthrough_ns = cg.esphome_ns.namespace("i2s_passthrough")
I2SPassthrough = i2s_passthrough_ns.class_("I2SPassthrough", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(I2SPassthrough),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)