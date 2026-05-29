import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.cpp_helpers import gpio_pin_expression
from esphome.const import (
    CONF_ID,
    CONF_PORT,
    CONF_FLOW_CONTROL_PIN,
)
from esphome import pins

AUTO_LOAD = ["socket"]
DEPENDENCIES = ["uart", "network"]
MULTI_CONF = True

mysensors_ns = cg.esphome_ns.namespace("mysensorsgw")
MySensorsGW = mysensors_ns.class_("MySensorsGW", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.All(
    cv.require_esphome_version(2023, 4, 0),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MySensorsGW),
            cv.Optional(CONF_PORT, default=5003): cv.port,
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    # RS485 framing requires 8 data bits, no parity, 1 stop bit
    cv.require_framework_version(
        esp_idf=cv.Version(4, 0, 0),
        esp8266_arduino=cv.Version(3, 0, 0),
        esp32_arduino=cv.Version(2, 0, 0),
        rp2040_arduino=cv.Version(1, 0, 0),
        host=cv.Version(0, 0, 0),
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    cg.add(var.set_port(config[CONF_PORT]))

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))
