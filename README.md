# MySensors RS485 Gateway — ESPHome Component

An ESPHome external component that bridges a [MySensors](https://www.mysensors.org/) RS485 bus to a TCP socket (port 5003 by default), allowing Home Assistant or any MySensors controller to communicate with RS485 nodes.

---

## Hardware

### Required wiring

| ESP pin | RS485 transceiver |
|---------|------------------|
| UART TX | DI (Driver Input) |
| UART RX | RO (Receiver Output) |
| GPIO (any) | DE + /RE tied together (flow control) |
| 3.3 V / GND | VCC / GND |

Recommended transceivers: **MAX485**, **SP3485**, **SN75176**.

> ⚠️ DE and /RE must be tied together and driven by a single GPIO. The component will assert this pin HIGH before transmitting and LOW immediately after, returning the transceiver to receive mode.

---

## Installation

Add the component as an ESPHome [external component](https://esphome.io/components/external_components):

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/JochenFriedrich/mysensors-rs485
      ref: master
```

---

## Minimal YAML example

```yaml
esphome:
  name: mysensors-gw

esp32:
  board: esp32dev
  framework:
    type: arduino

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

logger:

api:

ota:
  password: !secret ota_password

# ── SNTP for accurate time sync to nodes ─────────────────────────────────────
time:
  - platform: sntp
    id: sntp_time
    timezone: Europe/Berlin

# ── UART connected to RS485 transceiver ──────────────────────────────────────
uart:
  id: rs485_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 115200
  # RS485 framing: 8N1 (ESPHome default – no need to specify explicitly)

# ── MySensors RS485 gateway ───────────────────────────────────────────────────
mysensors_rs485:
  uart_id: rs485_uart
  port: 5003                     # TCP port the controller connects to
  flow_control_pin:
    number: GPIO4                # DE / /RE pin of the RS485 transceiver
    mode: OUTPUT
```

---

## Full YAML example (with recommended extras)

```yaml
esphome:
  name: mysensors-gw
  friendly_name: MySensors RS485 Gateway

esp32:
  board: esp32dev
  framework:
    type: arduino

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  ap:
    ssid: "MySensors GW Fallback"
    password: !secret ap_password

captive_portal:

logger:
  level: DEBUG

api:
  encryption:
    key: !secret ha_api_key

ota:
  password: !secret ota_password

time:
  - platform: sntp
    id: sntp_time
    timezone: Europe/Berlin
    servers:
      - 0.pool.ntp.org
      - 1.pool.ntp.org

uart:
  id: rs485_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 115200

mysensors_rs485:
  uart_id: rs485_uart
  port: 5003
  flow_control_pin:
    number: GPIO4
    mode: OUTPUT
```

---

## Home Assistant configuration

In your Home Assistant `configuration.yaml`, add:

```yaml
mysensors:
  gateways:
    - device: tcp://192.168.1.XXX:5003
      persistence_file: mysensors.json
      version: "2.3"
  optimistic: false
  retain: false
```

Replace `192.168.1.XXX` with the IP address of your ESP device.

---

## Protocol details

The component implements the **MySensors v2 serial protocol** over RS485 using ICSC framing:

```
SOH SOH <dst> <src> <cmd=0x58> <len> STX <payload[len]> ETX <crc> EOT
```

The TCP side uses the standard MySensors ASCII serial protocol:

```
node-id;child-sensor-id;message-type;ack;sub-type;payload\n
```

### Gateway-handled internal messages

The following MySensors internal messages are handled **locally** by the gateway and do not require a controller to be connected:

| Sub-type | Description |
|----------|-------------|
| `I_FIND_PARENT_REQUEST` | Replies with distance=0 (gateway is the parent) |
| `I_PING` | Replies with `I_PONG`, echoing the hop counter |
| `I_ID_REQUEST` | Assigns the next free node ID (1–254) and broadcasts `I_ID_RESPONSE` |
| `I_TIME` | Replies with current epoch time (requires SNTP for accuracy) |
| `I_REGISTRATION_REQUEST` | Replies with `I_REGISTRATION_RESPONSE` (allowed=1) |
| `I_VERSION` (from controller) | Replies with gateway firmware version string locally |

All other messages (including `I_TIME` requests) are also forwarded to the TCP controller.

---

## Configuration options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `uart_id` | ID | — | **Required.** The UART component connected to the RS485 transceiver. |
| `port` | int | `5003` | TCP port to listen on. |
| `flow_control_pin` | pin | — | GPIO driving DE+/RE of the RS485 transceiver. Omit if your hardware uses auto-direction control. |

---

## Troubleshooting

**Nodes can't find the gateway**
- Check wiring: A/B polarity on the RS485 bus.
- Verify `baud_rate` matches your nodes (common: 9600, 57600, 115200).
- Enable `logger: level: VERBOSE` to see every received byte.

**Home Assistant shows nodes as unavailable**
- Ensure the TCP port (default 5003) is reachable from HA (no firewall blocking it).
- Check the HA log for MySensors gateway connection errors.

**Garbled messages / checksum errors**
- Add a 120 Ω termination resistor between A and B at both ends of the bus.
- Reduce baud rate if the cable is long (>10 m).

**Nodes don't get a time response**
- Add an SNTP `time:` component to your ESPHome config. Without it the gateway falls back to `millis()/1000` (time since boot), which will cause incorrect timestamps on nodes.

---

## Changes vs original

- **Single-byte drain fixed**: all available UART bytes are consumed per `loop()` call.
- **TCP split-read fixed**: proper line buffer handles fragmented TCP packets.
- **Payload encoding fixed**: `encode()` no longer drops the payload field; correct length is written into the VSL byte.
- **Float decode fixed**: uses `memcpy` instead of pointer cast to avoid UB and misaligned access.
- **Receive timeout**: stale partial packets are discarded after 100 ms.
- **Transmit delay**: 2 µs hold after asserting DE before releasing to RX.
- **Node ID assignment**: gateway now handles `I_ID_REQUEST` and maintains a bitmap of assigned IDs.
- **Internal protocol**: `I_FIND_PARENT_REQUEST`, `I_PING`, `I_REGISTRATION_REQUEST`, and `I_TIME` are now handled locally.
- **Version string**: defined as constants, not a hardcoded string literal.
- **All magic numbers**: replaced with named constants from the MySensors v2 spec.
- **`__init__.py`**: framework version guard added; class name capitalised to `MySensorsGW`.
