#pragma once

#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/uart/uart.h"

#include <memory>
#include <string>
#include <vector>

namespace esphome {
namespace mysensorsgw {

// ── RS485 framing constants ────────────────────────────────────────────────
static const uint8_t ICSC_SOH = 0x01;
static const uint8_t ICSC_STX = 0x02;
static const uint8_t ICSC_ETX = 0x03;
static const uint8_t ICSC_EOT = 0x04;
static const uint8_t ICSC_SYS_PACK = 0x58;

// ── MySensors network addresses ───────────────────────────────────────────
static const uint8_t GATEWAY_ADDRESS   = 0x00;
static const uint8_t BROADCAST_ADDRESS = 0xFF;

// ── MySensors v2 protocol constants ───────────────────────────────────────
static const uint8_t MY_MYSENSORS_VERSION_STR_MAJOR = 2;
static const uint8_t MY_MYSENSORS_VERSION_STR_MINOR = 4;
static const uint8_t MY_MYSENSORS_VERSION_STR_PATCH = 0;

static const uint8_t  V2_MYS_HEADER_SIZE             = 7u;
static const uint8_t  V2_MYS_HEADER_MAX_MESSAGE_SIZE = 32u;
static const uint8_t  MAX_MESSAGE_SIZE  = V2_MYS_HEADER_MAX_MESSAGE_SIZE;
static const uint8_t  HEADER_SIZE       = V2_MYS_HEADER_SIZE;
static const uint8_t  MAX_PAYLOAD_SIZE  = MAX_MESSAGE_SIZE - HEADER_SIZE;

// MySensors node-ID registry: max nodes (IDs 1-254; 0=GW, 255=broadcast)
static const uint8_t  MAX_NODE_IDS      = 254u;

// Stale receive-state timeout (milliseconds)
static const uint32_t REC_TIMEOUT_MS    = 100u;

// TCP line buffer size (must fit the longest possible MySensors ASCII line)
static const size_t   TCP_LINE_BUF_SIZE = 512u;

static const uint8_t NO_SENSOR = 0xFF;
static const uint8_t NO_ACK = 0;
static const uint8_t REQ_ACK = 1;

// ── MySensors command types ────────────────────────────────────────────────
typedef enum {
  C_PRESENTATION = 0,
  C_SET          = 1,
  C_REQ          = 2,
  C_INTERNAL     = 3,
  C_STREAM       = 4,
  C_RESERVED_5   = 5,
  C_RESERVED_6   = 6,
  C_INVALID_7    = 7,
} mysensors_command_t;

// ── MySensors sensor types ─────────────────────────────────────────────────
typedef enum {
  S_DOOR                  = 0,
  S_MOTION                = 1,
  S_SMOKE                 = 2,
  S_BINARY                = 3,
  S_LIGHT                 = 3,  ///< deprecated alias
  S_DIMMER                = 4,
  S_COVER                 = 5,
  S_TEMP                  = 6,
  S_HUM                   = 7,
  S_BARO                  = 8,
  S_WIND                  = 9,
  S_RAIN                  = 10,
  S_UV                    = 11,
  S_WEIGHT                = 12,
  S_POWER                 = 13,
  S_HEATER                = 14,
  S_DISTANCE              = 15,
  S_LIGHT_LEVEL           = 16,
  S_ARDUINO_NODE          = 17,
  S_ARDUINO_REPEATER_NODE = 18,
  S_LOCK                  = 19,
  S_IR                    = 20,
  S_WATER                 = 21,
  S_AIR_QUALITY           = 22,
  S_CUSTOM                = 23,
  S_DUST                  = 24,
  S_SCENE_CONTROLLER      = 25,
  S_RGB_LIGHT             = 26,
  S_RGBW_LIGHT            = 27,
  S_COLOR_SENSOR          = 28,
  S_HVAC                  = 29,
  S_MULTIMETER            = 30,
  S_SPRINKLER             = 31,
  S_WATER_LEAK            = 32,
  S_SOUND                 = 33,
  S_VIBRATION             = 34,
  S_MOISTURE              = 35,
  S_INFO                  = 36,
  S_GAS                   = 37,
  S_GPS                   = 38,
  S_WATER_QUALITY         = 39,
} mysensors_sensor_t;

// ── MySensors value types ──────────────────────────────────────────────────
typedef enum {
  V_TEMP              = 0,
  V_HUM               = 1,
  V_STATUS            = 2,
  V_LIGHT             = 2,  ///< deprecated alias
  V_PERCENTAGE        = 3,
  V_DIMMER            = 3,  ///< deprecated alias
  V_PRESSURE          = 4,
  V_FORECAST          = 5,
  V_RAIN              = 6,
  V_RAINRATE          = 7,
  V_WIND              = 8,
  V_GUST              = 9,
  V_DIRECTION         = 10,
  V_UV                = 11,
  V_WEIGHT            = 12,
  V_DISTANCE          = 13,
  V_IMPEDANCE         = 14,
  V_ARMED             = 15,
  V_TRIPPED           = 16,
  V_WATT              = 17,
  V_KWH               = 18,
  V_SCENE_ON          = 19,
  V_SCENE_OFF         = 20,
  V_HVAC_FLOW_STATE   = 21,
  V_HEATER            = 21,  ///< deprecated alias
  V_HVAC_SPEED        = 22,
  V_LIGHT_LEVEL       = 23,
  V_VAR1              = 24,
  V_VAR2              = 25,
  V_VAR3              = 26,
  V_VAR4              = 27,
  V_VAR5              = 28,
  V_UP                = 29,
  V_DOWN              = 30,
  V_STOP              = 31,
  V_IR_SEND           = 32,
  V_IR_RECEIVE        = 33,
  V_FLOW              = 34,
  V_VOLUME            = 35,
  V_LOCK_STATUS       = 36,
  V_LEVEL             = 37,
  V_VOLTAGE           = 38,
  V_CURRENT           = 39,
  V_RGB               = 40,
  V_RGBW              = 41,
  V_ID                = 42,
  V_UNIT_PREFIX       = 43,
  V_HVAC_SETPOINT_COOL = 44,
  V_HVAC_SETPOINT_HEAT = 45,
  V_HVAC_FLOW_MODE    = 46,
  V_TEXT              = 47,
  V_CUSTOM            = 48,
  V_POSITION          = 49,
  V_IR_RECORD         = 50,
  V_PH                = 51,
  V_ORP               = 52,
  V_EC                = 53,
  V_VAR               = 54,
  V_VA                = 55,
  V_POWER_FACTOR      = 56,
} mysensors_data_t;

// ── MySensors internal message subtypes ───────────────────────────────────
typedef enum {
  I_BATTERY_LEVEL          = 0,
  I_TIME                   = 1,
  I_VERSION                = 2,
  I_ID_REQUEST             = 3,
  I_ID_RESPONSE            = 4,
  I_INCLUSION_MODE         = 5,
  I_CONFIG                 = 6,
  I_FIND_PARENT_REQUEST    = 7,
  I_FIND_PARENT_RESPONSE   = 8,
  I_LOG_MESSAGE            = 9,
  I_CHILDREN               = 10,
  I_SKETCH_NAME            = 11,
  I_SKETCH_VERSION         = 12,
  I_REBOOT                 = 13,
  I_GATEWAY_READY          = 14,
  I_SIGNING_PRESENTATION   = 15,
  I_NONCE_REQUEST          = 16,
  I_NONCE_RESPONSE         = 17,
  I_HEARTBEAT_REQUEST      = 18,
  I_PRESENTATION           = 19,
  I_DISCOVER_REQUEST       = 20,
  I_DISCOVER_RESPONSE      = 21,
  I_HEARTBEAT_RESPONSE     = 22,
  I_LOCKED                 = 23,
  I_PING                   = 24,
  I_PONG                   = 25,
  I_REGISTRATION_REQUEST   = 26,
  I_REGISTRATION_RESPONSE  = 27,
  I_DEBUG                  = 28,
  I_SIGNAL_REPORT_REQUEST  = 29,
  I_SIGNAL_REPORT_REVERSE  = 30,
  I_SIGNAL_REPORT_RESPONSE = 31,
  I_PRE_SLEEP_NOTIFICATION = 32,
  I_POST_SLEEP_NOTIFICATION = 33,
} mysensors_internal_t;

// ── MySensors stream subtypes ──────────────────────────────────────────────
typedef enum {
  ST_FIRMWARE_CONFIG_REQUEST  = 0,
  ST_FIRMWARE_CONFIG_RESPONSE = 1,
  ST_FIRMWARE_REQUEST         = 2,
  ST_FIRMWARE_RESPONSE        = 3,
  ST_SOUND                    = 4,
  ST_IMAGE                    = 5,
  ST_FIRMWARE_CONFIRM         = 6,
  ST_FIRMWARE_RESPONSE_RLE    = 7,
} mysensors_stream_t;

// ── MySensors payload types ────────────────────────────────────────────────
typedef enum {
  P_STRING  = 0,
  P_BYTE    = 1,
  P_INT16   = 2,
  P_UINT16  = 3,
  P_LONG32  = 4,
  P_ULONG32 = 5,
  P_CUSTOM  = 6,
  P_FLOAT32 = 7,
} mysensors_payload_t;

// ── Header byte-field masks & shifts ──────────────────────────────────────
// Byte 3 (VSL): [7:3]=length(5b) [2]=signed(1b) [1:0]=version(2b)
static const uint8_t VSL_VERSION_MASK  = 0x03u;
static const uint8_t VSL_VERSION_SHIFT = 0u;
static const uint8_t VSL_LENGTH_MASK   = 0xF8u;
static const uint8_t VSL_LENGTH_SHIFT  = 3u;
// Byte 4 (CEP): [7:5]=payloadtype(3b) [4]=echo(1b) [3]=echoreq(1b) [2:0]=command(3b)
static const uint8_t CEP_COMMAND_MASK      = 0x07u;
static const uint8_t CEP_COMMAND_SHIFT     = 0u;
static const uint8_t CEP_ECHOREQ_MASK      = 0x08u;
static const uint8_t CEP_ECHOREQ_SHIFT     = 3u;
static const uint8_t CEP_PAYLOADTYPE_MASK  = 0xE0u;
static const uint8_t CEP_PAYLOADTYPE_SHIFT = 5u;


class MySensorsGW : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  MySensorsGW() = default;

  // ESPHome lifecycle
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;

  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

  // Configuration setters (called from __init__.py generated code)
  void set_flow_control_pin(GPIOPin *pin) { this->flow_control_pin_ = pin; }
  void set_port(uint16_t port)            { this->port_ = port; }

 protected:
  // ── TCP server helpers ───────────────────────────────────────────────────
  void accept_();
  void tcp_read_();
  void tcp_write_(const char *buf, size_t len);
  void client_close_();

  // ── RS485 receive state machine ──────────────────────────────────────────
  void rs485_read_();
  void rec_reset_();

  // ── Message processing ───────────────────────────────────────────────────
  void announce_();
  void decode_();
  void encode_(const char *line);
  void rs485_send_();

  // ── Internal protocol handlers ────────────────────────────────────────────
  void handle_find_parent_(uint8_t src, uint8_t ver);
  void handle_ping_(uint8_t src, uint8_t ver, uint8_t hop);
  void handle_id_request_(uint8_t src, uint8_t ver);
  void handle_time_request_(uint8_t src, uint8_t sns, uint8_t ver);

  // ── Node ID registry ─────────────────────────────────────────────────────
  uint8_t next_node_id_();
  bool    node_registered_(uint8_t id) const;
  void    register_node_(uint8_t id);

  // ── Members ──────────────────────────────────────────────────────────────
  std::unique_ptr<esphome::socket::Socket> socket_{nullptr};
  std::unique_ptr<esphome::socket::Socket> client_{nullptr};
  GPIOPin  *flow_control_pin_{nullptr};
  uint16_t  port_{5003};

  // TCP line accumulation buffer (handles split reads)
  char     tcp_line_buf_[TCP_LINE_BUF_SIZE];
  size_t   tcp_line_len_{0};

  // RS485 receive state machine
  uint8_t  rec_phase_{0};
  uint8_t  rec_pos_{0};
  uint8_t  rec_command_{0};
  uint8_t  rec_len_{0};
  uint8_t  rec_station_{0};
  uint8_t  rec_sender_{0};
  uint8_t  rec_cs_{0};
  uint8_t  rec_calc_cs_{0};
  uint8_t  rec_header_[6]{};
  uint8_t  rec_message_[MAX_MESSAGE_SIZE + 1]{};
  uint32_t rec_last_byte_ms_{0};  ///< millis() of last received byte (for timeout)

  // RS485 transmit
  uint8_t  snd_len_{0};
  uint8_t  snd_receiver_{0};
  uint8_t  snd_message_[MAX_MESSAGE_SIZE + 1]{};

  // Node ID registry (bitmap: bit N set = node N is in use)
  uint8_t  node_id_bitmap_[32]{};   ///< 256 bits for IDs 0-255
  uint8_t  next_free_id_{1};        ///< rolling search cursor
};

}  // namespace mysensorsgw
}  // namespace esphome
