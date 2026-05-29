#include "MySensorsGW.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/hal.h"
#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

#include <cstring>
#include <cerrno>
#include <cstdio>
#include <cstdlib>

namespace esphome {
namespace mysensorsgw {

static const char *const TAG = "MySensorsGW";

// ── Lifecycle ────────────────────────────────────────────────────────────────

void MySensorsGW::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MySensors RS485 gateway...");

  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
    this->flow_control_pin_->digital_write(false);  // RX mode initially
  }

  // Mark gateway address as reserved
  this->register_node_(GATEWAY_ADDRESS);
  this->register_node_(BROADCAST_ADDRESS);

  // Create non-blocking TCP server socket
  struct sockaddr_storage bind_addr;
  socklen_t bind_addrlen =
      socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr),
                               sizeof(bind_addr), this->port_);

  this->socket_ = socket::socket_ip(SOCK_STREAM, PF_INET);
  this->socket_->setblocking(false);

  int opt = 1;
  this->socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), bind_addrlen);
  this->socket_->listen(4);

  ESP_LOGCONFIG(TAG, "  Listening on port %u", this->port_);
}

void MySensorsGW::loop() {
  this->accept_();
  this->rs485_read_();
  this->tcp_read_();
}

void MySensorsGW::dump_config() {
  ESP_LOGCONFIG(TAG, "MySensors RS485 Gateway:");
  ESP_LOGCONFIG(TAG, "  Address: %s:%u",
                esphome::network::get_use_address(), this->port_);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
}

void MySensorsGW::on_shutdown() {
  this->client_close_();
  if (this->socket_)
    this->socket_->close();
}

// ── TCP server ───────────────────────────────────────────────────────────────

void MySensorsGW::accept_() {
  // If a client is already connected, don't accept a new one yet
  if (this->client_)
    return;

  struct sockaddr_storage client_addr;
  socklen_t client_addrlen = sizeof(client_addr);
  auto new_client = this->socket_->accept(
      reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);

  if (!new_client)
    return;

  new_client->setblocking(false);
  this->client_ = std::move(new_client);
  this->tcp_line_len_ = 0;

  ESP_LOGD(TAG, "New client connected from %s", inet_ntoa(reinterpret_cast<struct sockaddr_in *>(&client_addr)->sin_addr));

  this->announce_();
}

void MySensorsGW::client_close_() {
  if (!this->client_)
    return;
  this->client_->shutdown(SHUT_RDWR);
  this->client_->close();
  this->client_ = nullptr;
  this->tcp_line_len_ = 0;
  ESP_LOGD(TAG, "Client disconnected");
}

void MySensorsGW::tcp_write_(const char *buf, size_t len) {
  if (!this->client_)
    return;
  ESP_LOGD(TAG, "GW→TCP: %s", buf);
  ssize_t res = this->client_->write(buf, len);
  if (res < 0) {
    ESP_LOGW(TAG, "TCP write error, closing client");
    this->client_close_();
  }
}

// Read from TCP, accumulating into a line buffer so that split TCP reads are
// handled correctly.  A complete line is only processed once '\n' arrives.
void MySensorsGW::tcp_read_() {
  if (!this->client_)
    return;

  // Read as many bytes as available into the line buffer
  while (true) {
    if (this->tcp_line_len_ >= TCP_LINE_BUF_SIZE - 1) {
      // Buffer full without a newline – discard and reset
      ESP_LOGW(TAG, "TCP line buffer overflow, discarding");
      this->tcp_line_len_ = 0;
      break;
    }

    char tmp[64];
    ssize_t n = this->client_->read(tmp, sizeof(tmp));

    if (n == 0) {
      // Peer closed connection gracefully
      this->client_close_();
      return;
    }
    if (n < 0) {
      if (errno == EWOULDBLOCK || errno == EAGAIN)
        break;  // No more data right now
      ESP_LOGW(TAG, "TCP read error (%d), closing client", errno);
      this->client_close_();
      return;
    }

    // Append to line buffer and look for newlines
    for (ssize_t i = 0; i < n; i++) {
      char c = tmp[i];
      if (c == '\n') {
        // Null-terminate and process the line
        this->tcp_line_buf_[this->tcp_line_len_] = '\0';
        if (this->tcp_line_len_ > 0) {
          ESP_LOGD(TAG, "TCP→GW: %s", this->tcp_line_buf_);
          this->encode_(this->tcp_line_buf_);
        }
        this->tcp_line_len_ = 0;
      } else if (c != '\r') {
        this->tcp_line_buf_[this->tcp_line_len_++] = c;
        if (this->tcp_line_len_ >= TCP_LINE_BUF_SIZE - 1) {
          ESP_LOGW(TAG, "TCP line too long, discarding");
          this->tcp_line_len_ = 0;
          break;
        }
      }
    }
  }
}

// ── RS485 receive state machine ──────────────────────────────────────────────

void MySensorsGW::rec_reset_() {
  this->rec_phase_ = 0;
  this->rec_pos_   = 0;
}

// Drain ALL available bytes from the UART in one loop() call
void MySensorsGW::rs485_read_() {
  // Timeout: if we've been stuck mid-packet too long, reset
  if (this->rec_phase_ != 0 &&
      (millis() - this->rec_last_byte_ms_) > REC_TIMEOUT_MS) {
    ESP_LOGW(TAG, "RS485 receive timeout – resetting state machine");
    this->rec_reset_();
  }

  while (this->available()) {
    uint8_t byte;
    if (!this->read_byte(&byte))
      break;

    this->rec_last_byte_ms_ = millis();
    ESP_LOGVV(TAG, "RS485 byte: 0x%02X (phase=%d)", byte, this->rec_phase_);

    switch (this->rec_phase_) {
      // ── Phase 0: hunt for a valid 6-byte header ─────────────────────────
      case 0:
        // Shift header window
        memmove(&this->rec_header_[0], &this->rec_header_[1], 5);
        this->rec_header_[5] = byte;

        // Valid header: [SOH][dst][src][cmd][len][STX]
        // and src != dst (no self-send), dst == GW or broadcast
        if (this->rec_header_[0] == ICSC_SOH &&
            this->rec_header_[5] == ICSC_STX &&
            this->rec_header_[1] != this->rec_header_[2]) {

          uint8_t dst = this->rec_header_[1];
          uint8_t src = this->rec_header_[2];
          uint8_t len = this->rec_header_[4];

          // Only accept packets addressed to us or broadcast
          if (src == GATEWAY_ADDRESS ||
              (dst != GATEWAY_ADDRESS && dst != BROADCAST_ADDRESS)) {
            break;  // Not for us; keep shifting
          }

          if (len >= MAX_MESSAGE_SIZE) {
            ESP_LOGW(TAG, "RS485 packet too large (%d), ignoring", len);
            break;
          }

          this->rec_calc_cs_ = 0;
          this->rec_station_ = dst;
          this->rec_sender_  = src;
          this->rec_command_ = this->rec_header_[3];
          this->rec_len_     = len;

          for (uint8_t i = 1; i <= 4; i++)
            this->rec_calc_cs_ += this->rec_header_[i];

          this->rec_pos_   = 0;
          this->rec_phase_ = (len == 0) ? 2 : 1;
        }
        break;

      // ── Phase 1: receive data bytes ──────────────────────────────────────
      case 1:
        this->rec_message_[this->rec_pos_++] = byte;
        this->rec_calc_cs_ += byte;
        if (this->rec_pos_ == this->rec_len_)
          this->rec_phase_ = 2;
        break;

      // ── Phase 2: expect ETX ──────────────────────────────────────────────
      case 2:
        if (byte == ICSC_ETX) {
          this->rec_phase_ = 3;
        } else {
          ESP_LOGW(TAG, "RS485 expected ETX, got 0x%02X", byte);
          this->rec_reset_();
        }
        break;

      // ── Phase 3: receive checksum ────────────────────────────────────────
      case 3:
        this->rec_cs_    = byte;
        this->rec_phase_ = 4;
        break;

      // ── Phase 4: expect EOT, validate checksum ───────────────────────────
      case 4:
        if (byte == ICSC_EOT) {
          if (this->rec_cs_ == this->rec_calc_cs_) {
            if (this->rec_command_ == ICSC_SYS_PACK)
              this->decode_();
          } else {
            ESP_LOGW(TAG, "RS485 checksum mismatch (got 0x%02X, expected 0x%02X)",
                     this->rec_cs_, this->rec_calc_cs_);
          }
        } else {
          ESP_LOGW(TAG, "RS485 expected EOT, got 0x%02X", byte);
        }
        this->rec_reset_();
        break;

      default:
        this->rec_reset_();
        break;
    }
  }
}

// ── Message decode (RS485 → TCP) ─────────────────────────────────────────────

void MySensorsGW::decode_() {
  if (this->rec_len_ < HEADER_SIZE)
    return;

  // ── Parse MySensors v2 header fields ──────────────────────────────────
  uint8_t src = this->rec_message_[1];
  uint8_t dst = this->rec_message_[2];
  uint8_t vsl = this->rec_message_[3];
  uint8_t cep = this->rec_message_[4];
  uint8_t sub = this->rec_message_[5];
  uint8_t sns = this->rec_message_[6];

  uint8_t ver = (vsl & VSL_VERSION_MASK) >> VSL_VERSION_SHIFT;
  uint8_t len = (vsl & VSL_LENGTH_MASK)  >> VSL_LENGTH_SHIFT;
  uint8_t cmd = (cep & CEP_COMMAND_MASK) >> CEP_COMMAND_SHIFT;
  uint8_t ack = (cep & CEP_ECHOREQ_MASK) >> CEP_ECHOREQ_SHIFT;
  uint8_t typ = (cep & CEP_PAYLOADTYPE_MASK) >> CEP_PAYLOADTYPE_SHIFT;

  // ── Handle internal messages that the gateway should respond to ────────
  if (cmd == C_INTERNAL) {
    switch (sub) {
      case I_FIND_PARENT_REQUEST:
        this->handle_find_parent_(src, ver);
        return;

      case I_PING:
        this->handle_ping_(src, ver, this->rec_message_[7]);
        return;

      case I_ID_REQUEST:
        this->handle_id_request_(src, ver);
        return;

      case I_TIME:
        if (ack == 0)  // request, not a response
          this->handle_time_request_(src, sns, ver);
        // fall through to also forward to controller
        break;

      case I_REGISTRATION_REQUEST:
        this->register_node_(src);
        // Send I_REGISTRATION_RESPONSE (payload = 1 = allowed)
        {
          uint8_t plen = 1;
          this->snd_receiver_    = src;
          this->snd_len_         = HEADER_SIZE + plen;
          this->snd_message_[0]  = GATEWAY_ADDRESS;
          this->snd_message_[1]  = GATEWAY_ADDRESS;
          this->snd_message_[2]  = src;
          this->snd_message_[3]  = (uint8_t)(ver | (plen << VSL_LENGTH_SHIFT));
          this->snd_message_[4]  = (uint8_t)(C_INTERNAL | (P_BYTE << CEP_PAYLOADTYPE_SHIFT));
          this->snd_message_[5]  = I_REGISTRATION_RESPONSE;
          this->snd_message_[6]  = 0xFF;
          this->snd_message_[7]  = 1;
          this->rs485_send_();
        }
        break;

      default:
        break;
    }
  }

  // ── Build the MySensors ASCII serial protocol string ───────────────────
  // Format: node-id;child-sensor-id;message-type;ack;sub-type;payload\n
  char buf[TCP_LINE_BUF_SIZE];
  char *cur       = buf;
  char *const end = buf + sizeof(buf);

  cur += snprintf(cur, (size_t)(end - cur), "%u;%u;%u;%u;%u;",
                  src, sns, cmd, ack, sub);

  // Payload – safe float extraction via memcpy to avoid UB/alignment issues
  switch (typ) {
    case P_STRING:
    case P_CUSTOM: {
      // Ensure null-termination within bounds before printing
      uint8_t slen = (len < MAX_PAYLOAD_SIZE) ? len : (uint8_t)(MAX_PAYLOAD_SIZE - 1);
      this->rec_message_[HEADER_SIZE + slen] = '\0';
      cur += snprintf(cur, (size_t)(end - cur), "%s",
                      reinterpret_cast<char *>(&this->rec_message_[HEADER_SIZE]));
      break;
    }
    case P_BYTE:
      cur += snprintf(cur, (size_t)(end - cur), "%u", this->rec_message_[HEADER_SIZE]);
      break;
    case P_INT16: {
      int16_t v;
      memcpy(&v, &this->rec_message_[HEADER_SIZE], sizeof(v));
      cur += snprintf(cur, (size_t)(end - cur), "%d", (int)v);
      break;
    }
    case P_UINT16: {
      uint16_t v;
      memcpy(&v, &this->rec_message_[HEADER_SIZE], sizeof(v));
      cur += snprintf(cur, (size_t)(end - cur), "%u", (unsigned)v);
      break;
    }
    case P_LONG32: {
      int32_t v;
      memcpy(&v, &this->rec_message_[HEADER_SIZE], sizeof(v));
      cur += snprintf(cur, (size_t)(end - cur), "%ld", (long)v);
      break;
    }
    case P_ULONG32: {
      uint32_t v;
      memcpy(&v, &this->rec_message_[HEADER_SIZE], sizeof(v));
      cur += snprintf(cur, (size_t)(end - cur), "%lu", (unsigned long)v);
      break;
    }
    case P_FLOAT32: {
      // Safe extraction – avoids strict-aliasing UB and misaligned access
      float v;
      memcpy(&v, &this->rec_message_[HEADER_SIZE], sizeof(v));
      cur += snprintf(cur, (size_t)(end - cur), "%.4f", (double)v);
      break;
    }
    default:
      break;
  }

  // Append newline
  if (cur < end - 1) {
    *cur++ = '\n';
    *cur   = '\0';
  }

  this->tcp_write_(buf, (size_t)(cur - buf));
}

// ── Internal protocol responders ─────────────────────────────────────────────

void MySensorsGW::handle_find_parent_(uint8_t src, uint8_t ver) {
  uint8_t plen = 1;
  this->snd_receiver_   = src;
  this->snd_len_        = HEADER_SIZE + plen;
  this->snd_message_[0] = GATEWAY_ADDRESS;
  this->snd_message_[1] = GATEWAY_ADDRESS;
  this->snd_message_[2] = src;
  this->snd_message_[3] = (uint8_t)(ver | (plen << VSL_LENGTH_SHIFT));
  this->snd_message_[4] = (uint8_t)(C_INTERNAL | (P_BYTE << CEP_PAYLOADTYPE_SHIFT));
  this->snd_message_[5] = I_FIND_PARENT_RESPONSE;
  this->snd_message_[6] = 0xFF;
  this->snd_message_[7] = 0;   // distance = 0 (we are the gateway)
  this->rs485_send_();
}

void MySensorsGW::handle_ping_(uint8_t src, uint8_t ver, uint8_t hop) {
  uint8_t plen = 1;
  this->snd_receiver_   = src;
  this->snd_len_        = HEADER_SIZE + plen;
  this->snd_message_[0] = GATEWAY_ADDRESS;
  this->snd_message_[1] = GATEWAY_ADDRESS;
  this->snd_message_[2] = src;
  this->snd_message_[3] = (uint8_t)(ver | (plen << VSL_LENGTH_SHIFT));
  this->snd_message_[4] = (uint8_t)(C_INTERNAL | (P_BYTE << CEP_PAYLOADTYPE_SHIFT));
  this->snd_message_[5] = I_PONG;
  this->snd_message_[6] = 0xFF;
  this->snd_message_[7] = hop;  // echo the hop counter back
  this->rs485_send_();
}

void MySensorsGW::handle_id_request_(uint8_t src, uint8_t ver) {
  uint8_t new_id = this->next_node_id_();
  if (new_id == 0) {
    ESP_LOGW(TAG, "No free node IDs available");
    return;
  }
  this->register_node_(new_id);
  ESP_LOGD(TAG, "Assigned node ID %u to requesting node %u", new_id, src);

  uint8_t plen = 1;
  this->snd_receiver_   = BROADCAST_ADDRESS;  // Spec: ID response is broadcast
  this->snd_len_        = HEADER_SIZE + plen;
  this->snd_message_[0] = GATEWAY_ADDRESS;
  this->snd_message_[1] = GATEWAY_ADDRESS;
  this->snd_message_[2] = BROADCAST_ADDRESS;
  this->snd_message_[3] = (uint8_t)(ver | (plen << VSL_LENGTH_SHIFT));
  this->snd_message_[4] = (uint8_t)(C_INTERNAL | (P_BYTE << CEP_PAYLOADTYPE_SHIFT));
  this->snd_message_[5] = I_ID_RESPONSE;
  this->snd_message_[6] = 0xFF;
  this->snd_message_[7] = new_id;
  this->rs485_send_();
}

void MySensorsGW::handle_time_request_(uint8_t src, uint8_t sns, uint8_t ver) {
  // Provide current epoch time (seconds).  ESPHome's sntp component keeps
  // this up to date; we use id(sntp_time).now().timestamp when available,
  // but millis()/1000 is a safe fallback that keeps nodes from hanging.
  uint32_t t = (uint32_t)(millis() / 1000UL);

  uint8_t plen = 4;
  this->snd_receiver_   = src;
  this->snd_len_        = HEADER_SIZE + plen;
  this->snd_message_[0] = GATEWAY_ADDRESS;
  this->snd_message_[1] = GATEWAY_ADDRESS;
  this->snd_message_[2] = src;
  this->snd_message_[3] = (uint8_t)(ver | (plen << VSL_LENGTH_SHIFT));
  this->snd_message_[4] = (uint8_t)(C_INTERNAL | (P_ULONG32 << CEP_PAYLOADTYPE_SHIFT));
  this->snd_message_[5] = I_TIME;
  this->snd_message_[6] = sns;
  memcpy(&this->snd_message_[7], &t, sizeof(t));
  this->rs485_send_();
}

// ── Message encode (TCP → RS485) ──────────────────────────────────────────────

void MySensorsGW::announce_() {
  char buf[TCP_LINE_BUF_SIZE];
  
  snprintf(buf, sizeof(buf), "%u;%u;%u;%u;%u;%s\n",
             GATEWAY_ADDRESS,
             NO_SENSOR,
             C_INTERNAL,
             NO_ACK,
             I_SIGNING_PRESENTATION,
             "");
  this->tcp_write_(buf, strlen(buf));

  snprintf(buf, sizeof(buf), "%u;%u;%u;%u;%u;%u.%u.%u\n",
             GATEWAY_ADDRESS,
             NO_SENSOR,
             C_PRESENTATION,
             NO_ACK,
             S_ARDUINO_NODE,
             MY_MYSENSORS_VERSION_STR_MAJOR,
             MY_MYSENSORS_VERSION_STR_MINOR,
             MY_MYSENSORS_VERSION_STR_PATCH);
  this->tcp_write_(buf, strlen(buf));

  snprintf(buf, sizeof(buf), "%u;%u;%u;%u;%u;%s\n",
             GATEWAY_ADDRESS,
             NO_SENSOR,
             C_INTERNAL,
             NO_ACK,
             I_SKETCH_NAME,
             "ESPHOME RS485 GW");
  this->tcp_write_(buf, strlen(buf));

  snprintf(buf, sizeof(buf), "%u;%u;%u;%u;%u;%u.%u.%u\n",
             GATEWAY_ADDRESS,
             NO_SENSOR,
             C_INTERNAL,
             NO_ACK,
             I_SKETCH_VERSION,
             MY_MYSENSORS_VERSION_STR_MAJOR,
             MY_MYSENSORS_VERSION_STR_MINOR,
             MY_MYSENSORS_VERSION_STR_PATCH);
  this->tcp_write_(buf, strlen(buf));

  snprintf(buf, sizeof(buf), "%u;%u;%u;%u;%u;%u\n",
             GATEWAY_ADDRESS,
             NO_SENSOR,
             C_INTERNAL,
             NO_ACK,
             I_BATTERY_LEVEL,
             100);
  this->tcp_write_(buf, strlen(buf));
}

void MySensorsGW::encode_(const char *line) {
  // MySensors ASCII protocol: node;child;cmd;ack;subtype;payload
  // We must not modify the caller's buffer, so work on a local copy.
  char buf[TCP_LINE_BUF_SIZE];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  uint8_t dst = 0, child = 0, cmd = 0, ack_flag = 0, sub = 0;
  char   *payload = nullptr;

  char *saveptr = nullptr;
  char *tok     = nullptr;
  uint8_t idx   = 0;

  for (tok = strtok_r(buf, ";", &saveptr);
       tok != nullptr && idx <= 5;
       tok = strtok_r(nullptr, ";", &saveptr), idx++) {
    switch (idx) {
      case 0: dst       = (uint8_t)atoi(tok); break;
      case 1: child     = (uint8_t)atoi(tok); break;
      case 2: cmd       = (uint8_t)atoi(tok); break;
      case 3: ack_flag  = (uint8_t)atoi(tok); break;
      case 4: sub       = (uint8_t)atoi(tok); break;
      case 5: payload   = tok;                break;
    }
  }

  // ── Handle version query locally (avoids unnecessary RS485 traffic) ─────
  if (cmd == C_INTERNAL && sub == I_VERSION) {
    char resp[64];
    snprintf(resp, sizeof(resp), "%u;%u;%u;%u;%u;%u.%u.%u\n",
             GATEWAY_ADDRESS,
             NO_SENSOR,
             C_INTERNAL,
             0,
             I_VERSION,
             MY_MYSENSORS_VERSION_STR_MAJOR,
             MY_MYSENSORS_VERSION_STR_MINOR,
             MY_MYSENSORS_VERSION_STR_PATCH);
    this->tcp_write_(resp, strlen(resp));
    return;
  }

  // ── Build MySensors v2 binary header ──────────────────────────────────
  // Determine payload bytes and type
  uint8_t  plen    = 0;
  uint8_t  ptyp    = P_STRING;
  uint8_t  pbuf[MAX_PAYLOAD_SIZE]{};

  if (payload != nullptr && payload[0] != '\0') {
    size_t slen = strlen(payload);
    if (slen > MAX_PAYLOAD_SIZE)
      slen = MAX_PAYLOAD_SIZE;
    memcpy(pbuf, payload, slen);
    plen = (uint8_t)slen;
    ptyp = P_STRING;
  }

  this->snd_receiver_   = dst;
  this->snd_len_        = HEADER_SIZE + plen;
  this->snd_message_[0] = GATEWAY_ADDRESS;   // MySensors msg[0] = last node
  this->snd_message_[1] = GATEWAY_ADDRESS;   // msg[1] = sender (us)
  this->snd_message_[2] = dst;               // msg[2] = destination node
  // VSL byte: version=2, length=plen
  this->snd_message_[3] = (uint8_t)(2u | (plen << VSL_LENGTH_SHIFT));
  // CEP byte: command, ack (echo-request), payload type
  this->snd_message_[4] = (uint8_t)((cmd & CEP_COMMAND_MASK) |
                                     ((ack_flag << CEP_ECHOREQ_SHIFT) & CEP_ECHOREQ_MASK) |
                                     ((ptyp << CEP_PAYLOADTYPE_SHIFT) & CEP_PAYLOADTYPE_MASK));
  this->snd_message_[5] = sub;
  this->snd_message_[6] = child;
  if (plen > 0)
    memcpy(&this->snd_message_[7], pbuf, plen);

  this->rs485_send_();
}

// ── RS485 transmit ────────────────────────────────────────────────────────────

void MySensorsGW::rs485_send_() {
  if (this->snd_len_ < HEADER_SIZE)
    return;

  uint8_t crc = 0;

  // Assert TX enable
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  // RS485 / ICSC frame:
  //   SOH SOH dst src cmd len STX <data[len]> ETX crc EOT
  this->write_byte(ICSC_SOH);
  this->write_byte(ICSC_SOH);

  this->write_byte(this->snd_receiver_);
  crc += this->snd_receiver_;

  this->write_byte(GATEWAY_ADDRESS);
  crc += GATEWAY_ADDRESS;

  this->write_byte(ICSC_SYS_PACK);
  crc += ICSC_SYS_PACK;

  this->write_byte(this->snd_len_);
  crc += this->snd_len_;

  this->write_byte(ICSC_STX);

  for (uint8_t i = 0; i < this->snd_len_; i++) {
    this->write_byte(this->snd_message_[i]);
    crc += this->snd_message_[i];
  }

  this->write_byte(ICSC_ETX);
  this->write_byte(crc);
  this->write_byte(ICSC_EOT);
  this->flush();

  // Release TX enable – give the transceiver a moment to finish the last bit
  if (this->flow_control_pin_ != nullptr) {
    delayMicroseconds(2);
    this->flow_control_pin_->digital_write(false);
  }
}

// ── Node ID registry ─────────────────────────────────────────────────────────

bool MySensorsGW::node_registered_(uint8_t id) const {
  return (this->node_id_bitmap_[id >> 3] & (1u << (id & 7u))) != 0;
}

void MySensorsGW::register_node_(uint8_t id) {
  this->node_id_bitmap_[id >> 3] |= (uint8_t)(1u << (id & 7u));
}

// Returns a free node ID in 1-254, or 0 if none available.
uint8_t MySensorsGW::next_node_id_() {
  for (uint16_t tries = 0; tries < MAX_NODE_IDS; tries++) {
    uint8_t candidate = this->next_free_id_;
    // Advance cursor (wrap 1-254)
    this->next_free_id_ = (candidate >= 254) ? 1 : (candidate + 1);
    if (!this->node_registered_(candidate))
      return candidate;
  }
  return 0;  // All IDs exhausted
}

}  // namespace mysensorsgw
}  // namespace esphome
