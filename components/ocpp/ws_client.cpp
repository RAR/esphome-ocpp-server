#include "ws_client.h"

#include "esphome/core/log.h"

#include <cstring>
#include <cstdlib>

// We don't have a SHA1 implementation directly available in ESPHome's API,
// but the HTTP Upgrade handshake's Sec-WebSocket-Accept validation is a
// sanity check, not a security boundary. For a dev build we accept any
// 101 response with the Upgrade: websocket header. TODO: validate Accept
// when we have mbedTLS or port a SHA1 lib. (A mismatched Accept from a
// well-behaved server still points at a protocol error we'd want to catch.)

namespace esphome {
namespace ocpp {

static const char *const TAG = "ocpp.ws";

static std::string base64_encode(const uint8_t *data, size_t len) {
  static const char *b64 =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((len + 2) / 3) * 4);
  size_t i = 0;
  while (i + 3 <= len) {
    uint32_t v = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2];
    out.push_back(b64[(v >> 18) & 0x3F]);
    out.push_back(b64[(v >> 12) & 0x3F]);
    out.push_back(b64[(v >> 6) & 0x3F]);
    out.push_back(b64[v & 0x3F]);
    i += 3;
  }
  if (i < len) {
    uint32_t v = data[i] << 16;
    if (i + 1 < len) v |= data[i + 1] << 8;
    out.push_back(b64[(v >> 18) & 0x3F]);
    out.push_back(b64[(v >> 12) & 0x3F]);
    out.push_back(i + 1 < len ? b64[(v >> 6) & 0x3F] : '=');
    out.push_back('=');
  }
  return out;
}

static void fill_random(uint8_t *buf, size_t n) {
  // LibreTiny exposes random() via Arduino.h; on ESP32/8266 it maps to the
  // hardware RNG. The WebSocket masking key doesn't require cryptographic
  // randomness (RFC 6455 §10.3 calls for "unpredictable", not "secure").
  for (size_t i = 0; i < n; i++) {
    buf[i] = static_cast<uint8_t>(random(0, 256));
  }
}

bool WsClient::parse_url_() {
  // Accept: ws://host[:port][/path]
  if (url_.rfind("ws://", 0) != 0) {
    ESP_LOGE(TAG, "Only ws:// URLs supported (got %s)", url_.c_str());
    return false;
  }
  std::string rest = url_.substr(5);
  size_t slash = rest.find('/');
  std::string hostport = slash == std::string::npos ? rest : rest.substr(0, slash);
  path_ = slash == std::string::npos ? "/" : rest.substr(slash);
  size_t colon = hostport.find(':');
  if (colon == std::string::npos) {
    host_ = hostport;
    port_ = 80;
  } else {
    host_ = hostport.substr(0, colon);
    port_ = static_cast<uint16_t>(atoi(hostport.c_str() + colon + 1));
  }
  return !host_.empty();
}

bool WsClient::begin() {
  if (!parse_url_()) return false;
  state_ = STATE_CONNECTING;
  state_enter_ms_ = millis();
  return start_tcp_();
}

bool WsClient::start_tcp_() {
  ESP_LOGI(TAG, "TCP connect %s:%u", host_.c_str(), port_);
  // Shorten the blocking window on unreachable hosts so we don't trigger
  // ESPHome's "took too long" watchdog at every reconnect attempt. Default
  // WiFiClient timeout is 3s; trim to ~500ms.
  client_.setTimeout(500);
  if (!client_.connect(host_.c_str(), port_)) {
    ESP_LOGW(TAG, "TCP connect failed");
    state_ = STATE_DISCONNECTED;
    state_enter_ms_ = millis();
    return false;
  }
  if (!send_http_upgrade_()) {
    reset_();
    return false;
  }
  state_ = STATE_HANDSHAKING;
  state_enter_ms_ = millis();
  handshake_response_.clear();
  return true;
}

bool WsClient::send_http_upgrade_() {
  uint8_t key_raw[16];
  fill_random(key_raw, sizeof(key_raw));
  handshake_key_ = base64_encode(key_raw, sizeof(key_raw));

  std::string req;
  req.reserve(256);
  req += "GET ";
  req += path_;
  req += " HTTP/1.1\r\nHost: ";
  req += host_;
  if (port_ != 80) {
    req += ":";
    req += std::to_string(port_);
  }
  req +=
      "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: "
      "13\r\nSec-WebSocket-Key: ";
  req += handshake_key_;
  if (!subprotocol_.empty()) {
    req += "\r\nSec-WebSocket-Protocol: ";
    req += subprotocol_;
  }
  req += "\r\n\r\n";

  size_t written = client_.write(reinterpret_cast<const uint8_t *>(req.data()), req.size());
  if (written != req.size()) {
    ESP_LOGW(TAG, "HTTP upgrade write short: %u/%u", (unsigned) written, (unsigned) req.size());
    return false;
  }
  return true;
}

bool WsClient::read_http_response_() {
  // Drain available bytes into handshake_response_ until we see \r\n\r\n.
  while (client_.available()) {
    int b = client_.read();
    if (b < 0) break;
    handshake_response_.push_back(static_cast<char>(b));
    if (handshake_response_.size() >= 4 &&
        handshake_response_.compare(handshake_response_.size() - 4, 4, "\r\n\r\n") == 0) {
      // Full headers received. Minimal validation: status line starts with "HTTP/1.1 101".
      if (handshake_response_.rfind("HTTP/1.1 101", 0) != 0 &&
          handshake_response_.rfind("HTTP/1.0 101", 0) != 0) {
        ESP_LOGW(TAG, "Server did not accept upgrade: %s",
                 handshake_response_.substr(0, 64).c_str());
        return false;
      }
      ESP_LOGI(TAG, "WebSocket open: %s%s", host_.c_str(), path_.c_str());
      state_ = STATE_OPEN;
      state_enter_ms_ = millis();
      last_connected_ms_ = millis();
      // Seed pong timer to "just received one" so the 60 s pong-watchdog
      // doesn't fire spuriously before the first ping/pong cycle completes.
      last_pong_ms_ = millis();
      last_ping_ms_ = millis();
      ping_counter_ = 0;
      return true;
    }
    if (handshake_response_.size() > 4096) {
      ESP_LOGW(TAG, "Handshake response too long");
      return false;
    }
  }
  return true;  // still reading, keep pumping
}

bool WsClient::send_frame_(uint8_t opcode, const uint8_t *payload, size_t len) {
  if (!client_.connected()) return false;
  uint8_t hdr[14];
  size_t hdr_len = 0;
  hdr[hdr_len++] = 0x80 | (opcode & 0x0F);  // FIN + opcode
  uint8_t mask_bit = 0x80;                  // client->server must mask
  if (len < 126) {
    hdr[hdr_len++] = mask_bit | static_cast<uint8_t>(len);
  } else if (len < 65536) {
    hdr[hdr_len++] = mask_bit | 126;
    hdr[hdr_len++] = static_cast<uint8_t>((len >> 8) & 0xFF);
    hdr[hdr_len++] = static_cast<uint8_t>(len & 0xFF);
  } else {
    hdr[hdr_len++] = mask_bit | 127;
    for (int i = 7; i >= 0; i--)
      hdr[hdr_len++] = static_cast<uint8_t>((static_cast<uint64_t>(len) >> (i * 8)) & 0xFF);
  }
  uint8_t mask[4];
  fill_random(mask, 4);
  memcpy(hdr + hdr_len, mask, 4);
  hdr_len += 4;

  if (client_.write(hdr, hdr_len) != hdr_len) return false;

  // Apply XOR mask and send in chunks to avoid a large temporary buffer.
  uint8_t chunk[128];
  size_t i = 0;
  while (i < len) {
    size_t take = std::min(sizeof(chunk), len - i);
    for (size_t j = 0; j < take; j++) chunk[j] = payload[i + j] ^ mask[(i + j) & 3];
    if (client_.write(chunk, take) != take) return false;
    i += take;
  }
  return true;
}

bool WsClient::send_text(const char *data, size_t len) {
  if (state_ != STATE_OPEN) return false;
  return send_frame_(0x1, reinterpret_cast<const uint8_t *>(data), len);
}

void WsClient::read_frames_() {
  // Read a single frame at a time per loop iteration. Extensions, fragmentation,
  // and payloads over 64 KiB are not supported (OCPP messages are tiny).
  if (client_.available() < 2) return;

  uint8_t h0, h1;
  if (client_.read(&h0, 1) != 1 || client_.read(&h1, 1) != 1) return;

  uint8_t opcode = h0 & 0x0F;
  bool fin = (h0 & 0x80) != 0;
  bool masked = (h1 & 0x80) != 0;
  uint64_t payload_len = h1 & 0x7F;
  if (payload_len == 126) {
    uint8_t ext[2];
    while (client_.available() < 2) { delay(1); }
    client_.read(ext, 2);
    payload_len = (ext[0] << 8) | ext[1];
  } else if (payload_len == 127) {
    uint8_t ext[8];
    while (client_.available() < 8) { delay(1); }
    client_.read(ext, 8);
    payload_len = 0;
    for (int i = 0; i < 8; i++) payload_len = (payload_len << 8) | ext[i];
  }

  // Server frames MUST NOT be masked per RFC 6455. Bail if otherwise.
  if (masked) {
    ESP_LOGW(TAG, "Server sent masked frame; closing");
    close_(1002, "masked-from-server");
    return;
  }
  if (!fin) {
    ESP_LOGW(TAG, "Fragmented frames not supported");
    close_(1003, "fragmented");
    return;
  }
  if (payload_len > 65536) {
    ESP_LOGW(TAG, "Payload too large: %llu", (unsigned long long) payload_len);
    close_(1009, "too-big");
    return;
  }

  frame_buf_.resize(static_cast<size_t>(payload_len));
  size_t got = 0;
  uint32_t read_deadline = millis() + 5000;
  while (got < frame_buf_.size()) {
    int avail = client_.available();
    if (avail > 0) {
      int n = client_.read(frame_buf_.data() + got, frame_buf_.size() - got);
      if (n > 0) got += n;
    } else if (millis() > read_deadline) {
      ESP_LOGW(TAG, "Frame read timeout");
      close_(1011, "read-timeout");
      return;
    } else {
      delay(1);
    }
  }

  switch (opcode) {
    case 0x1:  // text
      if (on_text_) on_text_(reinterpret_cast<const char *>(frame_buf_.data()), frame_buf_.size());
      break;
    case 0x8:  // close
      ESP_LOGI(TAG, "Server sent Close (len=%u)", (unsigned) frame_buf_.size());
      close_(1000, "peer-close");
      break;
    case 0x9:  // ping -> pong
      ESP_LOGD(TAG, "Server ping (%u bytes), sending pong", (unsigned) frame_buf_.size());
      if (!send_frame_(0xA, frame_buf_.data(), frame_buf_.size()))
        ESP_LOGW(TAG, "Pong send failed");
      break;
    case 0xA:  // pong
      ESP_LOGD(TAG, "Server pong received");
      last_pong_ms_ = millis();
      break;
    default:
      ESP_LOGW(TAG, "Unsupported opcode 0x%X", opcode);
      close_(1003, "bad-opcode");
      break;
  }
}

void WsClient::close_(uint16_t code, const char *reason) {
  if (state_ == STATE_OPEN) {
    uint8_t body[125];
    body[0] = static_cast<uint8_t>((code >> 8) & 0xFF);
    body[1] = static_cast<uint8_t>(code & 0xFF);
    size_t rl = reason ? strlen(reason) : 0;
    if (rl > sizeof(body) - 2) rl = sizeof(body) - 2;
    if (rl) memcpy(body + 2, reason, rl);
    send_frame_(0x8, body, 2 + rl);
  }
  reset_();
}

void WsClient::reset_() {
  if (client_.connected()) client_.stop();
  state_ = STATE_DISCONNECTED;
  state_enter_ms_ = millis();
  handshake_response_.clear();
  handshake_key_.clear();
  frame_buf_.clear();
}

void WsClient::loop() {
  uint32_t now = millis();

  switch (state_) {
    case STATE_DISCONNECTED: {
      // Reconnect after 5 s when idle. SteVe's session ends fast on transient
      // drops (TCP RST or 1006), and a 30 s gap leaves stations offline far
      // longer than necessary on a flaky LAN.
      if (!url_.empty() && now - state_enter_ms_ > 5000) {
        ESP_LOGI(TAG, "Reconnecting (idle %u ms)",
                 (unsigned)(now - state_enter_ms_));
        begin();
      }
      break;
    }
    case STATE_CONNECTING:
      // Shouldn't stay here long — start_tcp_ transitions synchronously.
      if (now - state_enter_ms_ > 5000) reset_();
      break;
    case STATE_HANDSHAKING:
      if (!read_http_response_() || !client_.connected()) {
        reset_();
      } else if (now - state_enter_ms_ > 5000 && state_ == STATE_HANDSHAKING) {
        ESP_LOGW(TAG, "Handshake timeout");
        reset_();
      }
      break;
    case STATE_OPEN:
      if (!client_.connected()) {
        ESP_LOGW(TAG, "TCP dropped while open (uptime %u ms)",
                 (unsigned)(now - last_connected_ms_));
        reset_();
        break;
      }
      read_frames_();
      // 20 s ping cadence beats any practical Jetty / proxy / NAT idle timeout
      // (most are 30 s / 60 s / 120 s). RFC 6455 §5.5.2 — ping payload is
      // optional but some servers ignore zero-length pings; send a 4-byte
      // monotonic counter so each frame is unique on the wire and visible in
      // packet captures.
      if (now - last_ping_ms_ > 20000) {
        last_ping_ms_ = now;
        ping_counter_++;
        uint8_t payload[4] = {
            static_cast<uint8_t>((ping_counter_ >> 24) & 0xFF),
            static_cast<uint8_t>((ping_counter_ >> 16) & 0xFF),
            static_cast<uint8_t>((ping_counter_ >> 8) & 0xFF),
            static_cast<uint8_t>(ping_counter_ & 0xFF)};
        bool ok = send_frame_(0x9, payload, sizeof(payload));
        ESP_LOGD(TAG, "Sent ping #%u (%s)", (unsigned) ping_counter_,
                 ok ? "ok" : "FAIL");
        if (!ok) {
          ESP_LOGW(TAG, "Ping send failed; tearing down");
          reset_();
          break;
        }
        // If we haven't seen a pong in 60 s, the link is dead — peer may have
        // half-closed without TCP RST. Trigger an explicit reconnect.
        if (last_pong_ms_ != 0 && now - last_pong_ms_ > 60000) {
          ESP_LOGW(TAG, "No pong in %u ms; reconnecting",
                   (unsigned)(now - last_pong_ms_));
          reset_();
        }
      }
      break;
    case STATE_CLOSING:
      reset_();
      break;
  }
}

}  // namespace rippleon_ocpp
}  // namespace esphome
