#pragma once

#include <Arduino.h>
#include <WiFiClient.h>
#include <functional>
#include <string>
#include <vector>

namespace esphome {
namespace ocpp {

// Minimal RFC 6455 WebSocket client over Arduino WiFiClient.
// Client-initiated, outbound only. Text frames, ping/pong, close.
// No TLS (ws:// only). No extensions, no subprotocol negotiation
// beyond a single offered protocol string.
class WsClient {
 public:
  using TextCallback = std::function<void(const char *data, size_t len)>;

  enum State : uint8_t {
    STATE_DISCONNECTED = 0,
    STATE_CONNECTING,        // TCP connect in flight
    STATE_HANDSHAKING,       // HTTP upgrade response pending
    STATE_OPEN,              // frames flowing
    STATE_CLOSING,           // close frame sent or received, waiting to tear down
  };

  void set_url(const std::string &url) { url_ = url; }
  void set_subprotocol(const std::string &p) { subprotocol_ = p; }
  void on_text(TextCallback cb) { on_text_ = std::move(cb); }

  // Kick off an async connect. Returns false if URL can't be parsed.
  bool begin();

  // Pump I/O. Call frequently from the main loop.
  void loop();

  // Send a text frame. Returns true on success (queued to socket).
  bool send_text(const char *data, size_t len);

  bool is_open() const { return state_ == STATE_OPEN; }
  State state() const { return state_; }
  uint32_t last_connected_ms() const { return last_connected_ms_; }

 protected:
  bool parse_url_();
  bool start_tcp_();
  bool send_http_upgrade_();
  bool read_http_response_();
  void read_frames_();
  bool send_frame_(uint8_t opcode, const uint8_t *payload, size_t len);
  void close_(uint16_t code, const char *reason);
  void reset_();

  WiFiClient client_;
  std::string url_;
  std::string subprotocol_{"ocpp1.6"};
  std::string host_;
  std::string path_{"/"};
  uint16_t port_{80};
  TextCallback on_text_;

  State state_{STATE_DISCONNECTED};
  uint32_t state_enter_ms_{0};
  uint32_t last_connected_ms_{0};
  uint32_t last_ping_ms_{0};

  std::string handshake_key_;        // base64 of 16 random bytes
  std::string handshake_response_;   // accumulating HTTP response
  std::vector<uint8_t> frame_buf_;   // frame reassembly (single fragment)
};

}  // namespace ocpp
}  // namespace esphome
