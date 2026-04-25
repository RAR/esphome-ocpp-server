#pragma once

#include <MicroOcpp/Core/Connection.h>

#include "ws_client.h"

namespace esphome {
namespace ocpp {

// Adapts our RFC 6455 WsClient to MicroOcpp::Connection so MicroOcpp can
// send/receive OCPP-1.6J JSON frames without caring about the underlying
// WebSocket implementation. Defined only when MO_CUSTOM_WS is in effect
// (we don't link links2004/WebSockets on LibreTiny).
class MoConnection : public MicroOcpp::Connection {
 public:
  explicit MoConnection(WsClient *ws) : ws_(ws) {
    ws_->on_text([this](const char *data, size_t len) {
      if (this->receive_cb_) this->receive_cb_(data, len);
    });
  }

  void loop() override { ws_->loop(); }

  bool sendTXT(const char *msg, size_t length) override {
    return ws_->send_text(msg, length);
  }

  void setReceiveTXTcallback(MicroOcpp::ReceiveTXTcallback &cb) override {
    receive_cb_ = cb;
  }

  unsigned long getLastConnected() override { return ws_->last_connected_ms(); }

  bool isConnected() override { return ws_->is_open(); }

 private:
  WsClient *ws_;
  MicroOcpp::ReceiveTXTcallback receive_cb_;
};

}  // namespace ocpp
}  // namespace esphome
