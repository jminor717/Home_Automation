#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "packet_transport.h"

namespace esphome::packet_transport {


template<typename... Ts> class SendCommandAction final : public Action<Ts...>, public Parented<PacketTransport> {
 public:
  void set_data_template(const char *consumer, uint32_t command, std::vector<uint8_t> (*func)(Ts...)) {
    this->command_ = command;
    this->data_.func = func;
    this->consumer_ = consumer;
    this->len_ = -1;  // Sentinel value indicates template mode
  }

  void set_data_static(const char *consumer,uint32_t command, const uint8_t *data, size_t len) {
    this->command_ = command;
    this->consumer_ = consumer;
    this->data_.data = data;
    this->len_ = len;  // Length >= 0 indicates static mode
  }

  void set_command_no_data(const char *consumer, uint32_t command) {
    this->command_ = command;
    this->consumer_ = consumer;
    this->len_ = -10;  // Sentinel value indicates no data
  }

  void play(const Ts &...x) override {
    std::vector<uint8_t> data;
    if (this->len_ >= 0) {
      // Static mode: copy from flash to vector
      data.assign(this->data_.data, this->data_.data + this->len_);
    } else if (this->len_ == -1) {
      // Template mode: call function
      data = this->data_.func(x...);
    } else if (this->len_ == -10) {
      // No data, leave vector empty
      data.clear();
    }
    this->parent_->send_command(this->consumer_, this->command_, data);
  }

 protected:
  uint32_t command_;
  const char *consumer_;
  ssize_t len_{-1};  // -10 = no command data, -1 = template mode, >=0 = static mode with length
  union Data {
    std::vector<uint8_t> (*func)(Ts...);  // Function pointer (stateless lambdas)
    const uint8_t *data;                  // Pointer to static data in flash
  } data_;
};

}  // namespace esphome::packet_transport
