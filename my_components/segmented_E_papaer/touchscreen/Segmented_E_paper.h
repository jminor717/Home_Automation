#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace gt911 {

struct Store {
  volatile bool available;

  static void gpio_intr(Store *store);
};

class GT911ButtonListener {
 public:
  virtual void update_button(uint8_t index, bool state) = 0;
};

class GT911Touchscreen : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_busy_pin(InternalGPIOPin* pin) { this->busy_pin_ = pin; }
  void set_reset_pin(InternalGPIOPin* pin) { this->reset_pin_ = pin; }
  void register_button_listener(GT911ButtonListener *listener) { this->button_listeners_.push_back(listener); }

 protected:
     InternalGPIOPin* busy_pin_;
     InternalGPIOPin* reset_pin_;
     Store store_;
     std::vector<GT911ButtonListener*> button_listeners_;
};

}  // namespace gt911
}  // namespace esphome
