#pragma once

#include "esphome.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
// using namespace i2c;

namespace esphome {
namespace pic_programer {
    class Pic_Programer;

    class Pic_Programer : public Component {
    public:
        float get_setup_priority() const override { return setup_priority::HARDWARE; }
        void setup() override;
        void loop() override;
        void dump_config() override;

        void set_Busy_pin(InternalGPIOPin* pin) { this->Busy_pin_ = pin; };
        void set_Reset_pin(InternalGPIOPin* pin) { this->Reset_pin_ = pin; };


    protected:
        static const uint64_t UINT_64_MAX = 0xfffffffffffffff; // one byte short because we are adding ~10 seconds worth of uS to this value when the screen is active and dont want an overflow

        InternalGPIOPin* Busy_pin_;
        InternalGPIOPin* Reset_pin_;
    };

} // namespace pic_programer
} // namespace esphome
