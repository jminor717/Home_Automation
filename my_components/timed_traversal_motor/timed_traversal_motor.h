#pragma once

#include "esphome.h"
// #include "esphome/components/button/button.h"
#include "esphome/components/adc/adc_sensor.h"
#include "esphome/components/globals/globals_component.h"
#include "esphome/components/servo/servo.h"
// #include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
// using namespace i2c;

namespace esphome {
namespace timed_traversal_motor {
    struct Position {
        float comand_position;
        float voltage_read;
    };

    class TimedTraversalMotor : public output::FloatOutput, public Component {
    public:
        float get_setup_priority() const override { return setup_priority::DATA; }
        void setup() override;
        void dump_config() override;
        /// Override FloatOutput's write_state.
        void write_state(float state) override;

    protected:
        static void find_travel_limits(void* params);
    };
} // namespace timed_traversal_motor
} // namespace esphome
