#pragma once

#include "esphome.h"
// #include "esphome/components/button/button.h"
#include "esphome/components/globals/globals_component.h"
#include "esphome/components/servo/servo.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
// using namespace i2c;

namespace esphome {
namespace hard_stop_damper {
    struct Position {
        float comand_position;
        float voltage_read;
    };

    class HardStopDamper : public Component {
    public:
        float get_setup_priority() const override { return setup_priority::DATA; }
        void setup() override;
        // void loop() override;
        void dump_config() override;


        // setters for code gen
        void set_v_servo_sensor(voltage_sampler::VoltageSampler* sen) { this->v_servo_sensor = sen; };
        void set_servo(servo::Servo* ser) { this->servo_control = ser; };
        void set_upper_limit(globals::GlobalsComponent<float>* lim) { this->servo_upper_limit = lim; };
        void set_lower_limit(globals::GlobalsComponent<float>* lim) { this->servo_lower_limit = lim; };
        void set_open_position(globals::GlobalsComponent<float>* lim) { this->open_position = lim; };
        void set_close_position(globals::GlobalsComponent<float>* lim) { this->close_position = lim; };

        void set_open_at_center(bool val) { this->open_at_center = val; };
        void set_switch_open_and_close(bool val) { this->switch_open_and_close = val; };
        void set_open_offset(bool val) { this->open_offset = val; };
        void set_close_offset(bool val) { this->close_offset = val; };

    protected:
        voltage_sampler::VoltageSampler* v_servo_sensor;
        servo::Servo* servo_control;
        globals::GlobalsComponent<float>* servo_upper_limit;
        globals::GlobalsComponent<float>* servo_lower_limit;
        globals::GlobalsComponent<float>* open_position;
        globals::GlobalsComponent<float>* close_position;
        TaskHandle_t task_handle { nullptr };
        bool open_at_center;
        bool switch_open_and_close;
        float open_offset;
        float close_offset;

        static void find_hard_stops(void* params);

        Position move_to_hard_stop(float increment, float startingPosition, float lowerLimit, float upperLimit);
        void setPositions(Position _zero, Position _one);
    };
} // namespace hard_stop_damper
} // namespace esphome
