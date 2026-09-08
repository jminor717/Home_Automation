#pragma once

#include "esphome.h"
// #include "esphome/components/button/button.h"
// #include "esphome/components/ledc/ledc_output.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
// #include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
// using namespace i2c;

namespace esphome {
namespace servo_garage_door {
    class ServoEvent {
    public:
        uint32_t steps;
        bool direction;
        uint32_t max_speed;
        uint32_t acceleration;
        uint32_t jerks;
    };

    class ServoMotor;

    class ServoGarageDoor : public Component {
    public:
        float get_setup_priority() const override { return setup_priority::LATE; }
        void setup() override;
        void dump_config() override;
        // void set_motors(std::vector<ServoMotor*> _motors) { this->motors = std::move(_motors); };
        void set_motors(std::vector<ServoMotor*> _motors) { this->motors = std::move(_motors); };

        void move_servo(uint32_t steps, bool direction, uint32_t max_speed, uint32_t acceleration, uint32_t jerks);
    protected:

        std::vector<ServoMotor*> motors;

        QueueHandle_t servo_event_queue;
        TaskHandle_t servo_task_handle { nullptr };
        static void backgroundServoControlTask(void* params);
        
    };


    class ServoMotor : public Component {
    public:
        void setup() override;

        GPIOPin* step_pin; // min pulse width 2us max frequency 300KHz
        GPIOPin* dir_pin; // min pulse width 2us max frequency 300KHz
        GPIOPin* enable_pin { nullptr }; // min pulse width 100us max frequency 10KHz
        GPIOPin* alarm_pin { nullptr }; // open drain output, active low
        GPIOPin* in_place_pin { nullptr }; // open drain output, active low
        uint32_t steps_per_rotation;
        
        void set_Enable_pin(GPIOPin* pin) { this->enable_pin = pin; };
        void set_Step_pin(GPIOPin* pin) { this->step_pin = pin; };
        void set_Dir_pin(GPIOPin* pin) { this->dir_pin = pin; };
        void set_Alarm_pin(GPIOPin* pin) { this->alarm_pin = pin; };
        void set_In_Place_pin(GPIOPin* pin) { this->in_place_pin = pin; };
        void set_Steps_Per_Rotation(uint32_t steps) { this->steps_per_rotation = steps; };

    };

} // namespace servo_garage_door
} // namespace esphome
