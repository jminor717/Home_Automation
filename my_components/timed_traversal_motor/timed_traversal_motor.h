#pragma once

// #include "esphome/components/button/button.h"
// #include "esphome/components/adc/adc_sensor.h"
// #include "esphome/components/globals/globals_component.h"
// #include "esphome/components/servo/servo.h"
// #include "esphome/components/voltage_sampler/voltage_sampler.h"
// #include "esphome/core/automation.h"
#include "esp_timer.h"
#include "esphome.h"
#include "esphome/components/hbridge/fan/hbridge_fan.h"
#include "esphome/components/output/float_output.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// #include <limits>
// using namespace i2c;

namespace esphome {
namespace timed_traversal_motor {

    enum class MovementDirection {
        STILL = 0,
        POS = 1,
        NEG = -1,
    };

    struct TimedTraversalMotorStore {
        QueueHandle_t limit_switch_triggered;
        QueueHandle_t timer_expired;
        uint32_t event_micros;
    };

    class TimedTraversalMotor : public output::FloatOutput, public PollingComponent {
    public:
        TimedTraversalMotor()
        {
            this->store_.limit_switch_triggered = xQueueCreate(1, sizeof(uint32_t));
            this->store_.timer_expired = xQueueCreate(1, sizeof(uint32_t));
            this->state_changed = xQueueCreate(1, sizeof(uint32_t));
        }

        float get_setup_priority() const override { return setup_priority::DATA; }
        void setup() override;
        void dump_config() override;
        /// Override FloatOutput's write_state.
        void write_state(float state) override;
        void update() override;

        static void gpio_intr(TimedTraversalMotorStore* arg);
        static void timer_intr(TimedTraversalMotorStore* arg);

    protected:
        hbridge::HBridgeFan* drive_motor;

        TaskHandle_t task_handle { nullptr };
        TimedTraversalMotorStore store_ {};
        esp_timer_handle_t timer_handle;
        InternalGPIOPin* stop_0_pin;
        InternalGPIOPin* stop_1_pin;
        float current_position = 0;
        float target_position = 0;
        uint32_t expected_travel_time = 0;
        uint32_t total_travel_time = 0;
        bool homed = false;

        MovementDirection is_moving = MovementDirection::STILL;
        uint32_t movement_start_time = 0;

        QueueHandle_t state_changed;
        bool home_requested = false;
        bool state_change_requested = false;
        float state_requested = 0;
        void request_state_change(float state);
        void request_home();

        static void motor_comand_thread(void* params);

        void command_motor(float state);
        uint32_t move_to_hardstop(float speed);
        void home();
    };
} // namespace timed_traversal_motor
} // namespace esphome
