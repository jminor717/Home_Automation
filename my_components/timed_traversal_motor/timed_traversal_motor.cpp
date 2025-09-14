#include "timed_traversal_motor.h"
#include "esphome/components/fan/fan.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace timed_traversal_motor {
    static const char* const TAG = "Timed_Traversal_Motor";



    void TimedTraversalMotor::setup()
    {
        // this->motor_comand_thread();
        this->stop_0_pin->attach_interrupt(TimedTraversalMotor::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
        this->stop_1_pin->attach_interrupt(TimedTraversalMotor::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);

        // create task with a high priority (19) to ensure minimal delay between the gpio limit switch event and the action to stop the motor
        // xTaskCreatePinnedToCore(TimedTraversalMotor::motor_comand_thread, "hard_stop_task", 8192 * 2, (void*)this, 19, &this->task_handle, 1);
        xTaskCreate(TimedTraversalMotor::motor_comand_thread, "hard_stop_task", 8192 * 2, (void*)this, 19, &this->task_handle);

        const esp_timer_create_args_t timer_args = {
            .callback = (esp_timer_cb_t)&TimedTraversalMotor::timer_intr,
            // #ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
            //             .dispatch_method = ESP_TIMER_ISR,
            // #else
            .dispatch_method = ESP_TIMER_TASK,
            // #endif
            .name = "timed_travel_timer"
        };
        esp_timer_create(&timer_args, &this->timer_handle);

        this->request_home();
    }

    void TimedTraversalMotor::write_state(float state)
    {
        if (!this->homed) {
            return;
        }
        // queues a state change to run command_motor on a higher priority thread for better timing accuracy
        this->request_state_change(state);
    }

    void TimedTraversalMotor::command_motor(float state)
    {
        float dist = state - this->current_position;
        this->target_position = state;
        esphome::fan::FanDirection _dir;
        if (dist > 0) {
            this->is_moving = MovementDirection::POS;
            _dir = esphome::fan::FanDirection::FORWARD;
        }
        if (dist < 0) {
            this->is_moving = MovementDirection::NEG;
            _dir = esphome::fan::FanDirection::REVERSE;
        } else {
            this->is_moving = MovementDirection::STILL;
            return;
        }

        dist = abs(dist);
        this->expected_travel_time = this->total_travel_time * dist;

        // todo: start moving motor
        this->drive_motor->turn_on().set_direction(_dir).set_speed(100).perform();

        xQueueReset(this->store_.timer_expired);
        esp_timer_start_once(this->timer_handle, this->expected_travel_time);
        this->movement_start_time = millis();

        uint32_t ulValReceived;
        xQueueReceive(this->store_.timer_expired, &ulValReceived, pdMS_TO_TICKS((this->expected_travel_time + 100'000) / 1000)); // wait for a maximum of the expected travel time plus 100 miliseconds
        this->drive_motor->turn_off().perform();
    }

    void
    TimedTraversalMotor::update()
    {
        int8_t dir = static_cast<int8_t>(this->is_moving);
        if (this->homed && dir) {
            uint32_t moving_for = millis() - this->movement_start_time;
            uint32_t expected_move_time = this->expected_travel_time / 1000;
            float percent_move_completed = float(moving_for) / float(expected_move_time);
            float assumed_position = this->current_position + ((this->target_position - this->current_position) * percent_move_completed);
            // todo: set motor position sensor
        }
    }

    void TimedTraversalMotor::motor_comand_thread(void* params)
    {
        TimedTraversalMotor* local_this = (TimedTraversalMotor*)params;

        while (true) {
            uint32_t ulValReceived;
            xQueueReceive(local_this->state_changed, &ulValReceived, portMAX_DELAY);
            if (local_this->home_requested) {
                local_this->home();
                local_this->home_requested = false;
            }
            if (local_this->state_change_requested) {
                local_this->command_motor(local_this->state_requested);
                local_this->state_change_requested = false;
            }
        }
    }

    void TimedTraversalMotor::home()
    {
        this->homed = false;
        // move to first hardstop no time tracking since we dont know our starting position
        this->move_to_hardstop(1);

        // move in other direction to second hard stop tracking elapsed time
        // if high and low speeds are desired move back to first hardstop ad a different speed to get a linear approximation for commanded speed vs actual speed
        uint32_t travel_time = this->move_to_hardstop(-1);
        this->current_position = 0;
        this->total_travel_time = travel_time;
        this->homed = true;
    }

    uint32_t TimedTraversalMotor::move_to_hardstop(float speed)
    {
        // setup

        bool stop_0 = this->stop_0_pin->digital_read();
        bool stop_1 = this->stop_1_pin->digital_read();
        if (stop_0 && stop_1) {
            return 0;
        }

        esphome::fan::FanDirection _dir;
        esphome::fan::FanDirection _reverse_dir;
        if (speed > 0) {
            this->is_moving = MovementDirection::POS;
            _dir = esphome::fan::FanDirection::FORWARD;
            _reverse_dir = esphome::fan::FanDirection::REVERSE;
        }
        if (speed < 0) {
            this->is_moving = MovementDirection::NEG;
            _dir = esphome::fan::FanDirection::REVERSE;
            _reverse_dir = esphome::fan::FanDirection::FORWARD;
        } else {
            this->is_moving = MovementDirection::STILL;
            return 0;
        }

        xQueueReset(this->store_.limit_switch_triggered);

        // TODO start moving motor
        this->drive_motor->turn_on().set_direction(_dir).set_speed(abs(speed) * 100).perform();
        uint32_t start = micros();

        uint32_t ulValReceived;
        xQueueReceive(this->store_.limit_switch_triggered, &ulValReceived, pdMS_TO_TICKS(60'000)); // give the motor up to a minute to move to the limit switch
        uint32_t end = this->store_.event_micros;
        this->drive_motor->turn_off().perform();

        // slowly back off the limit switch so it is not triggered when we start moving in the other direction
        xQueueReset(this->store_.limit_switch_triggered);
        this->drive_motor->turn_on().set_direction(_reverse_dir).set_speed(10).perform();
        xQueueReceive(this->store_.limit_switch_triggered, &ulValReceived, pdMS_TO_TICKS(60'000)); // give the motor up to a minute to move to the limit switch
        this->drive_motor->turn_off().perform();


        uint32_t dif = end - start;
        return dif;
    }
    void IRAM_ATTR HOT TimedTraversalMotor::timer_intr(TimedTraversalMotorStore* arg)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t ulVarToSend = 2;
        // #ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
        // xQueueOverwriteFromISR(arg->timer_expired, &ulVarToSend, &xHigherPriorityTaskWoken);
        xQueueOverwriteFromISR(arg->timer_expired, &ulVarToSend, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            vPortYield();
        }
    }

    void IRAM_ATTR HOT TimedTraversalMotor::gpio_intr(TimedTraversalMotorStore* arg)
    {
        arg->event_micros = micros();

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t ulVarToSend = 1;
        xQueueOverwriteFromISR(arg->limit_switch_triggered, &ulVarToSend, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            // Writing to the queue caused a task to unblock and the unblocked task
            // has a priority higher than or equal to the priority of the currently
            // executing task (the task this interrupt interrupted).  Perform a context
            // switch so this interrupt returns directly to the unblocked task.
            portYIELD_FROM_ISR(); // or portEND_SWITCHING_ISR() depending on the port.
        }
    }

    void TimedTraversalMotor::request_state_change(float state)
    {
        this->state_change_requested = true;
        this->state_requested = state;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t ulVarToSend = 1;
        xQueueOverwrite(this->state_changed, &ulVarToSend);
    }
    void TimedTraversalMotor::request_home()
    {
        this->home_requested = true;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t ulVarToSend = 1;
        xQueueOverwrite(this->state_changed, &ulVarToSend);
    }
    void TimedTraversalMotor::dump_config()
    {
        ESP_LOGCONFIG(TAG, "timed_traversal_motor reader:");
    }
} // namespace timed_traversal_motor
} // namespace esphome
