#include "servo_garage_door.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <driver/ledc.h>

namespace esphome {
namespace servo_garage_door {
    static const char *const TAG = "ServoGarageDoor";
    void ServoGarageDoor::setup()
    {
        for (ServoMotor* motor : this->motors) {
            motor->setup();
        }

        this->servo_event_queue = xQueueCreate(6, sizeof(ServoEvent));
        //   xTaskCreatePinnedToCore(&ESP32Camera::framebuffer_task,
        //                   "framebuffer_task",           // name
        //                   FRAMEBUFFER_TASK_STACK_SIZE,  // stack size
        //                   this,                         // task pv params
        //                   1,                            // priority
        //                   nullptr,                      // handle
        //                   1                             // core
        xTaskCreatePinnedToCore(ServoGarageDoor::backgroundServoControlTask, "servo_task", 8192 * 5, (void*)this, 19, &this->servo_task_handle, 0);

                    // xTaskCreate(ServoGarageDoor::backgroundServoControlTask, "servo_task", 8192 * 5, (void*)this, 2, &this->servo_task_handle);
    }

    void ServoMotor::setup()
    {

        this->step_pin->setup();
        this->step_pin->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);

        this->dir_pin->setup();
        this->dir_pin->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);

        if (this->enable_pin != nullptr) {
            this->enable_pin->setup();
            this->enable_pin->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);
            this->enable_pin->digital_write(0);
        }


    }

    void ServoGarageDoor::move_servo(uint32_t steps, bool direction, uint32_t max_speed, uint32_t acceleration, uint32_t jerks)
    {
        ServoEvent event{};
        event.steps = steps;
        event.direction = direction;
        event.max_speed = max_speed;
        event.acceleration = acceleration;
        event.jerks = jerks;
        ESP_LOGI(TAG, "Enqueuing servo move event");

        if (xQueueSendToBack(this->servo_event_queue, &event, 100 / portTICK_PERIOD_MS) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to enqueue servo move event");
        }
    }

    void ServoGarageDoor::dump_config()
    {
        ESP_LOGCONFIG(TAG, "ServoGarageDoor reader:");
    }

    void ServoGarageDoor::backgroundServoControlTask(void* params)
    {
        ServoGarageDoor* this_controller = static_cast<ServoGarageDoor*>(params);

        // 1 second after startup start running the servo control task
        delay(1'000);

        // run servo control logic in separate thread since it can take a long time to complete a servo movement
        while (true) {
            ServoEvent event{};
            if (xQueueReceive(this_controller->servo_event_queue, &event, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                ESP_LOGI(TAG, "popped servo move event: steps=%lu, direction=%s, max_speed=%lu, acceleration=%lu, jerks=%lu", event.steps, event.direction ? "true" : "false", event.max_speed, event.acceleration, event.jerks);

                if (this_controller->motors.empty()) {
                    ESP_LOGW(TAG, "servo move received but no motors were configured");
                    continue;
                }

                const uint32_t step_pulse_width_us = 10U;

                const uint32_t total_steps = std::max<uint32_t>(1U, event.steps);
                const uint32_t max_speed = std::max<uint32_t>(1U, event.max_speed);
                const uint32_t acceleration = std::max<uint32_t>(1U, event.acceleration);
                const uint32_t jerks = std::max<uint32_t>(1U, event.jerks);

                ServoMotor* motorToUse = nullptr;
                for (ServoMotor* motor : this_controller->motors) {
                    if (motor == nullptr) {
                        continue;
                    }
                    motorToUse = motor;
                    break;
                }

                if (motorToUse == nullptr) {
                    ESP_LOGW(TAG, "No valid motors found for servo move");
                    continue;
                }
                

                motorToUse->dir_pin->digital_write(event.direction ? 1 : 0);
                if (motorToUse->enable_pin != nullptr) {
                    motorToUse->enable_pin->digital_write(1);
                    delay(1000); // wait 1000 ms for enable to take effect
                }

                uint32_t accelerate_steps = total_steps;
                ESP_LOGI(TAG, "servo move:  accelerate_steps=%lu",  accelerate_steps);


                uint32_t timeSpent = 0;
                uint32_t lastPeriod = (uint32_t)((1.0f / 10) * 1000000.0f);
                float currentstepsPerSecond = 10;
                bool isAccelerating = true;
                for (uint32_t step_index = 0; step_index < total_steps; ++step_index) {
                    if(currentstepsPerSecond < (float)max_speed && isAccelerating) {
                        // currentstepsPerSecond = (timeSpent / 1'000'000.0f) * acceleration;
                        currentstepsPerSecond = currentstepsPerSecond + (float)((lastPeriod / 1'000'000.0f) * acceleration);
                        accelerate_steps = step_index;
                    } else if(step_index >= total_steps - accelerate_steps) {
                        currentstepsPerSecond = currentstepsPerSecond - (float)((lastPeriod / 1'000'000.0f) * acceleration);
                    } else {
                        currentstepsPerSecond = (float)max_speed;
                        isAccelerating = false;
                    }

                    if(currentstepsPerSecond < 10) {
                        currentstepsPerSecond = 10;
                    }
                    if(currentstepsPerSecond > (float)max_speed) {
                        currentstepsPerSecond = (float)max_speed;
                    }

                    int32_t period = (int32_t)((1.0f / currentstepsPerSecond) * 1000000.0f);
                    // timeSpent += period;
                    lastPeriod = period;
                    if(period < step_pulse_width_us * 2) {
                        ESP_LOGW(TAG, "pulse width too short : period=%ld, currentstepsPerSecond=%f", period, currentstepsPerSecond);

                        period = step_pulse_width_us * 2;
                    }

                    motorToUse->step_pin->digital_write(1);
                    delayMicroseconds(step_pulse_width_us);
                    motorToUse->step_pin->digital_write(0);
                    delayMicroseconds(period - step_pulse_width_us);
                }

                ESP_LOGI(TAG, "servo move done:  accelerate_steps=%lu",  accelerate_steps);

                if (motorToUse->enable_pin != nullptr) {
                    motorToUse->enable_pin->digital_write(0);
                }
            }
        }
    }


} // namespace servo_garage_door
} // namespace esphome
