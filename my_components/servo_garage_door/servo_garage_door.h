#pragma once

#include "esphome.h"
// #include "esphome/components/button/button.h"
#include "esphome/components/ledc/ledc_output.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
// using namespace i2c;

namespace esphome {
namespace servo_garage_door {


    class Servo_Garage_Door : public Component {
    public:



    protected:

        QueueHandle_t servo_event_queue;
        TaskHandle_t servo_task_handle { nullptr };
        static void backgroundServoControlTask(void* params);

    };

} // namespace servo_garage_door
} // namespace esphome
