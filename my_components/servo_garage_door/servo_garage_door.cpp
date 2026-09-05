#include "dc_relay.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <driver/ledc.h>

namespace esphome {
namespace servo_garage_door {
// copied from ledc_output.cpp

    void Servo_Garage_Door::setup()
    {

        this->servo_event_queue = xQueueCreate(4, sizeof(ServoEvent));
        xTaskCreate(Servo_Garage_Door::backgroundServoControlTask, "servo_task", 8192 * 2, (void*)this, 2, &this->servo_task_handle);
    }


    void Servo_Garage_Door::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Servo_Garage_Door reader:");
    }

    void Servo_Garage_Door::backgroundServoControlTask(void* params)
    {

    }
    
    void Servo_Garage_Door::update()
    {

    }

} // namespace dc_relay
} // namespace esphome
