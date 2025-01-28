#include "hard_stop_damper.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hard_stop_damper {
    static const char* const TAG = "Hard_stop_damper";

    void HardStopDamper::setup()
    {
        // this->find_hard_stops();
        xTaskCreate(HardStopDamper::find_hard_stops, "hard_stop_task", 8192 * 2, (void*)this, 0, &this->task_handle);
    }

    Position HardStopDamper::move_to_hard_stop(float increment, float startingPosition, float lowerLimit, float upperLimit)
    {
        this->servo_control->write(startingPosition);
        delay(3000);
        float reachedPosition = this->v_servo_sensor->sample();
        float reachedMapped = startingPosition;
        float commandedPosition = startingPosition;
        while (true) {
            commandedPosition += increment;
            if (commandedPosition > upperLimit) {
                break;
            }
            if (commandedPosition < lowerLimit) {
                break;
            }
            this->servo_control->write(commandedPosition);
            delay(500);
            reachedPosition = this->v_servo_sensor->sample();
            // reachedMapped = remap(reachedPosition, float(0.5), float(2.5), float(0.0), float(1.0));
            reachedMapped = remap(reachedPosition, float(0.0), float(3.3), float(0.0), float(1.0));
            ESP_LOGI(TAG, "commanded: %f, reached: %f, mapped: %f, diff: %f", commandedPosition, reachedPosition, reachedMapped, abs(commandedPosition - reachedMapped));
            if (abs(commandedPosition - reachedMapped) > 0.075) {
                break;
            }
        }
        Position pos;
        pos.comand_position = reachedMapped;
        pos.voltage_read = reachedPosition;
        return pos;
    }

    void HardStopDamper::find_hard_stops(void* params)
    {
        HardStopDamper* local_this = (HardStopDamper*)params;

        delay(2000);
        auto ZERO_ = local_this->move_to_hard_stop(-0.025, 0.5, 0, 1);
        ESP_LOGI(TAG, "stop 0: %f, %f", ZERO_.voltage_read, ZERO_.comand_position);
        auto ONE_ = local_this->move_to_hard_stop(+0.025, 0.5, 0, 1);
        ESP_LOGI(TAG, "stop 1: %f, %f", ONE_.voltage_read, ONE_.comand_position);


        if (local_this->open_at_center) {
            if (local_this->switch_open_and_close){
                local_this->servo_lower_limit->value() = ONE_.voltage_read;
                local_this->close_position->value() = ONE_.comand_position;
            }else{
                local_this->servo_lower_limit->value() = ZERO_.voltage_read;
                local_this->close_position->value() = ZERO_.comand_position;
            }

            local_this->servo_upper_limit->value() = ZERO_.voltage_read + (ONE_.voltage_read - ZERO_.voltage_read);
            local_this->open_position->value() = ZERO_.comand_position + (ONE_.comand_position - ZERO_.comand_position);

        } else if (local_this->switch_open_and_close) {
            local_this->servo_lower_limit->value() = ONE_.voltage_read;
            local_this->close_position->value() = ONE_.comand_position;

            local_this->servo_upper_limit->value() = ZERO_.voltage_read;
            local_this->open_position->value() = ZERO_.comand_position;
        } else {
            local_this->servo_lower_limit->value() = ZERO_.voltage_read;
            local_this->close_position->value() = ZERO_.comand_position;

            local_this->servo_upper_limit->value() = ONE_.voltage_read;
            local_this->open_position->value() = ONE_.comand_position;
        }

        vTaskDelete(NULL);
    }

    void HardStopDamper::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Hard_stop_damper reader:");
        //     LOG_PIN("  Reset pin: ", this->Reset_pin_);
        //     LOG_PIN("  Busy : ", this->Busy_pin_);
    }

} // namespace hard_stop_damper
} // namespace esphome
