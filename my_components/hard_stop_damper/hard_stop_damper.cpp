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
        float lastReached = reachedPosition, commandedPosition = startingPosition;
        while (true) {
            commandedPosition += increment;
            if (commandedPosition > upperLimit || commandedPosition < lowerLimit) {
                break;
            }
            this->servo_control->write(commandedPosition);
            delay(500);
            reachedPosition = this->v_servo_sensor->sample();
            if (abs(reachedPosition - lastReached) < abs(increment * 0.33)) {
                break; // when the distance moved gets small
            }
            lastReached = reachedPosition;
        }
        Position pos;
        pos.comand_position = commandedPosition - increment;
        pos.voltage_read = reachedPosition;
        return pos;
    }

    void HardStopDamper::find_hard_stops(void* params)
    {
        HardStopDamper* local_this = (HardStopDamper*)params;

        delay(2000);
        auto _zero = local_this->move_to_hard_stop(-0.025, 0.5, 0, 1);
        ESP_LOGI(TAG, "stop 0: %f, %f", _zero.voltage_read, _zero.comand_position);
        auto _one = local_this->move_to_hard_stop(+0.025, 0.5, 0, 1);
        ESP_LOGI(TAG, "stop 1: %f, %f", _one.voltage_read, _one.comand_position);
        local_this->setPositions(_zero, _one);

        if (abs(local_this->close_offset) > 0.05) {
            float close_pos = local_this->close_position->value() + local_this->close_offset;
            local_this->servo_control->write(close_pos);
            delay(2000);
            auto reached = local_this->v_servo_sensor->sample();
            local_this->close_position->value() = close_pos;
            local_this->servo_lower_limit->value() = reached;
        }

        if (abs(local_this->open_offset) > 0.05) {
            float open_pos = local_this->open_position->value() + local_this->open_offset;
            local_this->servo_control->write(open_pos);
            delay(2000);
            auto reached = local_this->v_servo_sensor->sample();
            local_this->open_position->value() = open_pos;
            local_this->servo_upper_limit->value() = reached;
        }
        local_this->servo_control->write(local_this->open_position->value());

        vTaskDelete(NULL);
    }

    void HardStopDamper::setPositions(Position _zero, Position _one)
    {
        if (this->open_at_center) {
            if (this->switch_open_and_close) {
                this->servo_lower_limit->value() = _one.voltage_read;
                this->close_position->value() = _one.comand_position;
            } else {
                this->servo_lower_limit->value() = _zero.voltage_read;
                this->close_position->value() = _zero.comand_position;
            }

            this->servo_upper_limit->value() = _zero.voltage_read + ((_one.voltage_read - _zero.voltage_read) / 2.0);
            this->open_position->value() = _zero.comand_position + ((_one.comand_position - _zero.comand_position) / 2.0);

        } else if (this->switch_open_and_close) {
            this->servo_lower_limit->value() = _one.voltage_read;
            this->close_position->value() = _one.comand_position;

            this->servo_upper_limit->value() = _zero.voltage_read;
            this->open_position->value() = _zero.comand_position;
        } else {
            this->servo_lower_limit->value() = _zero.voltage_read;
            this->close_position->value() = _zero.comand_position;

            this->servo_upper_limit->value() = _one.voltage_read;
            this->open_position->value() = _one.comand_position;
        }
    }

    void HardStopDamper::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Hard_stop_damper reader:");
    }

} // namespace hard_stop_damper
} // namespace esphome
