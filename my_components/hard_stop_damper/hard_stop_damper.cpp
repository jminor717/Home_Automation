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
            if (abs(reachedPosition - lastReached) < abs(increment * 0.5)) {
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
        if (!local_this->homed) {
            // if this is the first home from setup wait 15 seconds for everything else to be done
            delay(15'000);
        }
        local_this->v_servo_sensor->stop_poller();
        local_this->v_servo_sensor->set_update_interval(250);
        local_this->v_servo_sensor->start_poller();

        local_this->homed = false;
        delay(2000);
        auto _zero = local_this->move_to_hard_stop(-0.025, 0.5, 0, 1);
        ESP_LOGI(TAG, "stop 0: %f, %f", _zero.voltage_read, _zero.comand_position);
        auto _one = local_this->move_to_hard_stop(+0.025, 0.5, 0, 1);
        ESP_LOGI(TAG, "stop 1: %f, %f", _one.voltage_read, _one.comand_position);
        local_this->setPositions(_zero, _one);

        local_this->setOffsets();

        local_this->servo_control->write(local_this->open_position);
        local_this->homed = true;

        local_this->v_servo_sensor->stop_poller();
        local_this->v_servo_sensor->set_update_interval(60'000);
        local_this->v_servo_sensor->start_poller();

        vTaskDelete(NULL);
    }


    void HardStopDamper::setOffsets(){
        if (abs(this->close_offset) > 0.02) {
            float close_pos = this->close_position + this->close_offset;
            ESP_LOGI(TAG, "find close offset, initial: %f, offset: %f, new: %f", this->close_position, this->close_offset, close_pos);
            this->servo_control->write(close_pos);
            delay(2000);
            auto reached = this->v_servo_sensor->sample();
            ESP_LOGI(TAG, "found close offset, old: %f, new: %f", this->lower_limit, reached);
            this->close_position = close_pos;
            this->lower_limit = reached;
        }

        if (abs(this->open_offset) > 0.02) {
            float open_pos = this->open_position + this->open_offset;
            ESP_LOGI(TAG, "find open offset, initial: %f, offset: %f, new: %f", this->open_position, this->open_offset, open_pos);
            this->servo_control->write(open_pos);
            delay(2000);
            auto reached = this->v_servo_sensor->sample();
            ESP_LOGI(TAG, "found open offset, old: %f, new: %f", this->upper_limit, reached);
            this->open_position = open_pos;
            this->upper_limit = reached;
        }
    }

    void HardStopDamper::setPositions(Position _zero, Position _one)
    {
        if (this->open_at_center) {
            if (this->switch_open_and_close) {
                this->lower_limit = _one.voltage_read;
                this->close_position = _one.comand_position;
            } else {
                this->lower_limit = _zero.voltage_read;
                this->close_position = _zero.comand_position;
            }

            this->upper_limit = _zero.voltage_read + ((_one.voltage_read - _zero.voltage_read) / 2.0);
            this->open_position = _zero.comand_position + ((_one.comand_position - _zero.comand_position) / 2.0);

        } else if (this->switch_open_and_close) {
            this->lower_limit = _one.voltage_read;
            this->close_position = _one.comand_position;

            this->upper_limit = _zero.voltage_read;
            this->open_position = _zero.comand_position;
        } else {
            this->lower_limit = _zero.voltage_read;
            this->close_position = _zero.comand_position;

            this->upper_limit = _one.voltage_read;
            this->open_position = _one.comand_position;
        }
    }

    void HardStopDamper::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Hard_stop_damper reader:");
    }

} // namespace hard_stop_damper
} // namespace esphome
