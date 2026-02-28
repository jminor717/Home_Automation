#include "hard_stop_damper.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hard_stop_damper {
    static const char* const TAG = "Hard_stop_damper";
    // the flowing 2 look up tables comprise the output from the blow equation and its inverse
    // they are used to convert between a flow percentage and an angle to comand the damper to and vice versa
    // this allows the UI to command a flow percentage and abstract away the angle the damper needs to move to to achieve that flow

    // pipeArea and diameter cancel eachother out
    //  area = (tilt / 100) * pipeArea
    //  cosA = -((area / ((pi * diameter**2)/4)) - 1) * cosA0
    //  angle = math.degrees( math.acos(cosA))
    //  cmd = (angle / 90) * 100
    /**
     * @brief convert flow percentage to damper angle,
     * assumes damper close position is 2° offset from 90° to prevent binding also assumes the damper has a minimal thickness
     */
    // clang-format off
    const float flow_compensation_LUT[] = {
        0.0000, 0.0928, 0.1294, 0.1579, 0.1820, 0.2033, 0.2227, 0.2406, 0.2573, 0.2730,
        0.2879, 0.3022, 0.3158, 0.3289, 0.3416, 0.3538, 0.3657, 0.3773, 0.3885, 0.3995,
        0.4102, 0.4207, 0.4309, 0.4410, 0.4509, 0.4605, 0.4701, 0.4795, 0.4887, 0.4978,
        0.5067, 0.5156, 0.5243, 0.5329, 0.5415, 0.5499, 0.5582, 0.5664, 0.5746, 0.5826,
        0.5906, 0.5985, 0.6064, 0.6142, 0.6219, 0.6295, 0.6371, 0.6446, 0.6521, 0.6595,
        0.6669, 0.6742, 0.6815, 0.6887, 0.6959, 0.7030, 0.7101, 0.7172, 0.7242, 0.7312,
        0.7382, 0.7451, 0.7520, 0.7589, 0.7657, 0.7725, 0.7793, 0.7860, 0.7928, 0.7995,
        0.8061, 0.8128, 0.8194, 0.8261, 0.8327, 0.8392, 0.8458, 0.8523, 0.8589, 0.8654,
        0.8719, 0.8784, 0.8849, 0.8913, 0.8978, 0.9042, 0.9106, 0.9171, 0.9235, 0.9299,
        0.9363, 0.9427, 0.9490, 0.9554, 0.9618, 0.9682, 0.9745, 0.9809, 0.9873, 0.99
    };

    /**
     * @brief revers of the above operation to calculate a percent flow given a measured angle
     */
    const float revers_LUT[] = { 
        0.0000, 0.0000, 0.0015, 0.0027, 0.0040, 0.0056, 0.0075, 0.0095, 0.0118, 0.0144,
        0.0171, 0.0201, 0.0233, 0.0268, 0.0305, 0.0344, 0.0385, 0.0429, 0.0475, 0.0523,
        0.0574, 0.0626, 0.0681, 0.0738, 0.0798, 0.0859, 0.0923, 0.0988, 0.1056, 0.1126,
        0.1198, 0.1272, 0.1348, 0.1427, 0.1507, 0.1589, 0.1673, 0.1760, 0.1848, 0.1938,
        0.2030, 0.2124, 0.2219, 0.2317, 0.2416, 0.2517, 0.2620, 0.2725, 0.2831, 0.2939,
        0.3049, 0.3161, 0.3274, 0.3388, 0.3504, 0.3622, 0.3741, 0.3862, 0.3984, 0.4107,
        0.4232, 0.4358, 0.4486, 0.4615, 0.4745, 0.4876, 0.5009, 0.5143, 0.5278, 0.5414,
        0.5551, 0.5689, 0.5828, 0.5968, 0.6110, 0.6252, 0.6395, 0.6538, 0.6683, 0.6828,
        0.6974, 0.7121, 0.7269, 0.7417, 0.7566, 0.7715, 0.7865, 0.8015, 0.8166, 0.8318,
        0.8469, 0.8621, 0.8774, 0.8926, 0.9079, 0.9232, 0.9386, 0.9539, 0.9693, 0.9846,
    };
    // clang-format on

    void HardStopDamper::setup()
    {
        if (this->homing)
        {
            return; // if we are already homing from a previous setup call, don't start another one
        }
        this->homing = true;
        xTaskCreate(HardStopDamper::find_hard_stops, "hard_stop_task", 8192 * 2, (void*)this, 0, &this->task_handle);
    }

    float HardStopDamper::tilt_to_servo_position(float tilt)
    {
        uint16_t tilt_index = std::min((int)(tilt * 100), (int)(sizeof(flow_compensation_LUT) / sizeof(flow_compensation_LUT[0])) - 1);
        return remap(flow_compensation_LUT[tilt_index], float(0), float(1), this->close_position, this->open_position);
    }
    float HardStopDamper::get_tilt()
    {
        // remap damper position to the percentage flow at that angle
        int reverse_index = remap(this->v_servo_sensor->state, this->upper_limit, this->lower_limit, float(1), float(0)) * 100;
        reverse_index = std::min(reverse_index, (int)(sizeof(revers_LUT) / sizeof(revers_LUT[0])) - 1);
        return revers_LUT[reverse_index];
    }
    float HardStopDamper::get_cover_state()
    {
        return this->v_servo_sensor->state < (this->lower_limit + float(0.1)) ? esphome::cover::COVER_CLOSED : esphome::cover::COVER_OPEN;
    }

    Position HardStopDamper::move_to_hard_stop(float increment, float startingPosition, float lowerLimit, float upperLimit)
    {
        this->servo_control->internal_write(startingPosition);
        delay(1000);
        this->servo_control->internal_write(startingPosition);
        delay(1000);
        this->servo_control->internal_write(startingPosition);
        delay(1000);

        std::deque<Position> position_queue = {};
        std::deque<PositionVariance> position_variance_queue = {};
        float reachedPosition = this->v_servo_sensor->sample();
        float commandedPosition = startingPosition;
        while (true) {
            commandedPosition += increment;
            if (commandedPosition > upperLimit || commandedPosition < lowerLimit) {
                ESP_LOGW(TAG, "commanded position (%.3f) out of bounds before hard stop reached", commandedPosition);
                break;
            }
            this->servo_control->internal_write(commandedPosition);
            delay(200);
            // sample the position 10 times and average it to get a more stable reading
            int count = 0;
            for (size_t i = 0; i < 10; i++)
            {
                reachedPosition += this->v_servo_sensor->sample();
                count++;
                delay(1);
            }
            reachedPosition = reachedPosition / (float)count;

            position_queue.push_back({commandedPosition, reachedPosition});

            // calculate the relation between reachedPosition and commandedPosition using the position_queue
            float m, c;
            this->fitLine(position_queue, m, c);
            float predictedPosition = (commandedPosition * m) + c;
            ESP_LOGI(TAG, "comand: %.3f, reached: %.3f, predicted: %.3f, diff: %.3f, m: %.3f, c: %.3f", commandedPosition, reachedPosition, predictedPosition, abs(reachedPosition - predictedPosition), m, c);

            position_variance_queue.push_back({commandedPosition, reachedPosition, predictedPosition, abs(reachedPosition - predictedPosition), m, c});

            if (position_queue.size() > 9) {
                bool hard_stop_reached =
                position_variance_queue[(position_variance_queue.size() - 1) - 0].diff > 0.02 &&
                position_variance_queue[(position_variance_queue.size() - 1) - 1].diff > 0.02 &&
                position_variance_queue[(position_variance_queue.size() - 1) - 2].diff > 0.02;
                if (hard_stop_reached) {
                    auto pos = position_variance_queue[(position_variance_queue.size() - 1) - 2];
                    return { pos.comand_position, pos.voltage_read };
                }
            }
        }

        return { commandedPosition - increment, reachedPosition };
    }

    void HardStopDamper::find_hard_stops(void* params)
    {
        HardStopDamper* local_this = (HardStopDamper*)params;
        if (!local_this->homed) {
            // if this is the first home from setup wait 15 seconds for everything else to be done
            delay(15'000);
        }

        local_this->homed = false;
        local_this->servo_control->write(0.5);
        delay(2000);
        auto _zero = local_this->move_to_hard_stop(-0.01, 0.5, 0, 1);
        ESP_LOGI(TAG, "stop 0: %f, %f", _zero.voltage_read, _zero.comand_position);
        auto _one = local_this->move_to_hard_stop(+0.01, 0.5, 0, 1);
        ESP_LOGI(TAG, "stop 1: %f, %f", _one.voltage_read, _one.comand_position);
        local_this->setPositions(_zero, _one);

        local_this->setOffsets();

        local_this->servo_control->write(local_this->open_position);

        delay(8'000);
        local_this->servo_control->write(local_this->tilt_to_servo_position(1.0));

        local_this->homed = true;
        local_this->homing = false;
        vTaskDelete(NULL);
    }

    void HardStopDamper::fitLine(const std::deque<Position>& points, float& m, float& c)
    {
        int n = points.size();
        if (n < 2)
            return; // Need at least two points to fit a line

        float sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;

        for (const auto& point : points) {
            sum_x += point.comand_position;
            sum_y += point.voltage_read;
            sum_xy += point.comand_position * point.voltage_read;
            sum_x2 += point.comand_position * point.comand_position;
        }

        float denominator = (n * sum_x2 - sum_x * sum_x);

        if (std::abs(denominator) > 1e-9) { // Avoid division by zero
            m = (n * sum_xy - sum_x * sum_y) / denominator;
            c = (sum_y - m * sum_x) / n;
        } else {
            // Handle vertical line case or all points have same x
            m = std::numeric_limits<float>::infinity();
            c = std::numeric_limits<float>::quiet_NaN();
        }
    }

    void HardStopDamper::setOffsets()
    {
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
