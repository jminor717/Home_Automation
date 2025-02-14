#include "timed_traversal_motor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace timed_traversal_motor {
    static const char* const TAG = "Timed_Traversal_Motor";

    void TimedTraversalMotor::setup()
    {
        // this->find_hard_stops();
        // xTaskCreate(TimedTraversalMotor::find_hard_stops, "hard_stop_task", 8192 * 2, (void*)this, 0, &this->task_handle);
    }

    void TimedTraversalMotor::dump_config()
    {
        ESP_LOGCONFIG(TAG, "timed_traversal_motor reader:");
    }

} // namespace timed_traversal_motor
} // namespace esphome
