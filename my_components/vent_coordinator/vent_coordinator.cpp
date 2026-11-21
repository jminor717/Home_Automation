#include "vent_coordinator.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
// #include <driver/ledc.h>

namespace esphome {
namespace vent_coordinator {
    static const char* const TAG = "Vent_Coordinator";

    void Vent_Coordinator::setup()
    {

        // this->circuit_event_queue = xQueueCreate(BUFFER_COUNT, sizeof(ChangeStateEvent));
        // xTaskCreate(Dc_Relay::backgroundCircuitMonitorTask, "circuit_task", 8192 * 2, (void*)this, 2, &this->circuit_task_handle);
    }

    // void Dc_Relay::backgroundCircuitMonitorTask(void* params)
    // {
    //     Dc_Relay* this_breaker = (Dc_Relay*)params;

    // }

    
    void Vent_Coordinator::loop()
    {
        // component loop, called frequently
    }

    void Vent_Coordinator::update()
    {
        // polling component, called at update_interval rate
    }

    void Vent_Coordinator::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Vent_Coordinator reader:");
        //     LOG_PIN("  Reset pin: ", this->Reset_pin_);
        //     LOG_PIN("  Busy : ", this->Busy_pin_);
    }
} // namespace vent_coordinator
} // namespace esphome
