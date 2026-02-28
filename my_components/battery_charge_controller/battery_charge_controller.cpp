#include "battery_charge_controller.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <driver/ledc.h>

namespace esphome {
namespace charge_controller {

    static const size_t BUFFER_COUNT = 15;

    static const char* const TAG = "Charge_Controller";
    void Charge_Controller::setup()
    {

        xTaskCreate(Charge_Controller::backgroundMonitorTask, "circuit_task", 8192 * 2, (void*)this, 10, &this->task_handle);
        this->lastUpdateTime = micros();
    }

    void Charge_Controller::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Charge_Controller reader:");
        //     LOG_PIN("  Reset pin: ", this->Reset_pin_);
        //     LOG_PIN("  Busy : ", this->Busy_pin_);
    }

    void Charge_Controller::backgroundMonitorTask(void* params)
    {
        Charge_Controller* this_breaker = (Charge_Controller*)params;

        // 5 seconds after startup try to enable all the circuits
        delay(5'000);

        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(50);
        uint32_t lastLoopTime = micros();
        while (true) {
            // auto xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
            delay(50);
            float current = this_breaker->readBatCurrent();
            uint32_t currentLoopTime = micros();
            uint32_t deltaTime = (currentLoopTime - lastLoopTime);
            auto AmpMicroSeconds = current * deltaTime;
            this_breaker->loopAmpMicroSecondsAccumulated += (int32_t)AmpMicroSeconds;

            lastLoopTime = currentLoopTime;
        }
    }

    void Charge_Controller::update()
    {
        uint32_t currentTime = micros();
        int32_t AccumulatedCurrent = this->loopAmpMicroSecondsAccumulated;
        this->loopAmpMicroSecondsAccumulated = 0;
        uint32_t deltaTime = (currentTime - this->lastUpdateTime);
        this->lastUpdateTime = currentTime;

        this->UpdateChargeCurrent(AccumulatedCurrent);

        float V = this->Vin_Sensor->get_state();
        float I = AccumulatedCurrent / (float)deltaTime; // convert ampMicroSeconds to Amps
        float P = V * I;

        if (this->v_bat_display_sensor) {
            this->v_bat_display_sensor->publish_state(V);
        }

        if (this->power_display_sensor) {
            this->power_display_sensor->publish_state(P);
        }

        if (this->i_bat_display_sensor) {
            this->i_bat_display_sensor->publish_state(I);
        }
    }

    void Charge_Controller::UpdateChargeCurrent(int32_t AccumulatedCurrent)
    {

        this->milliAmpSecondsAccumulated += (AccumulatedCurrent / 1000); // convert ampMicroSeconds to milliAmpSeconds



        this->bat_charge_control_channel->write_state(0.5);
    }

    float Charge_Controller::readBatCurrent()
    {
        return (this->Battery_Current_Sensor->sample() - this->Current_zero_point) * this->Current_gain;
    }

} // namespace charge_controller
} // namespace esphome
