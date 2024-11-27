#include "dc_relay.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dc_relay {

    // can have at most 5 circuits on 3sp32-s3 plus plenty of extra room for future expansion
    static const size_t BUFFER_COUNT = 15;

    static const char* const TAG = "Dc_Relay";

    void Dc_Relay::setup()
    {
        this->inLockOut = false;
        this->inLockOutRecovery = false;
        uint8_t i = 0;
        for (CircuitConfig* circuit : this->circuits) {
            circuit->setup();
            circuit->id = i;
            i++;
        }

        this->circuit_event_queue = xQueueCreate(BUFFER_COUNT, sizeof(ChangeStateEvent));
        xTaskCreate(Dc_Relay::backgroundCircuitMonitorTask, "circuit_task", 8192, (void*)this, 0, &this->circuit_task_handle);
    }

    void CircuitConfig::setup()
    {
        // this->V_out_Sensor->stop_poller();
        // this->Current_Sensor->stop_poller();
        this->Enable_pin_->setup();
        this->Enable_pin_->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);
        if (this->Short_Circuit_Test_pin) {
            this->Short_Circuit_Test_pin->setup();
            this->Short_Circuit_Test_pin->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);
        }
    }

    void Dc_Relay::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Dc_Relay reader:");
        //     LOG_PIN("  Reset pin: ", this->Reset_pin_);
        //     LOG_PIN("  Busy : ", this->Busy_pin_);
    }

    void Dc_Relay::backgroundCircuitMonitorTask(void* params)
    {
        Dc_Relay* this_breaker = (Dc_Relay*)params;
        // delayMicroseconds
        // delay
        while (true) {
            ChangeStateEvent event;
            if (xQueueReceive(this_breaker->circuit_event_queue, &event, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                // stop poling while running a circuit change events
                this_breaker->stopIfNecessary();
                CircuitConfig* circuitToChange;
                bool found = false;
                for (CircuitConfig* circuit : this_breaker->circuits) {
                    if (circuit->id == event.circuit_id) {
                        circuitToChange = circuit;
                        found = true;
                        break;
                    }
                }
                if (found) {
                    circuitToChange->Do_State_Change(event.newState);
                } else {
                    ESP_LOGW(TAG, "could not find circuit from event data, expected id:%d", event.circuit_id);
                }

                this_breaker->startIfAble();
            }
        }
    }
    void Dc_Relay::stopIfNecessary()
    {
        if (!this->stopped) {
            this->stop_poller();
            this->stopped = true;
        }
    }
    void Dc_Relay::startIfAble()
    {
        // if the circuit_event_queue is empty re-start polling
        uint8_t numEventsWaiting = uxQueueMessagesWaiting(this->circuit_event_queue);
        if (this->stopped && numEventsWaiting == 0) {
            this->start_poller();
            this->stopped = false;
        }
    }

    void Dc_Relay::update()
    {
        float V = this->Vin_Sensor->sample() * this->Voltage_Divider_Ratio;
        if (this->UVLO > 0){
            if (V < this->UVLO && !this->inLockOut && !this->inLockOutRecovery) {
                this->inLockOut = true;
                ESP_LOGI(TAG, "entering UVLO voltage:%f", V);
                for (CircuitConfig* circuit : this->circuits) {
                    circuit->Circuit_Enable(false);
                }
            }

            // if the input voltage has recovered and all of the circuits have been disabled then re-enable them
            if (this->inLockOut && V > this->UVLO + this->Hysteresis && uxQueueMessagesWaiting(this->circuit_event_queue) == 0) {
                this->inLockOutRecovery = true;
                ESP_LOGI(TAG, "recovering from UVLO voltage:%f", V);
                for (CircuitConfig* circuit : this->circuits) {
                    circuit->Circuit_Enable(true);
                }
            }

            // if lock out recovery has completed return to the default state
            if (this->inLockOut && this->inLockOutRecovery && uxQueueMessagesWaiting(this->circuit_event_queue) == 0) {
                ESP_LOGV(TAG, "recovered from UVLO");
                this->inLockOut = false;
                this->inLockOutRecovery = false;
            }

            if (this->inLockOut) {
                return;
            }
        }


        ESP_LOGV(TAG, "read input voltage #%f", V);
        for (CircuitConfig* circuit : this->circuits) {
            circuit->read_power();
        }
    }

    float CircuitConfig::read_power()
    {
        float V = this->V_out_Sensor->sample() * this->parent_->Voltage_Divider_Ratio;
        float I = this->Current_Sensor->sample() * this->parent_->Current_Calibration;
        float P = V * I;

        if (this->power_sensor) {
            this->power_sensor->publish_state(P);
        }

        if (this->current_sensor) {
            this->current_sensor->publish_state(I);
        }

        return P;
    }

    void CircuitConfig::Do_State_Change(bool state)
    {
        if (!state) {
            if (this->Short_Circuit_Test_pin)
                this->Short_Circuit_Test_pin->digital_write(0);
            this->Enable_pin_->digital_write(0);
        } else if (this->Short_Circuit_Test_pin) {
            float V = this->V_out_Sensor->sample() * this->parent_->Voltage_Divider_Ratio;

            //TODO implement short circuit detection
            this->Enable_pin_->digital_write(1);
            this->Short_Circuit_Test_pin->digital_write(0);

            state = V >= this->parent_->UVLO;
        } else {
            this->Enable_pin_->digital_write(1);
        }
        this->Circuit_State_Changed(state);
    }

    void CircuitConfig::Circuit_State_Changed(bool state)
    {
        this->Enable_Circuit_switch->publish_state(state);
    }

    void CircuitConfig::Circuit_Enable(bool state)
    {
        ChangeStateEvent data;
        data.newState = state;
        data.circuit_id = this->id;
        xQueueSendToBack(this->parent_->circuit_event_queue, &data, portMAX_DELAY);
        // xQueueSendToFront()
    }

    // CircuitEnable switch, from UI
    void CircuitEnable::write_state(bool state) { this->parent_->Circuit_Enable(state); }
} // namespace dc_relay
} // namespace esphome
