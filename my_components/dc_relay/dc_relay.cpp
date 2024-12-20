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
        xTaskCreate(Dc_Relay::backgroundCircuitMonitorTask, "circuit_task", 8192 * 2, (void*)this, 0, &this->circuit_task_handle);
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
        if (this->UVLO > 0) {
            if (V < this->UVLO && !this->inLockOut && !this->inLockOutRecovery) {
                this->inLockOut = true;
                ESP_LOGI(TAG, "entering UVLO voltage:%f", V);
                for (CircuitConfig* circuit : this->circuits) {
                    circuit->Circuit_Enable(false);
                }
                return;
            }

            // if the input voltage has recovered and all of the circuits have been disabled then re-enable them
            if (this->inLockOut && V > (this->UVLO + this->Hysteresis) && uxQueueMessagesWaiting(this->circuit_event_queue) == 0) {
                this->inLockOutRecovery = true;
                ESP_LOGI(TAG, "recovering from UVLO voltage:%f", V);
                for (CircuitConfig* circuit : this->circuits) {
                    circuit->Circuit_Enable(true);
                }
                return;
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
            // float V = this->V_out_Sensor->sample() * this->parent_->Voltage_Divider_Ratio;

            // increase the duty through the short circuit buck converter while monitoring output current and voltage
            // look at the current and voltage to figure out if there is a short, or a failing component that is causing a breakdown at higher voltage
            const uint8_t maxDuty = 75;
            float V[maxDuty * 2] = { 0.0 };
            float I[maxDuty * 2] = { 0.0 };
            float R[maxDuty * 2] = { 0.0 };
            float I_Interpolated[maxDuty * 2] = { 0.0 };
            float Sum_I_Interpolated = 0;
            float EMA_I[maxDuty * 2] = { 0.0 };
            float Delta_I[maxDuty * 2] = { 0.0 };
            float EMA_alpha = 2.0 / (5.0 + 1.0); // exponential moving average over last ~5 datapoints
            uint8_t i = 0;
            for (float duty = 0; duty < maxDuty; duty += 0.5) {
                // TODO: set duty v = ir
                // wait for the output to settle
                delay(1);
                float V_sum = 0, I_sum = 0;
                for (uint8_t aver = 0; aver < 10; aver++) {
                    V_sum += this->V_out_Sensor->sample() * this->parent_->Voltage_Divider_Ratio;
                    I_sum += this->Current_Sensor->sample() * this->parent_->Current_Calibration;
                }

                V[i] = V_sum / 10.0;
                I[i] = I_sum / 10.0;
                R[i] = V[i] / I[i];

                // calculate what the current would be at the open circuit voltage based on the current voltage and current
                I_Interpolated[i] = this->parent_->V_Open_Circuit / R[i];
                Sum_I_Interpolated += I_Interpolated[i];

                if (i == 0) {
                    EMA_I[i] = I[i];
                } else {
                    EMA_I[i] = (EMA_alpha * I[i]) + ((1.0 - EMA_alpha) * EMA_I[i - 1]);
                    // EMA = α*current + (1 − α) * EMA yesterday
                    Delta_I[i] = EMA_I[i] - EMA_I[i - 1];
                }

                if (I[i] > this->I_SC_Test_Max) {
                    break;
                }
                i++;
            }

            bool shortCircuit = I[i] > this->I_Max;
            // this is the best guess at what the load current would be assuming a resistive load
            // the circuit is considered tripped if this current is greater than 85% of the max current to add safety factor since this is an estimation
            float Avg_I_Interpolated = Sum_I_Interpolated / (float)i;
            if (Avg_I_Interpolated > this->I_Max * 0.85) {
                ESP_LOGW(TAG, "short circuit test found extrapolated current of: %f, greater than 85%% of %f", Avg_I_Interpolated, this->I_Max);
                shortCircuit = true;
            }

            float sum_delta_i = 0;
            for (uint8_t tp = 0; tp < i; tp++) {
                sum_delta_i += Delta_I[tp];
                float average_delta = sum_delta_i / tp;
                if (tp > 5 && Delta_I[tp] > average_delta * 1.20 && I_Interpolated[tp] > this->I_Max * 1.20) {
                    // Delta_I can be interprited as resistance assuming delta V is stable
                    // so if Delta_I starts rapidly increasing it could indicate a breakdown at some voltage
                    // a plot of I over V in this scenario will have a 'knee' at some voltage
                    ESP_LOGW(TAG, "short circuit test found a potential breakdown at %f volts and %f amps; delta_I: %f; max_I: %f; extrapolated current: %f", V[tp], I[tp], Delta_I[tp], I[i], I_Interpolated[tp]);
                    shortCircuit = true;
                    break;
                }
            }

            // TODO disable pwm
            this->Enable_pin_->digital_write(!shortCircuit);
            this->Short_Circuit_Test_pin->digital_write(0);

            state = !shortCircuit;
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
