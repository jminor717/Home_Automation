#pragma once

#include "esphome.h"
// #include "esphome/components/button/button.h"
#include "esphome/components/ledc/ledc_output.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
// using namespace i2c;

namespace esphome {
namespace dc_relay {
    struct ChangeStateEvent {
        bool newState;
        uint8_t circuit_id;
        // size_t len;
        // uint8_t data[BUFFER_SIZE];
    };

    class Dc_Relay;
    class CircuitConfig;
    class customLEDCOutput : public ledc::LEDCOutput {
    public:
        uint8_t get_channel() { return this->channel_; }
        uint8_t get_pinNum() { return this->pin_->get_pin(); }
    };


    class Dc_Relay : public PollingComponent {
    public:
        float get_setup_priority() const override { return setup_priority::DATA; }
        void setup() override;
        // void loop() override;
        void dump_config() override;

        // setters for code gen
        void set_Vin_Sensor(voltage_sampler::VoltageSampler* sen) { this->Vin_Sensor = sen; };
        void set_UVLO(float uvlo)
        {
            // assumes a 5% hysteresis for coming out of UVLO
            this->UVLO = uvlo;
            this->Hysteresis = uvlo * 0.05;
        };
        void set_Voltage_Divider_Ratio(float val) { this->Voltage_Divider_Ratio = val; };
        void set_Current_Calibration(float val) { this->Current_Calibration = val; };
        void set_circuits(std::vector<CircuitConfig*> _circuits) { this->circuits = std::move(_circuits); }
        void set_Short_Circuit_Test_Chanel(ledc::LEDCOutput* chan) {
            this->SC_Test_Chanel = static_cast<customLEDCOutput*>(chan);
        };

        void update() override;

        void stopIfNecessary();
        void startIfAble();

        QueueHandle_t circuit_event_queue;
        float UVLO;
        float Hysteresis;
        float V_Open_Circuit = 48.0;
        float Voltage_Divider_Ratio;
        float Current_Calibration;

    protected:
        voltage_sampler::VoltageSampler* Vin_Sensor;
        customLEDCOutput* SC_Test_Chanel;
        std::vector<CircuitConfig*> circuits;

        bool inLockOut;
        bool inLockOutRecovery;
        bool stopped;

        TaskHandle_t circuit_task_handle { nullptr };
        static void backgroundCircuitMonitorTask(void* params);
    };

    class CircuitEnable : public switch_::Switch, public Parented<CircuitConfig> {
    protected:
        void write_state(bool state) override;
    };

    class CircuitConfig : public sensor::Sensor, public Parented<Dc_Relay> {
    public:
        float read_power();
        void Circuit_Enable(bool state);
        void Circuit_State_Changed(bool state);
        void Do_State_Change(bool state);
        void setup();

        // setters for code gen
        void set_power_sensor(sensor::Sensor* _power_sensor) { this->power_sensor = _power_sensor; }
        void set_current_sensor(sensor::Sensor* _current_sensor) { this->current_sensor = _current_sensor; }

        void set_V_out_sensor(voltage_sampler::VoltageSampler* _V_out_Sensor) { this->V_out_Sensor = _V_out_Sensor; }
        void set_I_out_sensor(voltage_sampler::VoltageSampler* _I_out_Sensor) { this->Current_Sensor = _I_out_Sensor; }

        void set_Enable_pin(InternalGPIOPin* pin) { this->Enable_pin_ = pin; };
        void set_Short_Circuit_Test_pin(InternalGPIOPin* pin) { this->Short_Circuit_Test_pin = pin; };
        void set_Enable_Circuit_Switch(switch_::Switch* _switch_) { this->Enable_Circuit_switch = _switch_; };

        uint8_t id;
        customLEDCOutput* SC_Test_Chanel;

    protected:
        voltage_sampler::VoltageSampler* V_out_Sensor;
        voltage_sampler::VoltageSampler* Current_Sensor;
        InternalGPIOPin* Enable_pin_;
        InternalGPIOPin* Short_Circuit_Test_pin { nullptr };
        sensor::Sensor* power_sensor { nullptr };
        sensor::Sensor* current_sensor { nullptr };
        switch_::Switch* Enable_Circuit_switch;
        float I_Max = 5.0;
        float I_SC_Test_Max = 1.2;
    };
} // namespace dc_relay
} // namespace esphome
