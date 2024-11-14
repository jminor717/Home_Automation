#pragma once

#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
// using namespace i2c;

namespace esphome {
namespace dc_relay {
    class Dc_Relay;
    class CircuitConfig;

    class Dc_Relay : public PollingComponent {
    public:
        float get_setup_priority() const override { return setup_priority::HARDWARE; }
        void setup() override;
        // void loop() override;
        void dump_config() override;

        void set_Vin_Sensor(voltage_sampler::VoltageSampler* sen) { this->Vin_Sensor = sen; };
        void set_UVLO(float uvlo) { this->UVLO = uvlo; };
        void set_Voltage_Divider_Ratio(float val) { this->Voltage_Divider_Ratio = val; };
        void set_circuits(std::vector<CircuitConfig*> _circuits) { this->circuits = std::move(_circuits); }
        void update() override;

    protected:
        static const uint64_t UINT_64_MAX = 0xfffffffffffffff; // one byte short because we are adding ~10 seconds worth of uS to this value when the screen is active and dont want an overflow

        voltage_sampler::VoltageSampler* Vin_Sensor;
        std::vector<CircuitConfig*> circuits;
        float UVLO;
        float Voltage_Divider_Ratio;
    };


    class CircuitConfig : public sensor::Sensor {
    public:

        void set_power_sensor(sensor::Sensor* _power_sensor) { this->power_sensor = _power_sensor; }
        void set_current_sensor(sensor::Sensor* _current_sensor) { this->current_sensor = _current_sensor; }

        void set_V_out_sensor(voltage_sampler::VoltageSampler* _V_out_Sensor) { this->V_out_Sensor = _V_out_Sensor; }
        void set_I_out_sensor(voltage_sampler::VoltageSampler* _I_out_Sensor) { this->Current_Sensor = _I_out_Sensor; }

        void set_Enable_pin(InternalGPIOPin* pin) { this->Enable_pin_ = pin; };
        void set_Short_Circuit_Test_pin(InternalGPIOPin* pin) { this->Short_Circuit_Test_pin = pin; };

        float read_power();

    protected:
        InternalGPIOPin* Enable_pin_;
        InternalGPIOPin* Short_Circuit_Test_pin { nullptr };
        voltage_sampler::VoltageSampler* V_out_Sensor;
        voltage_sampler::VoltageSampler* Current_Sensor;

        sensor::Sensor* power_sensor { nullptr };
        sensor::Sensor* current_sensor { nullptr };
    };

} // namespace dc_relay
} // namespace esphome
