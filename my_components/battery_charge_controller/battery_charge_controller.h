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
namespace charge_controller {

    class Charge_Controller;
    // class customLEDCOutput : public ledc::LEDCOutput {
    // public:
    //     uint8_t get_channel() { return this->channel_; }
    //     uint8_t get_pinNum() { return this->pin_->get_pin(); }
    // };

    class Charge_Controller : public PollingComponent {
    public:
        float get_setup_priority() const override { return setup_priority::DATA; }
        void setup() override;
        // void loop() override;
        void dump_config() override;
        void update() override;

        // setters for code gen
        void set_Vin_Sensor(voltage_sampler::VoltageSampler* sen) { this->Vin_Sensor = sen; };
        void set_current_Sensor(voltage_sampler::VoltageSampler* sen) { this->Current_Sensor = sen; };
        void set_OCV(float vOC) { this->V_Open_Circuit = vOC; };

        void set_Voltage_Divider_Ratio(float val) { this->Voltage_Divider_Ratio = val; };
        void set_current_zero_point_voltage(float val) { this->Current_zero_point = val; };
        void set_current_gain(float val) { this->Current_gain = val; };
        void set_voltage_display(sensor::Sensor* _voltage_sensor) { this->v_bat_display_sensor = _voltage_sensor; }
        void set_current_display(sensor::Sensor* _current_sensor) { this->i_bat_display_sensor = _current_sensor; }

        float readBatCurrent();

        // QueueHandle_t circuit_event_queue;
        float UVLO;
        float Hysteresis;
        float V_Open_Circuit;
        float Voltage_Divider_Ratio;
        float Current_zero_point;
        float Current_gain;

    protected:
        voltage_sampler::VoltageSampler* Vin_Sensor;
        voltage_sampler::VoltageSampler* Current_Sensor;
        // customLEDCOutput* SC_Test_Chanel;
        sensor::Sensor* v_bat_display_sensor { nullptr };
        sensor::Sensor* i_bat_display_sensor { nullptr };

        uint32_t milliAmpSecondsAccumulated { 0 };
        uint32_t loopAmpMicroSecondsAccumulated { 0 };

        uint32_t lastUpdateTime { 0 };

        TaskHandle_t task_handle { nullptr };
        static void backgroundMonitorTask(void* params);
    };
} // namespace charge_controller
} // namespace esphome
