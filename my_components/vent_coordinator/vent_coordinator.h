#pragma once

#include "esphome.h"
// #include "esphome/components/button/button.h"
// #include "esphome/components/ledc/ledc_output.h"
#include "esphome/components/sensor/sensor.h"
// #include "esphome/components/switch/switch.h"
// #include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/components/cover/cover.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
#include <vector>
#include "../packet_transport/packet_transport.h"

// using namespace i2c;

namespace esphome {
namespace vent_coordinator {


    class VentConfig {
    public:
        // setters for code gen
        void set_Cover_Position_Sensor(sensor::Sensor* sen) { this->positionSensor = sen; }
        void set_Cover(cover::Cover* cover) { this->cover = cover; this->Remote = false; };
        void set_Remote_Provider(const char* rem) { this->remote_provider_name = rem; this->Remote = true; };
        void set_Local_Vent_ID(uint16_t id) { this->local_id = id; }
        void set_Remote_ID(uint16_t id) { this->remote_id = id; }
        // required, can be a local sensor or remote one
        // use sensor instead of cover.make_call().get_position() so that local and remote are consistent
        sensor::Sensor* positionSensor;
        bool Remote {false};

        // if remote, use provider name to send commands to open and close the vent
        const char* remote_provider_name {""};
        uint16_t remote_id {0};

        // if local comand cover directly
        cover::Cover* cover { nullptr };
        uint16_t local_id {0};
    protected:

    };

    class Vent_Coordinator : public PollingComponent {
    public:
        float get_setup_priority() const override { return setup_priority::DATA; }
        void setup() override;
        void loop() override;
        void update() override;
        void dump_config() override;

        // setters for code gen
        void set_transport(packet_transport::PacketTransport* t) { this->transport = t; }
        void set_vents(std::vector<VentConfig*> vents) { this->vents = std::move(vents); }

        
    protected:
        packet_transport::PacketTransport* transport;
        std::vector<VentConfig*> vents;

        // sensor::Sensor* Vin_Sensor;
    };
} // namespace vent_coordinator
} // namespace esphome
