#pragma once

// #include "esphome/components/button/button.h"
// #include "esphome/components/ledc/ledc_output.h"
// #include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <lib/shared_command_structure.h>
#include <limits>
#include <string>
// using namespace i2c;

namespace esphome {
namespace servo_garage_door {




    class ServoMotor;

    class ServoGarageDoor : public Component, public uart::UARTDevice {
    public:
        float get_setup_priority() const override { return setup_priority::LATE; }
        void setup() override;
        void loop() override;
        void dump_config() override;
        void set_motors(std::vector<ServoMotor*> _motors) { this->motors = std::move(_motors); };

        void move_servo(uint32_t steps, servoMoveDirection direction, uint32_t max_speed, uint32_t acceleration, uint8_t interrupt_pin_function = 0);
        static std::string format_32bit_binary(uint32_t value);
    protected:

        std::vector<ServoMotor*> motors;

        uint8_t in_flight_command[UART_FRAME_SIZE]{};
        uint8_t running_command[UART_FRAME_SIZE]{};


        void send_uart_frame_(servoMoveDirection direction, uint32_t steps, uint32_t max_speed, uint32_t acceleration, uint8_t interrupt_pin_function);
        static uint32_t crc32_(const uint8_t *data, size_t length);
        static void write_u32_le_(uint8_t *destination, uint32_t value);
        static void write_u32_msb_(uint8_t *destination, uint32_t value);
        void read_log_message(commandType log_type);
        void read_uart_reply();
        void empty_uart_buffer_until_new_command();

        QueueHandle_t servo_event_queue;
        TaskHandle_t servo_task_handle { nullptr };

        
        bool looking_for_new_line = false;
        commandType current_log_type;
        char log_message_buffer[512];
        uint16_t log_message_index;
    };


    class ServoMotor : public Component {
    public:
        void setup() override;

        GPIOPin* step_pin; // min pulse width 2us max frequency 300KHz
        GPIOPin* dir_pin; // min pulse width 2us max frequency 300KHz
        GPIOPin* enable_pin { nullptr }; // min pulse width 100us max frequency 10KHz
        GPIOPin* alarm_pin { nullptr }; // open drain output, active low
        GPIOPin* in_place_pin { nullptr }; // open drain output, active low
        uint32_t steps_per_rotation;
        
        void set_Enable_pin(GPIOPin* pin) { this->enable_pin = pin; };
        void set_Step_pin(GPIOPin* pin) { this->step_pin = pin; };
        void set_Dir_pin(GPIOPin* pin) { this->dir_pin = pin; };
        void set_Alarm_pin(GPIOPin* pin) { this->alarm_pin = pin; };
        void set_In_Place_pin(GPIOPin* pin) { this->in_place_pin = pin; };
        void set_Steps_Per_Rotation(uint32_t steps) { this->steps_per_rotation = steps; };

    };

} // namespace servo_garage_door
} // namespace esphome
