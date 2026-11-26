#include "servo_garage_door.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <bitset>
#include <cstring>
#include <driver/ledc.h>
#include <lib/cppcrc.h>


namespace esphome {
namespace servo_garage_door {

    static const char *const TAG = "ServoGarageDoor";
    void ServoGarageDoor::setup()
    {
        for (ServoMotor* motor : this->motors) {
            motor->setup();
        }
        ESP_LOGI(TAG, "Garage door UART controller ready");
    }


    void ServoGarageDoor::loop()
    {

        // ESP_LOGI(TAG, "loop available: %02X", this->available() );
        auto aval = this->available();
        if(aval > 0 && this->looking_for_new_line){
            this->read_log_message(this->current_log_type);
        }
        else if (aval > 0) {
            uint8_t byte;
            if (this->peek_byte(&byte)) {
                // ESP_LOGI(TAG, "read: %d", byte);
                switch (byte)
                {
                case commandType::MOVE_SUCCESS:
                case commandType::COMMAND_ACK:
                case commandType::COMMAND_NAK:
                case commandType::COMMAND_FORMAT_ERROR:
                    read_uart_reply();
                    break;
                case commandType::LOG_MESSAGE:
                case commandType::LOG_WARNING:
                case commandType::LOG_ERROR:
                    this->read_byte(&byte); // remove the command type from the buffer before reading the message
                    this->read_log_message(static_cast<commandType>(byte));
                    return;
                default:
                    // buffer start is not a known command peek byte and read until we find a known command or the buffer is empty
                    this->read_byte(&byte);
                    this->empty_uart_buffer_until_new_command();
                    break;
                }
            }
        }
    }

    void ServoGarageDoor::empty_uart_buffer_until_new_command(){
        size_t avail = this->available();
        while (avail > 0) {
            uint8_t byte;
            if (this->peek_byte(&byte)) {
                if (is_valid_command_type(byte)) {
                    return;
                }
                if (!this->read_byte(&byte)) {
                    return;
                }
                avail--;
                continue;
            }
            return; // return if peek_byte indicates uart buffer is empty
        }
    }

    void ServoGarageDoor::read_uart_reply(){
        auto aval = this->available();
        uint8_t frame[UART_FRAME_SIZE]{};

        if (aval >= UART_FRAME_SIZE)
        {
            this->read_array(frame, UART_FRAME_SIZE);
            switch (frame[0])
            {
            case commandType::COMMAND_ACK:
                ESP_LOGI(TAG, "STM32->: Command ACK received");
                memcpy(this->running_command, this->in_flight_command, UART_FRAME_SIZE);
                std::fill(this->in_flight_command, this->in_flight_command + UART_FRAME_SIZE, 0);
                break;
            case commandType::COMMAND_NAK:
                ESP_LOGI(TAG, "STM32->: Command NAK received");
                std::fill(this->running_command, this->running_command + UART_FRAME_SIZE, 0);
                break;
            case commandType::COMMAND_FORMAT_ERROR:
                ESP_LOGI(TAG, "STM32->: Command format error received");
                std::fill(this->running_command, this->running_command + UART_FRAME_SIZE, 0);
                std::fill(this->in_flight_command, this->in_flight_command + UART_FRAME_SIZE, 0);
                break;
            case commandType::MOVE_SUCCESS:
                ESP_LOGI(TAG, "STM32->: Move success received");
                std::fill(this->running_command, this->running_command + UART_FRAME_SIZE, 0);
                std::fill(this->in_flight_command, this->in_flight_command + UART_FRAME_SIZE, 0);
                break;
            }
        }


    }

    void ServoGarageDoor::read_log_message(commandType log_type){
        this->current_log_type = log_type;
        char buf[512];
        int32_t avail = this->available();
        if(!this->looking_for_new_line){
            // this is a new log message clear the buffer and reset the index
            std::fill(this->log_message_buffer, this->log_message_buffer+sizeof(this->log_message_buffer), 0);
            this->log_message_index = 0;
        }

        this->looking_for_new_line = true;

        while (avail > 0) {
            uint8_t byte;
            if (!this->read_byte(&byte)) {
                ESP_LOGI(TAG, "end of uart buffer before finding new line");
                break;
            }
            avail--;
            if (byte == '\n') {
                this->looking_for_new_line = false;
                break;
            }
            if (this->log_message_index < sizeof(this->log_message_buffer) - 1) {
                this->log_message_buffer[this->log_message_index++] = static_cast<char>(byte);
            }
        }

        this->log_message_buffer[this->log_message_index] = '\0';
        if(!this->looking_for_new_line){
            switch (this->current_log_type)
            {
            case commandType::LOG_MESSAGE:
                ESP_LOGI(TAG, "STM32<-: %s", this->log_message_buffer);
                break;
            case commandType::LOG_WARNING:
                ESP_LOGW(TAG, "STM32<-: %s", this->log_message_buffer);
                break;
            case commandType::LOG_ERROR:
                ESP_LOGE(TAG, "STM32<-: %s", this->log_message_buffer);
                break;
            default:
                ESP_LOGW(TAG, "STM32<-: Unknown log type %d", this->current_log_type);
                break;
            }
        }
    }

    void ServoGarageDoor::write_u32_le_(uint8_t *destination, uint32_t value)
    {
        destination[0] = value & 0xFF;
        destination[1] = (value >> 8) & 0xFF;
        destination[2] = (value >> 16) & 0xFF;
        destination[3] = (value >> 24) & 0xFF;
    }

    void ServoGarageDoor::write_u32_msb_(uint8_t *destination, uint32_t value)
    {
        destination[0] = (value >> 24) & 0xFF;
        destination[1] = (value >> 16) & 0xFF;
        destination[2] = (value >> 8) & 0xFF;
        destination[3] = value & 0xFF;
    }

    uint32_t ServoGarageDoor::crc32_(const uint8_t *data, size_t length) { return CRC32::CRC32::calc(data, length); }

    void ServoGarageDoor::send_uart_frame_(servoMoveDirection direction, uint32_t steps, uint32_t max_speed, uint32_t acceleration, uint8_t interrupt_pin_function)
    {
        uint8_t frame[UART_FRAME_SIZE]{};
        frame[0] = commandType::MOVE_SERVO;
        frame[1] = direction;
        write_u32_le_(frame + 2, steps);
        write_u32_le_(frame + 6, max_speed);
        write_u32_le_(frame + 10, acceleration);
        frame[14] = interrupt_pin_function;
        write_u32_msb_(frame + UART_CRC_OFFSET, crc32_(frame, UART_CRC_OFFSET));

        memcpy(this->in_flight_command, frame, UART_FRAME_SIZE);
        this->write_array(frame, UART_FRAME_SIZE);
    }

    void ServoMotor::setup()
    {

        this->step_pin->setup();
        this->step_pin->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);

        if (this->dir_pin != nullptr) {
            this->dir_pin->setup();
            this->dir_pin->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);
        }

        if (this->enable_pin != nullptr) {
            this->enable_pin->setup();
            this->enable_pin->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLDOWN);
            this->enable_pin->digital_write(0);
        }
    }

    void ServoGarageDoor::move_servo(uint32_t steps, servoMoveDirection direction, uint32_t max_speed, uint32_t acceleration,
                                     uint8_t interrupt_pin_function)
    {
        if (this->running_command[0] == commandType::MOVE_SERVO || !is_valid_direction(direction))
        {
            return;
        }

        this->send_uart_frame_(direction, steps, max_speed, acceleration, interrupt_pin_function);
        ESP_LOGI(TAG, "Sent UART servo command: steps=%lu, direction=%s, max_speed=%lu, acceleration=%lu",
                 static_cast<unsigned long>(steps), direction ? "true" : "false",
                 static_cast<unsigned long>(max_speed), static_cast<unsigned long>(acceleration));

    }

    void ServoGarageDoor::dump_config()
    {
        ESP_LOGCONFIG(TAG, "ServoGarageDoor reader:");
    }

    std::string ServoGarageDoor::format_32bit_binary(uint32_t value) {
        return std::bitset<32>(value).to_string();
    }


} // namespace servo_garage_door
} // namespace esphome
