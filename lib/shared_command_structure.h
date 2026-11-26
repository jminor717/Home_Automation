/*
packet structures: 32 bytes
move servo
    MOVE_SERVO (1 byte)
    direction (1 byte)
    steps (4 bytes)
    max_speed (4 bytes)
    acceleration (4 bytes)
    interruptPinFunction (1 byte)
    NULLS (13 bytes)
    CRC (4 bytes)
home servo
    HOME_SERVO (1 byte)
    NULLS (31 bytes)

*/

#ifndef COMMON_COMMANDS_H_
#define COMMON_COMMANDS_H_

#include <cstdint>

static constexpr size_t UART_FRAME_SIZE = 32;
static constexpr size_t UART_CRC_OFFSET = 28;

enum commandType : uint8_t {
    MOVE_SERVO = 1,
    HOME_SERVO = 2,
    MOVE_SUCCESS = 50,
    COMMAND_ACK = 100,
    COMMAND_NAK = 101,
    COMMAND_FORMAT_ERROR = 102,
    LOG_MESSAGE = 253,
    LOG_WARNING = 254,
    LOG_ERROR = 255
};

constexpr bool is_valid_command_type(uint8_t value) {
    switch (value) {
    case commandType::MOVE_SERVO:
    case commandType::HOME_SERVO:
    case commandType::MOVE_SUCCESS:
    case commandType::COMMAND_ACK:
    case commandType::COMMAND_NAK:
    case commandType::COMMAND_FORMAT_ERROR:
    case commandType::LOG_MESSAGE:
    case commandType::LOG_WARNING:
    case commandType::LOG_ERROR:
        return true;
    default:
        return false;
    }
}


enum servoMoveDirection : uint8_t {
    UP = 0,
    DOWN = 1,
};

constexpr bool is_valid_direction(uint8_t value) {
    switch (value) {
    case servoMoveDirection::UP:
    case servoMoveDirection::DOWN:
        return true;
    default:
        return false;
    }
}

enum interruptPinFunction : uint8_t {
    SLOW_STOP = 0,
    FAST_STOP = 1,
    EMERGENCY_STOP = 2,
};

constexpr bool is_valid_interrupt(uint8_t value) {
    switch (value) {
    case interruptPinFunction::SLOW_STOP:
    case interruptPinFunction::FAST_STOP:
    case interruptPinFunction::EMERGENCY_STOP:
        return true;
    default:
        return false;
    }
}

#endif