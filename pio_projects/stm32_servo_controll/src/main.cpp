#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "cppcrc.h"
#include "shared_command_structure.h"

/*
  TODO: 
    implement uart commands
    fault states:
      stepper fault state
      garage door object present sensor triggered
    fault recovery
      if moving down- decelerate quickly, and move back up one inch above when fault occurs
      if moving up- decelerate quickly

*/




void moveServoFloat(uint32_t total_steps, uint16_t max_speed, uint16_t acceleration, uint16_t min_speed);
void moveServoMicros(const servoMoveDirection direction, const uint32_t total_steps, const uint32_t max_speed, const uint32_t acceleration, const uint16_t min_speed);
void ParseMoveCommand(const uint8_t* frame);

const uint16_t MIN_SPEED = 40; // steps per second

const int step_pin = PB5;
const int dir_pin = PB4;
const int enable_pin = PB3;

const int LED_PIN = PC13; 


// auto uart = Uart(PA3, PA2); // working
auto uart = Uart(PA3, PA2);


void sendSerialLog(const char* message) {
  uart.write(commandType::LOG_MESSAGE);
  uart.println(String(message));
}
void sendSerialLog(String message) { sendSerialLog(message.c_str()); }

void sendSerialWarning(const char* message) {
  uart.write(commandType::LOG_WARNING);
  uart.println(String(message));
}
void sendSerialWarning(String message) { sendSerialWarning(message.c_str()); }


void write_u32_msb_(uint8_t *destination, uint32_t value)
{
    destination[0] = (value >> 24) & 0xFF;
    destination[1] = (value >> 16) & 0xFF;
    destination[2] = (value >> 8) & 0xFF;
    destination[3] = value & 0xFF;
}

void sendCommandResponse(commandType resp) {
  uint8_t frame[UART_FRAME_SIZE]{};
  frame[0] = resp;
  write_u32_msb_(frame + UART_CRC_OFFSET, CRC32::CRC32::calc(frame, UART_CRC_OFFSET));
  uart.write(frame, UART_FRAME_SIZE);
}


bool isValidUartFrame(const uint8_t* frame) {
  const uint32_t received_crc =
      (static_cast<uint32_t>(frame[UART_CRC_OFFSET]) << 24) |
      (static_cast<uint32_t>(frame[UART_CRC_OFFSET + 1]) << 16) |
      (static_cast<uint32_t>(frame[UART_CRC_OFFSET + 2]) << 8) |
      static_cast<uint32_t>(frame[UART_CRC_OFFSET + 3]);
  const uint32_t calculated_crc = CRC32::CRC32::calc(frame, UART_CRC_OFFSET);
  if(calculated_crc != received_crc){
    sendSerialWarning("received " + String(received_crc, HEX) + " calculated " + String(calculated_crc, HEX));
  }

  return calculated_crc == received_crc;
}

void writeU32LittleEndian(uint8_t* destination, uint32_t value) {
  destination[0] = value & 0xFF;
  destination[1] = (value >> 8) & 0xFF;
  destination[2] = (value >> 16) & 0xFF;
  destination[3] = (value >> 24) & 0xFF;
}

void clearUartBuffer() {
  String warning_message = "Clearing UART buffer, available bytes: " + String(uart.available()) + " read bytes: ";
  while (uart.available() > 0) {
    warning_message+= String(uart.read())+", ";
  }
  sendSerialWarning(warning_message);
}

void readUartFrames() {
  static uint8_t frame[UART_FRAME_SIZE];
  static size_t byte_count = 0;

  if (uart.available() < UART_FRAME_SIZE) {
    // Not enough bytes available to form a complete frame, return and wait for more data
    return;
  }
  
  while (uart.available() >= 0) {
    const int value = uart.read();
    if (value < 0) {
      return;
    }

    frame[byte_count++] = static_cast<uint8_t>(value);
    if (byte_count >= UART_FRAME_SIZE) {
      uint8_t command_byte = frame[0];

      if (isValidUartFrame(frame)) {
        if(!is_valid_command_type(command_byte)){
          sendSerialWarning("invalid command type: " + String(command_byte));
        }
        switch (command_byte) {
          case commandType::MOVE_SERVO:
            ParseMoveCommand(frame);
            break;
          case commandType::HOME_SERVO:
            moveServoMicros(servoMoveDirection::DOWN, 1000, 400, 400, MIN_SPEED);
            break;
          default:
            // no action or response needed for other command types
            break;
        }

      }
      byte_count = 0;
      if (uart.available() > 0 && uart.available() < UART_FRAME_SIZE) {
        // sendSerialWarning("extra bytes in UART buffer, clearing buffer");
        clearUartBuffer();
      }
    }
  }
}

void ParseMoveCommand (const uint8_t* frame) {
  uint8_t dir_byte = frame[1];
  uint8_t interrupt_byte = frame[14];
  if(!is_valid_direction(dir_byte) || !is_valid_interrupt(interrupt_byte)){
    sendSerialWarning("invalid direction or interrupt byte");
    return;
  }
  const uint32_t steps =
      (static_cast<uint32_t>(frame[5]) << 24) |
      (static_cast<uint32_t>(frame[4]) << 16) |
      (static_cast<uint32_t>(frame[3]) << 8) |
      static_cast<uint32_t>(frame[2]);
  const uint32_t max_speed =
      (static_cast<uint32_t>(frame[9]) << 24) |
      (static_cast<uint32_t>(frame[8]) << 16) |
      (static_cast<uint32_t>(frame[7]) << 8) |
      static_cast<uint32_t>(frame[6]);
  const uint32_t acceleration =
      (static_cast<uint32_t>(frame[13]) << 24) |
      (static_cast<uint32_t>(frame[12]) << 16) |
      (static_cast<uint32_t>(frame[11]) << 8) |
      static_cast<uint32_t>(frame[10]);
  const servoMoveDirection direction = static_cast<servoMoveDirection>(dir_byte);
  const interruptPinFunction interrupt_pin_function = static_cast<interruptPinFunction>(interrupt_byte);

  sendCommandResponse(commandType::COMMAND_ACK);
  sendSerialLog("move received steps: " + String(steps) + ", speed: " + String(max_speed) + ", acc: " + String(acceleration) + ", dir: " + String(direction == servoMoveDirection::DOWN ? "DOWN" : "UP"));
  (void) interrupt_pin_function;
  moveServoMicros(direction, steps, max_speed, acceleration, MIN_SPEED);
  sendCommandResponse(commandType::MOVE_SUCCESS);

}

void setup() {
  // Serial.begin(38400);
  uart.begin(38400);
  pinMode(enable_pin, OUTPUT);
  // flash the enable pin to reset the stepper driver
  digitalWrite(enable_pin, HIGH);
  delay(50);
  digitalWrite(enable_pin, LOW);
  pinMode(step_pin, OUTPUT);
  digitalWrite(step_pin, LOW);
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);



  delay(90);
}
int MHZ = 84000000 / 1000000;
void localDelayMicroseconds(uint32_t us)
{
  int32_t start  = dwt_getCycles();
  int32_t cycles = us * (MHZ);

  while ((int32_t)dwt_getCycles() - start < cycles);
}

void loop() {
  readUartFrames();
  delay(0);


  // Serial.println("loop");
  // moveServoMicros(80000, 18000, 8953, 40);
  // moveServoMicros(10000, 18000, 20000, 40);
  // uint32_t max = 1000;
  // for (unsigned int i = 0; i < max; i++)
  // {
  //   digitalWrite(LED_PIN, HIGH);
  //   localDelayMicroseconds(max+2-i);
  //   digitalWrite(LED_PIN, LOW);
  //   localDelayMicroseconds(i);
  // }






  // uint32_t max = 1000;
  // for (unsigned int i = 0; i < max; i++)
  // {
  //   digitalWrite(LED_PIN, HIGH);
  //   localDelayMicroseconds(max+2-i);
  //   digitalWrite(LED_PIN, LOW);
  //   localDelayMicroseconds(i);
  // }
  
  // for (unsigned int i = max; i > 2; i--)
  // {
  //   digitalWrite(LED_PIN, HIGH);
  //   localDelayMicroseconds(max+2-i);
  //   digitalWrite(LED_PIN, LOW);
  //   localDelayMicroseconds(i);
  // }

  // digitalWrite(LED_PIN, HIGH);
  // delay(5000);

  //       total_steps, max_speed, acceleration, min_speed
  // moveServoFloat(10000, 18000, 6000, 40);
  // moveServoFloat(80000, 18000, 8953, 40);






}


void moveServoMicros(const servoMoveDirection direction, const uint32_t total_steps, const uint32_t max_speed, const uint32_t acceleration, const uint16_t min_speed){
  digitalWrite(enable_pin, HIGH);
  digitalWrite(dir_pin, direction == servoMoveDirection::DOWN ? HIGH : LOW);
  // driver needs a delay after enabling before it will accept step pulses
  delay(1000);


  // fixed point shift limited by   1000000ULL << fixed_point_shift
  const uint8_t fixed_point_shift = 8;

  // Fixed-point speed in steps/sec 
  const uint32_t min_speed_fixed = ((uint32_t)min_speed << fixed_point_shift);
  const uint32_t max_speed_fixed = ((uint32_t)max_speed << fixed_point_shift);
  // acceleration_fixed needs to be divided by 1,000,000 to keep as much precision as possible split this into first 
  // deleting the constant by 1953 then after multiplying by the last period right shift by 9, 
  // this cumulatively results in dividing by 999,936 while reducing the number of divisions that need to be done in the loop
  // >> 6 and / 15,625 would result in dividing by exactly 1,000,000 but would result in a lower minium acceleration
  // current minimum acceleration is ~ 100 before resolution begins being lost
  const uint32_t acceleration_fixed = ((uint32_t)acceleration << fixed_point_shift) / 1953;


  uint32_t current_speed = min_speed_fixed;
  uint32_t last_period_us = 1000000ULL / min_speed;
  uint32_t acceleration_steps = 0;
  uint32_t acceleration_time_us = 0;
  bool is_accelerating = true;
  const uint8_t step_pulse_width_us = 5;
  uint8_t period_overrun_count = 0;

  for (uint32_t step_index = 0; step_index < total_steps; ++step_index) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(step_pin, HIGH);
    localDelayMicroseconds(step_pulse_width_us);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(step_pin, LOW);
    unsigned long computeStart = micros();

    // 
    if (is_accelerating && current_speed < max_speed_fixed && step_index < ((total_steps >> 1) - 2)) {
      current_speed += (uint32_t)(acceleration_fixed * last_period_us)  >> 9; // += delta_speed
      acceleration_steps = step_index;
      acceleration_time_us += last_period_us;

      if (current_speed > max_speed_fixed) { current_speed = max_speed_fixed; }

    } else if (step_index >= total_steps - acceleration_steps) {
      current_speed -= (uint32_t)(acceleration_fixed * last_period_us) >> 9; // -= delta_speed

      if (current_speed < min_speed_fixed) { current_speed = min_speed_fixed; }
    } else {
      is_accelerating = false;
    }

    uint32_t period_us = (1000000ULL << fixed_point_shift) / current_speed;
    last_period_us = period_us;

    unsigned long computeFinished = micros();

    if(computeFinished - computeStart > period_us - (step_pulse_width_us * 2)) {
      period_overrun_count++;
    }else{
      period_us -= (computeFinished - computeStart);
      localDelayMicroseconds(period_us - step_pulse_width_us);
    }

  }

  delay(300);
  digitalWrite(enable_pin, LOW);
}

void moveServoFloat(uint32_t total_steps, uint16_t max_speed, uint16_t acceleration, uint16_t min_speed){
  digitalWrite(enable_pin, HIGH);
  delay(1000);

  const float acceleration_float = acceleration / 1000000.0f;

  float current_speed = (float)min_speed;
  uint32_t lastPeriod = (int32_t)((1.0f / current_speed) * 1000000.0f);
  uint32_t accelerationSteps = 0;
  uint32_t accelerationTime = 0;
  bool isAccelerating = true;
  const uint8_t step_pulse_width_us = 5;
  uint8_t PeriodOverrunCount = 0;
  for (uint32_t step_index = 0; step_index < total_steps; ++step_index) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(step_pin, HIGH);
    localDelayMicroseconds(step_pulse_width_us);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(step_pin, LOW);
    unsigned long computeStart = micros();

    if (isAccelerating && current_speed < (float)max_speed && step_index < ((total_steps >> 1) - 2)) {
      current_speed = current_speed + (acceleration_float * lastPeriod) ;
      accelerationSteps = step_index;
      accelerationTime += lastPeriod;

      if (current_speed > max_speed) { current_speed = max_speed; }

    } else if (step_index >= total_steps - accelerationSteps) {
      current_speed = current_speed - (acceleration_float * lastPeriod);

      if(current_speed < min_speed) { current_speed = min_speed; }
    } else {
      isAccelerating = false;
    }




    uint32_t period = (uint32_t)((1.0f / current_speed) * 1000000.0f);
    lastPeriod = period;

    unsigned long computeFinished = micros();

    if(computeFinished - computeStart > period - (step_pulse_width_us * 2)) {
      PeriodOverrunCount++;
    }else{
      period -= (computeFinished - computeStart);
      localDelayMicroseconds(period - step_pulse_width_us);
      // delay((period - step_pulse_width_us)/1000);
    }
  }
  delay(300);
  digitalWrite(enable_pin, LOW);
}