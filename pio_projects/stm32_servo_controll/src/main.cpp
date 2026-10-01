#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// put function declarations here:
void moveServoFloat(uint32_t total_steps, uint16_t max_speed, uint16_t acceleration, uint16_t min_speed);
void moveServoMicros(uint32_t total_steps, uint16_t max_speed, uint16_t acceleration, uint16_t min_speed);

const int step_pin = PB5;
const int dir_pin = PB4;
const int enable_pin = PB3;

#define SCK_PIN   5  // D13 = pin19 = PortB.5
#define MISO_PIN  6  // D12 = pin18 = PortB.4
#define MOSI_PIN  7  // D11 = pin17 = PortB.3
#define SS_PIN    4  // D10 = pin16 = PortB.2
const int LED_PIN = PC13; 

volatile bool rx = false;

void SPI_ISR() {
  rx = true;
}

// SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCK_PIN, SS_PIN);
// HardwareSerial Serial1();
// HardwareSerial Serial2(); 



byte RxByte;

void I2C_RxHandler(int numBytes)
{
  while(Wire.available()) {  // Read Any Received Data
    RxByte = Wire.read();
    Wire.write(0b11001010);
    Wire.write(RxByte);
  }
}


void setup() {
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);
  delay(50);
  digitalWrite(enable_pin, LOW);


  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);  // (only if bidirectional mode needed)
  pinMode(SS_PIN, INPUT);

  // spi.begin(SPI_PERIPHERAL);
  // spi.attachSlaveInterrupt(SS_PIN, SPI_ISR); // Add an ISR to the SS pin to detect device selection by SPI master
  Wire.begin(0x55); // Initialize I2C (Slave Mode: address=0x55 )
  Wire.onReceive(I2C_RxHandler);

  pinMode(step_pin, OUTPUT);
  digitalWrite(step_pin, LOW);
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, LOW);
  
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(9000);
}
int MHZ = 84000000 / 1000000;
void localDelayMicroseconds(uint32_t us)
{
  int32_t start  = dwt_getCycles();
  int32_t cycles = us * (MHZ);

  while ((int32_t)dwt_getCycles() - start < cycles);
}

void loop() {

  if (rx) {
    // Use the various SPI.transfer() functions to read and write data
    rx = false;
    // SPI_MODE0 == CPOL=0 && CPHA=0
    auto set = SPISettings(200000, MSBFIRST, SPI_MODE0, SPI_PERIPHERAL);
    spi.beginTransaction(set);
    uint8_t bits = spi.transfer(0b0);
    uint8_t bits2 = spi.transfer(0b11111111);
    uint8_t second = spi.transfer(0b11111111);
    uint8_t second2 = spi.transfer(bits);
    spi.endTransaction();
    // Serial.println("Hello World!");

      uint32_t max = 1000;
    for (unsigned int i = 0; i < max; i++)
    {
      digitalWrite(LED_PIN, HIGH);
      localDelayMicroseconds(max+2-i);
      digitalWrite(LED_PIN, LOW);
      localDelayMicroseconds(i);
    }
  }

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



  // delay(500);
  // moveServoMicros(80000, 18000, 8953, 40);
  // moveServoMicros(10000, 18000, 20000, 40);



}


void moveServoMicros(uint32_t total_steps, uint16_t max_speed, uint16_t acceleration, uint16_t min_speed){
  digitalWrite(enable_pin, HIGH);
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