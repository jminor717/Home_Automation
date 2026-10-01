#include <Arduino.h>
// #include <AccelStepper.h>

// put function declarations here:
// AccelStepper stepper(AccelStepper::DRIVER, 6, 7);
uint32_t position = 0;
uint32_t step_pin = 6;
uint32_t dir_pin = 7;
uint32_t enable_pin = 8;

void setup() {

  // put your setup code here, to run once:
  // pinMode(LED_BUILTIN, OUTPUT);
  pinMode(step_pin, OUTPUT);
  digitalWrite(step_pin, LOW);
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, LOW);
  pinMode(enable_pin, OUTPUT);
  
  
  digitalWrite(enable_pin, HIGH);
  delay(1000);
  digitalWrite(enable_pin, LOW);

  Serial.begin(9600);
  Serial.println("Hello World!");

  // stepper.setMaxSpeed(16000);
  // stepper.setAcceleration(8000);
  // // stepper.setEnablePin(8);
  // stepper.setMinPulseWidth(5);
  // stepper.setPinsInverted(false, false, true);
  
  delay(9000);

}


void moveServoMicros(){
  digitalWrite(enable_pin, HIGH);
  delay(1000);

  const uint32_t total_steps = 80000;
  const uint16_t max_speed = 16000;
  const uint16_t acceleration = 8953;
  const uint16_t min_speed = 75;
  const uint8_t fixed_point_shift = 8;

  // Fixed-point speed in steps/sec using Q16.16.
  const uint32_t min_speed_fixed = ((uint32_t)min_speed << fixed_point_shift);
  const uint32_t max_speed_fixed = ((uint32_t)max_speed << fixed_point_shift);
  const uint32_t acceleration_fixed = ((uint32_t)acceleration << fixed_point_shift) / 7812;

  // const uint32_t total_steps = 1000;
  // const uint32_t max_speed = 450;
  // const uint32_t acceleration = 250;
  // const uint32_t min_speed = 75;

  Serial.print("start move div micros, acc:");
  Serial.print(acceleration_fixed);
  Serial.print(" min:");
  Serial.print(min_speed_fixed);
  Serial.print(" max:");
  Serial.println(max_speed_fixed);

  uint32_t current_speed = min_speed_fixed;
  uint32_t last_period_us = 1000000ULL / min_speed;
  // uint32_t last_period_us = (1000000ULL << fixed_point_shift) / current_speed;
  // 13,333
  uint32_t acceleration_steps = 0;
  uint32_t acceleration_time_us = 0;
  bool is_accelerating = true;
  const uint8_t step_pulse_width_us = 10;
  uint8_t period_overrun_count = 0;

  for (uint32_t step_index = 0; step_index < total_steps; ++step_index) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(step_pulse_width_us);
    digitalWrite(step_pin, LOW);
    unsigned long computeStart = micros();

    if (is_accelerating && current_speed < max_speed_fixed) {
      const uint32_t delta_speed = (acceleration_fixed * last_period_us)  >> 7; // 20,479
      current_speed += delta_speed;
      if (current_speed > max_speed_fixed) {
        current_speed = max_speed_fixed;
      }
      acceleration_steps = step_index;
      acceleration_time_us += last_period_us;
    } else if (step_index >= total_steps - acceleration_steps) {
      const uint32_t delta_speed = (acceleration_fixed * last_period_us) >> 7;
      current_speed -= delta_speed;
      if (current_speed < min_speed_fixed) {
        current_speed = min_speed_fixed;
      }
    } else {
      is_accelerating = false;
    }

    if (current_speed < min_speed_fixed) {
      current_speed = min_speed_fixed;
    }

    uint32_t period_us = (1000000ULL << fixed_point_shift) / current_speed;
    last_period_us = period_us;

    unsigned long computeFinished = micros();

    if(computeFinished - computeStart > period_us - (step_pulse_width_us * 2)) {
      period_overrun_count++;
    }else{
      period_us -= (computeFinished - computeStart);
      delayMicroseconds(period_us - step_pulse_width_us);
    }

  }

  digitalWrite(enable_pin, LOW);
  Serial.print("Acceleration Steps: ");
  Serial.println(acceleration_steps);

  Serial.print("Acceleration Time: ");
  Serial.println(acceleration_time_us);

  Serial.print("PeriodOverrun count: ");
  Serial.println(period_overrun_count);
}

void moveServoFloat(){
  digitalWrite(enable_pin, HIGH);
  delay(1000);

  const uint32_t total_steps = 80000;
  const uint16_t max_speed = 12000;
  const float acceleration = 5953 / 1000000.0f;
  const uint16_t min_speed = 75;

  // const uint32_t total_steps = 1000;
  // const uint32_t max_speed = 450;
  // const uint32_t acceleration = 250;
  // const uint32_t min_speed = 75;

  Serial.println("start move float");

  float current_speed = (float)min_speed;
  uint32_t lastPeriod = (int32_t)((1.0f / current_speed) * 1000000.0f);
  uint32_t accelerationSteps = 0;
  uint32_t accelerationTime = 0;
  bool isAccelerating = true;
  const uint8_t step_pulse_width_us = 5;
  uint8_t PeriodOverrunCount = 0;
  for (uint32_t step_index = 0; step_index < total_steps; ++step_index) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(step_pulse_width_us);
    digitalWrite(step_pin, LOW);
    unsigned long computeStart = micros();

    if (isAccelerating && current_speed < (float)max_speed) {
      current_speed = current_speed + (acceleration * lastPeriod) ;
      accelerationSteps = step_index;
      accelerationTime += lastPeriod;
    } else if (step_index >= total_steps - accelerationSteps) {
      current_speed = current_speed - (acceleration * lastPeriod);
    } else {
      isAccelerating = false;
    }

    if(current_speed < min_speed) {
        current_speed = min_speed;
    }


    uint32_t period = (uint32_t)((1.0f / current_speed) * 1000000.0f);
    lastPeriod = period;

    unsigned long computeFinished = micros();

    if(computeFinished - computeStart > period - (step_pulse_width_us * 2)) {
      PeriodOverrunCount++;
    }else{
      period -= (computeFinished - computeStart);
      delayMicroseconds(period - step_pulse_width_us);
    }
  }

  digitalWrite(enable_pin, LOW);
  Serial.print("Acceleration Steps float: ");
  Serial.println(accelerationSteps);

  Serial.print("Acceleration Time float: ");
  Serial.println(accelerationTime);

  Serial.print("PeriodOverrun count float: ");
  Serial.println(PeriodOverrunCount);
}

void loop() {

  // digitalWrite(LED_BUILTIN, LOW);
  // delay(100);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(100);


  // digitalWrite(8, HIGH);
  // delay(1000);
  // Serial.println("start move to " + String(1200));
  // stepper.runToNewPosition(80000);
  // // Serial.println("finished move to " + String(1200));
  // Serial.println("finished move to " + String(1200));
  // delay(100);
  // Serial.println("start move to " + String(2400));
  // stepper.runToNewPosition(0);
  // Serial.println("finished move to " + String(2400));
  // digitalWrite(8, LOW);
  // delay(10000);

  moveServoMicros();

  delay(1000);

  moveServoFloat();

  delay(10000);
}
