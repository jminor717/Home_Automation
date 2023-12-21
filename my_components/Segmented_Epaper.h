#include "EPD_1in9.h"
#include "esphome.h"

using namespace i2c;

class SegmentedEPaper : public Component, public i2c::I2CDevice {
  I2CBus *_i2cBus;

public:
  SegmentedEPaper(I2CBus *Bus) { _i2cBus = Bus; }

  void setup() override {
    // This will be called once to set up the component
    // think of it as the setup() call in Arduino
    // set_i2c_address(0x44);
    pinMode(18, INPUT);
    pinMode(19, OUTPUT);
    EPD_1in9_init();

    EPD_1in9_lut_5S();
    EPD_1in9_Write_Screen(DSPNUM_1in9_off);
    delay(500);

    EPD_1in9_lut_GC();

    EPD_1in9_Write_Screen1(DSPNUM_1in9_on);
    delay(500);

    EPD_1in9_Write_Screen(DSPNUM_1in9_off);
    delay(500);

    
  }
  void loop() override {
    // This will be called very often after setup time.
    // think of it as the loop() call in Arduino

  }
};
