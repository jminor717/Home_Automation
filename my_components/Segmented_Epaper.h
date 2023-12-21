#include "EPD_1in9.h"
#include "esphome.h"

using namespace i2c;

#define EPD_RST 19

typedef void (*callback_function)(void);

struct DisplayAction {
    callback_function function; // Function pointer
    uint16_t delayAfter; // time to wait after this action before the next is performed
};

void init_Display(void)
{
}

class SegmentedEPaper : public Component, public i2c::I2CDevice {
    I2CBus* _i2cBus;
    uint8_t VAR_Temperature = 20;
    DisplayAction actionQueue[32];
    uint8_t queueIndex = 0;
    uint8_t queueHead = 0;
    uint8_t queueLength = 0;
    uint8_t queueModulus = 0b00011111;

    uint64_t canRunNextActionAt = 0;

    void AddAction(callback_function action, uint16_t delay)
    {
        if (queueLength >= 31) {
            // bork
            return;
        }
        actionQueue[queueHead] = { action, delay };
        queueHead = (queueHead + 1) & queueModulus;
        queueLength++;
    }

public:
    SegmentedEPaper(I2CBus* Bus) { _i2cBus = Bus; }

    void setup() override
    {
        // This will be called once to set up the component
        // think of it as the setup() call in Arduino
        // set_i2c_address(0x44);
        pinMode(18, INPUT);
        pinMode(EPD_RST, OUTPUT);
        AddAction(init_Display, 200);
        AddAction(init_Display, 200);
    }

    void loop() override
    { // This will be called very often after setup time. think of it as the loop() call in Arduino
        uint64_t currentTime = esp_timer_get_time();
        if (queueLength > 0 && canRunNextActionAt > currentTime) {

            DisplayAction runningAction = actionQueue[queueIndex];
            runningAction.function();
            canRunNextActionAt = esp_timer_get_time() + (runningAction.delayAfter * 1000);
            queueLength--;
            queueIndex = (queueIndex + 1) & queueModulus;
        }
    }
};
