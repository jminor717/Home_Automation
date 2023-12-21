#include "EPD_1in9.h"
#include "esphome.h"

using namespace i2c;

#define EPD_RST 19

enum DisplayActions {
    No_Action,
    Reset_Display,
    Write_Screen,
};

enum HighLevelActions {
    No_HL_Action,
    Init,
    Full_Refresh,
};

class SegmentedEPaper : public Component, public i2c::I2CDevice {
    I2CBus* _i2cBus;

    uint64_t startNextStepAt = 0;
    uint64_t startNextActionAt = 0;
    uint64_t eligibleForNextActionAt = 0;
    DisplayActions nextAction = No_Action;
    DisplayActions currentAction = No_Action;
    int8_t nextStepNumber = 0;
    int8_t currentStepNumber = -1;

    HighLevelActions currentHLAction = No_HL_Action;
    int8_t next_HL_StepNumber = 0;
    int8_t current_HL_StepNumber = -1;

    bool actionRunning = false;

    uint8_t VAR_Temperature = 20;

    void StartNextActionIn(DisplayActions next, uint64_t offset_ms)
    {
        if (offset_ms == 0) {
            currentAction = nextAction;
            currentStepNumber = 0;
            actionRunning = true;
        } else {
            startNextActionAt = esp_timer_get_time() + (offset_ms * 1000);
            nextAction = next;
        }
    }

    void StartNextStepIn(uint64_t offset_ms)
    {
        startNextStepAt = esp_timer_get_time() + (offset_ms * 1000);
        nextStepNumber = currentStepNumber + 1;
        currentStepNumber = -1;
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
        currentHLAction = Init;
        current_HL_StepNumber = 0;
    }

    void loop() override
    { // This will be called very often after setup time. think of it as the loop() call in Arduino
        uint64_t currentTime = esp_timer_get_time();
        if (currentTime > startNextActionAt && !actionRunning) {
            StartNextActionIn(nextAction, 0);
        }
        if (currentTime > startNextStepAt) {
            currentStepNumber = nextStepNumber;
        }

        switch (currentHLAction) {
        case Init:
            switch (current_HL_StepNumber) {
            case 0:
                StartNextActionIn(Reset_Display, 0);
                current_HL_StepNumber++;
                break;
            case 1:
                if (!actionRunning) {
                    
                }
                current_HL_StepNumber++;
                break;
            default:
                break;
            }
            break;
        case Full_Refresh:
            /* code */
            break;

        default:
            break;
        }

        switch (currentAction) {
        case No_Action:
            break;
        case Reset_Display: {
            switch (currentStepNumber) {
            case 0:
                digitalWrite(EPD_RST, 1);
                StartNextStepIn(200);
                break;
            case 1:
                digitalWrite(EPD_RST, 0);
                StartNextStepIn(20);
                break;
            case 2:
                digitalWrite(EPD_RST, 1);
                StartNextStepIn(200);
                break;
            case 3:
                actionRunning = false;
                break;
            default:
                break;
            }
            break;
        }
        case Full_Refresh:
            break;
        default:
            break;
        }
    }
};
