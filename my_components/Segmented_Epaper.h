#include "EPD_1in9.h"
#include "esphome.h"

using namespace i2c;

#define EPD_RST 19
static const char* const TAG = "Segmented_E_paper";

typedef void (*callback_function)(I2CBus* bussin);

struct DisplayAction {
    callback_function function; // Function pointer
    uint16_t delayAfter; // time to wait after this action before the next is performed
};

uint8_t Current_EPD_Temperature_Compensation = 25;

void EPD_Reset_1(I2CBus* bussin) { digitalWrite(EPD_RST, 1); }
void EPD_Reset_2(I2CBus* bussin) { digitalWrite(EPD_RST, 0); }
void EPD_Reset_3(I2CBus* bussin) { digitalWrite(EPD_RST, 1); }
void EPD_PowerOn(I2CBus* bussin) { bussin->write(adds_com, new uint8_t[1] { 0x2B }, 1); } // POWER_ON
void EPD_Boost(I2CBus* bussin) { bussin->write(adds_com, new uint8_t[2] { 0xA7, 0xE0 }, 2); } // boost, TSON
void EPD_Temperature1(I2CBus* bussin)
{
    bussin->write(adds_com, new uint8_t[3] { 0x7E, 0x81, 0xB4 }, 3);
}
void EPD_Temperature2(I2CBus* bussin)
{
    uint8_t data[] = { 0xe7, 0 }; // Set default frame time
    if (Current_EPD_Temperature_Compensation < 5)
        data[1] = 0x31; // 0x31  (49+1)*20ms=1000ms
    else if (Current_EPD_Temperature_Compensation < 10)
        data[1] = 0x22; // 0x22  (34+1)*20ms=700ms
    else if (Current_EPD_Temperature_Compensation < 15)
        data[1] = 0x18; // 0x18  (24+1)*20ms=500ms
    else if (Current_EPD_Temperature_Compensation < 20)
        data[1] = 0x13; // 0x13  (19+1)*20ms=400ms
    else
        data[1] = 0x0e; // 0x0e  (14+1)*20ms=300ms
    bussin->write(adds_com, data, 2);
}

void AsyncDelay(I2CBus* bussin) { }

class SegmentedEPaper : public Component, public i2c::I2CDevice {
    I2CBus* _i2cBus;

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

    void Reset_Display(void)
    {
        AddAction(EPD_Reset_1, 200);
        AddAction(EPD_Reset_2, 20);
        AddAction(EPD_Reset_3, 200);
    }
    void Init_Display(void)
    {
        ESP_LOGD(TAG, "HEY ##### ###### #### Display init started");
        Reset_Display();
        AddAction(EPD_PowerOn, 10);
        AddAction(EPD_Boost, 10);
        CompensateForTemperature(25);
    }

    void CompensateForTemperature(uint8_t currentTemp)
    {
        ESP_LOGD(TAG, "temp Compensation %d", currentTemp);
        Current_EPD_Temperature_Compensation = currentTemp;
        AddAction(EPD_Temperature1, 10);
        AddAction(EPD_Temperature2, 10);
    }

    void setup() override
    { // This will be called once to set up the component think of it as the setup() call in Arduino
        // set_i2c_address(0x44);
        pinMode(18, INPUT);
        pinMode(EPD_RST, OUTPUT);
        Init_Display();
    }

    void loop() override
    { // This will be called very often after setup time. think of it as the loop() call in Arduino
        uint64_t currentTime = esp_timer_get_time();
        if (queueLength > 0 && currentTime > canRunNextActionAt) {
            DisplayAction runningAction = actionQueue[queueIndex];
            ESP_LOGD(TAG, "running action %d at %d, %d remaining, next action in %d", queueIndex, (currentTime / 1000), queueLength, runningAction.delayAfter);
            runningAction.function(_i2cBus);
            canRunNextActionAt = esp_timer_get_time() + (runningAction.delayAfter * 1000);
            queueLength--;
            queueIndex = (queueIndex + 1) & queueModulus;
        }
    }
};
