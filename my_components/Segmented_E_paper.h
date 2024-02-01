//#include "EPD_1in9.h"
#include "esphome.h"
using namespace i2c;

#define adds_com 0x3C
#define adds_data 0x3D

#define EPD_RST 7
#define EPD_BUSY 8

#define UINT_64_MAX 0xfffffffffffffff // one byte short because we are adding ~10 seconds worth of uS to this value when the screen is active and dont want an overflow
#define UPDATES_BETWEEN_REFRESH 20
const uint64_t TimeBetweenFullUpdates = 60 * 1000 * 1000; // 2 minute timeout

static const char* const TAG = "Segmented_E_paper";

// clang-format off
unsigned char DSPNUM_1in9_on[]   = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,      };  // all black
unsigned char DSPNUM_1in9_off[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      };  // all white
unsigned char DSPNUM_1in9_W3[]   = {0x00, 0xf5, 0x1f, 0xf5, 0x1f, 0xf5, 0x1f, 0xf5, 0x1f, 0xf5, 0x1f, 0xf5, 0x1f, 0x00, 0x00, 0x00,      };  // 3
#define TEMP_TENS 1
#define TEMP_ONES 3
#define TEMP_TENTHS 11

#define HUMID_TENS 5
#define HUMID_ONES 7
#define HUMID_TENTHS 9

uint8_t SegmentNumbers[][2] = {
    {0xbf, 0x1f}, // 0
    {0x00, 0x1f}, // 1
    {0xfd, 0x17}, // 2
    {0xf5, 0x1f}, // 3
    {0x47, 0x1f}, // 4
    {0xf7, 0x1d}, // 5
    {0xff, 0x1d}, // 6
    {0x21, 0x1f}, // 7
    {0xff, 0x1f}, // 8
    {0xf7, 0x1f}, // 9
} ;
// clang-format on

typedef void (*callback_function)(I2CBus* bussin);

struct DisplayAction {
    callback_function function; // Function pointer
    uint16_t delayAfter; // time to wait after this action before the next is performed
    uint16_t actionId;
};

#define InitId 22

// 8 frame deep circular buffer, one frame is 15 bytes plus one for some unknown purpose taken from the exapmle driver code
uint8_t ScreenBuffer[8][16];
uint8_t ScreenBufferIndex = 0;
uint8_t ScreenBufferHead = 0;
uint8_t ScreenBufferLength = 0;
const uint8_t ScreenBufferModulus = 0b111;

uint8_t CurrentDisplay[16] = { 0 };

bool updatingDisplay = false;
bool initting = false;
uint8_t Current_EPD_Temperature_Compensation = 25;

esp32::ESP32InternalGPIOPin* Busy_pin_obj;
esp32::ESP32InternalGPIOPin* Rst_pin_obj;

void EPD_RST_ON(I2CBus* bussin) { /*digitalWrite(EPD_RST, 1);*/ }
void EPD_RST_OFF(I2CBus* bussin) { /*digitalWrite(EPD_RST, 0);*/ }
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
    initting = false;
}
/* 5 waveform better ghosting, Boot waveform EPD_1in9_lut_5S*/
void EPD_ClearScreen_Brush1(I2CBus* bussin) { bussin->write(adds_com, new uint8_t[7] { 0x82, 0x28, 0x20, 0xA8, 0xA0, 0x50, 0x65 }, 7); }
/* GC waveform, The brush waveform EPD_1in9_lut_GC*/
void EPD_ClearScreen_Brush2(I2CBus* bussin) { bussin->write(adds_com, new uint8_t[7] { 0x82, 0x20, 0x00, 0xA0, 0x80, 0x40, 0x63 }, 7); }
/*  DU waveform white extinction diagram + black out diagram, Bureau of brush waveform EPD_1in9_lut_DU_WB*/
void EPD_Default_Brush(I2CBus* bussin) { bussin->write(adds_com, new uint8_t[7] { 0x82, 0x80, 0x00, 0xC0, 0x80, 0x80, 0x62 }, 7); }

void EPD_Screen_Sleep(I2CBus* bussin) { bussin->write(adds_com, new uint8_t[3] { 0xAE, 0x28, 0xAD }, 3); } // display off, HV OFF, sleep in

// grab the oldest frame from the buffer and write it to the display
void EPD_Write_Screen(I2CBus* bussin)
{
    bussin->write(adds_com, new uint8_t[5] { 0xAC, 0x2B, 0x40, 0xA9, 0xA8 }, 5); // Close the sleep, turn on the power,  Write RAM address, Turn on the first SRAM, Shut down the first SRAM

    // ESP_LOGD(TAG, "WRITE:   %d,%d,%d,%d,%d,%d,%d,%d,%d", dispNumber, dispVal[0], dispVal[1], dispVal[2], dispVal[3], dispVal[4], dispVal[5], dispVal[6], dispVal[7], dispVal[8]);
    updatingDisplay = false;
    bussin->write(adds_data, ScreenBuffer[ScreenBufferIndex], 16);
    memcpy(CurrentDisplay, ScreenBuffer[ScreenBufferHead], sizeof(uint8_t) * 16);

    ScreenBufferLength--;
    ScreenBufferIndex = (ScreenBufferIndex + 1) & ScreenBufferModulus;

    bussin->write(adds_com, new uint8_t[3] { 0xAB, 0xAA, 0xAF }, 3); // Turn on the second SRAM, Shut down the second SRAM, display on
}

void EPD_POWER_OFF(I2CBus* bussin)
{
    bussin->write(adds_com, new uint8_t[1] { 0x28 }, 3); // POWER_OFF
}

void EPD_DEEP_SLEEP(I2CBus* bussin)
{
    ESP_LOGD(TAG, "Display Asleep");
    bussin->write(adds_com, new uint8_t[1] { 0xAD }, 1); // DEEP_SLEEP
}

// void EPD_1in9_ReadBusy(void)
// {
//     Serial.println("e-Paper busy");
//     delay(10);
//     while (1) { //=1 BUSY;
//         if (digitalRead(EPD_BUSY_PIN) == 1)
//             break;
//         delay(1);
//     }
//     delay(10);
//     Serial.println("e-Paper busy release");
// }

void AsyncDelay(I2CBus* bussin) { }

class SegmentedEPaper : public Component, public i2c::I2CDevice {
    I2CBus* _i2cBus;

    DisplayAction actionQueue[32];
    uint8_t queueIndex = 0;
    uint8_t queueHead = 0;
    uint8_t queueLength = 0;
    const uint8_t queueModulus = 0b00011111;

    uint64_t canRunNextActionAt = 0;
    uint64_t InactiveSince = UINT_64_MAX;
    bool displayAsleep = false;
    const uint32_t DisplayTimeout = 30 * 1000 * 1000; // 30 second timeout

    float displayedTemp = 0;
    uint8_t displayedSetpoint = 0;

    uint8_t Temp_tenthsPlace = 0;
    uint8_t Temp_onesPlace = 0;
    uint8_t Temp_tensPlace = 0;

    uint8_t SP_onesPlace = 0;
    uint8_t SP_tensPlace = 0;

    uint8_t UpdatesTillFullRefresh = UPDATES_BETWEEN_REFRESH;
    uint64_t TimeOfLastFullUpdate = 0;

    void AddAction(callback_function action, uint16_t delay, uint16_t Id = 0)
    {
        if (queueLength >= queueModulus) {
            // bork
            ESP_LOGE(TAG, "actionQueue overflow");
            return;
        }
        actionQueue[queueHead] = { action, delay, 0 };
        queueHead = (queueHead + 1) & queueModulus;
        queueLength++;
    }

    bool FindAction(uint16_t ID)
    {
        for (size_t i = queueIndex; i < queueHead - 1; i++) {
            if (actionQueue[i].actionId == ID)
                return true;
        }
        return false;
    }

public:
    SegmentedEPaper(I2CBus* Bus) { _i2cBus = Bus; }

    void SetPins(uint8_t EpdRst, uint8_t EpdBusy){

    }

    void Reset_Display(uint16_t id)
    {
        AddAction(EPD_RST_ON, 200, id);
        AddAction(EPD_RST_OFF, 20, id);
        AddAction(EPD_RST_ON, 200, id);
    }

    void CompensateForTemperature(uint8_t currentTemp)
    {
        ESP_LOGD(TAG, "temp Compensation %d", currentTemp);
        Current_EPD_Temperature_Compensation = currentTemp;
        AddAction(EPD_Temperature1, 10);
        AddAction(EPD_Temperature2, 10);
    }

    void Init_Display(void)
    {
        if (!initting) {
            initting = true;
            ESP_LOGD(TAG, "HEY ##### ###### #### Display init started");
            displayAsleep = false;
            Reset_Display(InitId);
            AddAction(EPD_PowerOn, 10, InitId);
            AddAction(EPD_Boost, 10, InitId);
            CompensateForTemperature(25);
        }
    }

    void Sleep_Display(void)
    {
        AddAction(EPD_POWER_OFF, 2000);
        AddAction(EPD_DEEP_SLEEP, 2000);
        AddAction(EPD_RST_OFF, 20);
        displayAsleep = true;
    }

    void FullRefreshScreen(void)
    {
        AddAction(EPD_ClearScreen_Brush1, 0);
        WriteScreen(DSPNUM_1in9_off);
        AddAction(EPD_ClearScreen_Brush2, 0);
        WriteScreen(DSPNUM_1in9_on, true);
        WriteScreen(DSPNUM_1in9_off);
        AddAction(EPD_Default_Brush, 0);
        TimeOfLastFullUpdate = esp_timer_get_time();
    }

    float absVal(float number)
    {
        if (number < 0)
            return -number;
        else
            return number;
    }

    void SetAmbientTemp(float temp)
    {
        float dif = absVal(displayedTemp - temp);
        if (dif > 0.5) {
            ESP_LOGD(TAG, "SET TEMP: current:%f next:%f", displayedTemp, temp);
            Temp_tenthsPlace = ((uint16_t)((temp * 10.0) + 0.5 - ((temp * 10.0) < 0))) % 10;
            Temp_onesPlace = ((uint16_t)temp) % 10;
            Temp_tensPlace = ((uint16_t)(temp / 10.0)) % 10;

            UpdateScreen();

            displayedTemp = temp;
        }
    }

    void SetTempSetpoint(float setpoint)
    {
        setpoint = setpoint + 0.5 - (setpoint < 0); // add 0.5 if x > 0
        uint8_t intSetpoint = (int)setpoint;

        if (displayedSetpoint != intSetpoint) {
            ESP_LOGD(TAG, "SET Setpoint: current:%d next:%f", displayedSetpoint, setpoint);
            SP_onesPlace = ((uint16_t)intSetpoint) % 10;
            SP_tensPlace = ((uint16_t)(intSetpoint / 10)) % 10;

            UpdateScreen();

            displayedSetpoint = intSetpoint;
        }
    }

    void UpdateScreen()
    {
        uint8_t dispVal[16] = { 0 };
        // memcpy(dispVal, CurrentDisplay, sizeof(uint8_t) * 16);
        memcpy(dispVal + TEMP_TENS, SegmentNumbers[Temp_tensPlace], sizeof(uint8_t) * 2);
        memcpy(dispVal + TEMP_ONES, SegmentNumbers[Temp_onesPlace], sizeof(uint8_t) * 2);
        memcpy(dispVal + TEMP_TENTHS, SegmentNumbers[Temp_tenthsPlace], sizeof(uint8_t) * 2);

        memcpy(dispVal + HUMID_TENS, SegmentNumbers[SP_tensPlace], sizeof(uint8_t) * 2);
        memcpy(dispVal + HUMID_ONES, SegmentNumbers[SP_onesPlace], sizeof(uint8_t) * 2);

        dispVal[13] = 0x06; // fahrenheit symbol
        dispVal[4] |= 0xf0; // decimal place

        WriteScreen(dispVal);
    }

    void IncrementTest(void)
    {
        // uint8_t dispVal[16] = { 0 };
        // memcpy(dispVal + 1, SegmentNumbers[1], sizeof(uint8_t) * 2);
        // memcpy(dispVal + 3, SegmentNumbers[2], sizeof(uint8_t) * 2);
        // memcpy(dispVal + 5, SegmentNumbers[3], sizeof(uint8_t) * 2);
        // memcpy(dispVal + 7, SegmentNumbers[4], sizeof(uint8_t) * 2);
        // memcpy(dispVal + 9, SegmentNumbers[5], sizeof(uint8_t) * 2);
        // memcpy(dispVal + 11, SegmentNumbers[6], sizeof(uint8_t) * 2);
        // ESP_LOGD(TAG, "dec:   %d,%d,%d,%d,%d,%d,%d,%d,%d",  dispVal[0], dispVal[1], dispVal[2], dispVal[3], dispVal[4], dispVal[5], dispVal[6], dispVal[7], dispVal[8]);
        // WriteScreen(dispVal);

        float temp = 0;
        if (displayedTemp < 20 || displayedTemp > 90) {
            temp = 20.7;
        } else {
            temp = displayedTemp + 1;
        }
        SetAmbientTemp(temp);
    }

    void DecrementTest(void)
    {
        // WriteScreen(DSPNUM_1in9_off);
        uint8_t setpoint = 0;
        if (displayedSetpoint < 20 || displayedSetpoint > 90) {
            setpoint = 20;
        } else {
            setpoint = displayedSetpoint + 1;
        }
        float spSim = ((float)setpoint) + 0.3;
        SetTempSetpoint(spSim);
    }
    void WriteScreen(uint8_t* data, bool fullBlack = false)
    {
        UpdatesTillFullRefresh--;
        bool updateNeeded = UpdatesTillFullRefresh <= 1 || esp_timer_get_time() > TimeOfLastFullUpdate + TimeBetweenFullUpdates;

        if (displayAsleep || updateNeeded) {
            Init_Display();
            AddAction(EPD_Default_Brush, 0);
        }
        if (updateNeeded) {
            TimeOfLastFullUpdate = esp_timer_get_time();
            UpdatesTillFullRefresh = UPDATES_BETWEEN_REFRESH;
            FullRefreshScreen();
            updatingDisplay = false;
        }

        if (!updatingDisplay) {
            updatingDisplay = true;
            if (ScreenBufferLength >= ScreenBufferModulus) {
                // bork
                ESP_LOGE(TAG, "ScreenBuffer overflow");
                return;
            }
            if (fullBlack) {
                data[15] = 0x03;
            } else {
                data[15] = 0x00;
            }

            memcpy(ScreenBuffer[ScreenBufferHead], data, sizeof(uint8_t) * 16);
            ScreenBufferHead = (ScreenBufferHead + 1) & ScreenBufferModulus;
            ScreenBufferLength++;

            AddAction(EPD_Write_Screen, 1500);
            // AddAction(AsyncDelay, 2000); // replace with monitoring the busy pin
            AddAction(EPD_Screen_Sleep, 500);
            // AddAction(AsyncDelay, 500);
        } else {
            memcpy(ScreenBuffer[ScreenBufferIndex], data, sizeof(uint8_t) * 16);
        }
    }

    void setup() override
    { // This will be called once to set up the component think of it as the setup() call in Arduino
        // set_i2c_address(0x44);
        gpio_num_t busy_pin = static_cast<gpio_num_t>(EPD_BUSY);
        gpio_num_t rst_pin = static_cast<gpio_num_t>(EPD_RST);

        Busy_pin_obj = new esp32::ESP32InternalGPIOPin();
        Busy_pin_obj->set_pin(busy_pin);
        Busy_pin_obj->set_inverted(false);
        Busy_pin_obj->set_drive_strength(::GPIO_DRIVE_CAP_2);
        Busy_pin_obj->set_flags(gpio::Flags::FLAG_INPUT);

        Rst_pin_obj = new esp32::ESP32InternalGPIOPin();
        Rst_pin_obj->set_pin(rst_pin);
        Rst_pin_obj->set_inverted(false);
        Rst_pin_obj->set_drive_strength(::GPIO_DRIVE_CAP_2);
        Rst_pin_obj->set_flags(gpio::Flags::FLAG_OUTPUT);
        // pinMode(EPD_BUSY, INPUT);
        // pinMode(EPD_RST, OUTPUT);
        Init_Display();
        FullRefreshScreen();
    }

    void loop() override
    { // This will be called very often after setup time. think of it as the loop() call in Arduino
        uint64_t currentTime = 3; // esp_timer_get_time()
        if (queueLength > 0 && currentTime > canRunNextActionAt) {
            DisplayAction runningAction = actionQueue[queueIndex];
            InactiveSince = UINT_64_MAX;
            // ESP_LOGD(TAG, "running action %d at %d, %d remaining, next action in %d", queueIndex, currentTime, queueLength, runningAction.delayAfter);

            runningAction.function(_i2cBus);
            canRunNextActionAt = 5 + (runningAction.delayAfter * 1000);

            queueIndex = (queueIndex + 1) & queueModulus;
            queueLength--;
            if (queueLength == 0) {
                InactiveSince = 5;
            }
        }
        if (!displayAsleep && currentTime > InactiveSince + DisplayTimeout) {
            Sleep_Display();
        }
    }
};
