#pragma once

#include "esphome.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/i2c/i2c_bus.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <limits>
// using namespace i2c;

namespace esphome {
namespace segmented_epaper {

    class Segmented_ePaper;

    // typedef void (*callback_function)();

    typedef void (Segmented_ePaper::*callback_function)();
    typedef uint8_t (Segmented_ePaper::*callback_withReturn_function)();

    struct DisplayAction {
        bool UseReturn;
        callback_function function; // Function pointer to display callback function
        callback_withReturn_function returnFunction;
        uint16_t delayAfter; // time to wait after this action before the next is performed
        int16_t maxRunTime; // for a function with return specifies the maximum amount of time this function can run for;
        uint16_t actionId;
    };

    class Segmented_ePaper : public Component, public i2c::I2CDevice {
    public:
        float get_setup_priority() const override { return setup_priority::HARDWARE; }
        void setup() override;
        void loop() override;
        void dump_config() override;

        void set_Busy_pin_pin(InternalGPIOPin* pin) { this->Busy_pin_ = pin; };
        void set_Reset_pin_pin(InternalGPIOPin* pin) { this->Reset_pin_ = pin; };

        void SetUpperDisplayFloat(float Num);
        void SetLowerDisplayInt(float setpoint);

        void Init_Display(void);
        void Sleep_Display(void);
        void Reset_Display(void);
        void FullRefreshScreen(void);
        void CompensateForTemperature(uint8_t currentTemp);
        void WriteScreen(uint8_t* data, bool fullBlack = false);

    protected:
        static const uint64_t UINT_64_MAX = 0xfffffffffffffff; // one byte short because we are adding ~10 seconds worth of uS to this value when the screen is active and dont want an overflow

        static const uint8_t UPDATES_BETWEEN_REFRESH = 20;
        static const uint64_t TIME_BETWEEN_FULL_UPDATES = 60 * 1000 * 1000; // 1 minute timeout
        static const uint32_t DISPLAY_TIMEOUT = 30 * 1000 * 1000; // 30 second timeout

        static const uint8_t adds_com = 0x3C;
        static const uint8_t adds_data = 0x3D;

        InternalGPIOPin* Busy_pin_;
        InternalGPIOPin* Reset_pin_;

        uint8_t ScreenBuffer[8][16];
        uint8_t ScreenBufferIndex = 0;
        uint8_t ScreenBufferHead = 0;
        uint8_t ScreenBufferLength = 0;
        static const uint8_t ScreenBufferModulus = 0b0111;
        uint8_t CurrentDisplay[16] = { 0 };

        DisplayAction actionQueue[32];
        uint8_t queueIndex = 0;
        uint8_t queueHead = 0;
        uint8_t queueLength = 0;
        static const uint8_t queueModulus = 0b00011111;

        uint64_t canRunNextActionAt = 0;
        uint8_t runningActionId = 0;
        uint64_t currentActionStartedAt = 0;
        uint64_t InactiveSince = UINT_64_MAX;
        uint8_t UpdatesTillFullRefresh = UPDATES_BETWEEN_REFRESH;
        uint64_t TimeOfLastFullUpdate = 0;

        bool updatingDisplay = false;
        bool initting = false;
        bool BufferOverflow = false;
        bool displayAsleep = false;

        float displayedUpper = 0;
        uint8_t displayedLower = 0;

        uint8_t Upper_tenthsPlace = 0;
        uint8_t Upper_onesPlace = 0;
        uint8_t Upper_tensPlace = 0;

        uint8_t Lower_onesPlace = 0;
        uint8_t Lower_tensPlace = 0;

        uint8_t Current_EPD_Temperature_Compensation = 25; // room temp 25 c

        bool AddActionStart();
        void AddAction(callback_function action, uint16_t delay, uint16_t Id = 0);
        void AddAction(callback_withReturn_function action, uint16_t delay, uint16_t Id = 0, int16_t MaxRunTime = -1); // numeric_limits<int16_t>::max()
        void UpdateScreen(void);
        void CleanupQueueAndRestart(void);

        void addressed_write(uint8_t address, const uint8_t* data, size_t len);
        void EPD_RST_ON() { this->Reset_pin_->digital_write(1); }
        void EPD_RST_OFF() { this->Reset_pin_->digital_write(0); }

        void EPD_PowerOn() { this->addressed_write(adds_com, new uint8_t[1] { 0x2B }, 1); }
        void EPD_POWER_OFF() { this->addressed_write(adds_com, new uint8_t[1] { 0x28 }, 3); }

        void EPD_Boost() { this->addressed_write(adds_com, new uint8_t[2] { 0xA7, 0xE0 }, 2); } // boost, TSON
        void EPD_Temperature1() { this->addressed_write(adds_com, new uint8_t[3] { 0x7E, 0x81, 0xB4 }, 3); }
        void EPD_Temperature2();

        /* 5 waveform better ghosting, Boot waveform EPD_1in9_lut_5S*/
        void EPD_ClearScreen_Brush1() { this->addressed_write(adds_com, new uint8_t[7] { 0x82, 0x28, 0x20, 0xA8, 0xA0, 0x50, 0x65 }, 7); }
        /* GC waveform, The brush waveform EPD_1in9_lut_GC*/
        void EPD_ClearScreen_Brush2() { this->addressed_write(adds_com, new uint8_t[7] { 0x82, 0x20, 0x00, 0xA0, 0x80, 0x40, 0x63 }, 7); }
        /*  DU waveform white extinction diagram + black out diagram, Bureau of brush waveform EPD_1in9_lut_DU_WB*/
        void EPD_Default_Brush() { this->addressed_write(adds_com, new uint8_t[7] { 0x82, 0x80, 0x00, 0xC0, 0x80, 0x80, 0x62 }, 7); }
        // display off, HV OFF, sleep in
        void EPD_Screen_Sleep() { this->addressed_write(adds_com, new uint8_t[3] { 0xAE, 0x28, 0xAD }, 3); }

        void EPD_Write_Screen();
        void EPD_DEEP_SLEEP();
        uint8_t EPD_ReadBusy();
        void AsyncDelay() { }
    };

} // namespace segmented_epaper
} // namespace esphome
