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

    struct DisplayFurnishings
    {
        bool show_temp_symbol = false;
        bool temp_is_fahrenheit = false;
        bool show_lower_decimal_point = false;
        bool show_upper_decimal_point = false;
        bool show_humidity_symbol = false;
    };

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

        void set_Busy_pin(InternalGPIOPin* pin) { this->Busy_pin_ = pin; };
        void set_Reset_pin(InternalGPIOPin* pin) { this->Reset_pin_ = pin; };

        /// @brief displays a displays a float in the range of 199.9 to 0 on the upper 7 segment display
        /// @param Num takes input as float and rounds to nearest int
        /// @param flushToScreen if false will only update the stored display state which will be displayed the next time the display is written to
        void SetUpperDisplayFloat(float Num, bool flushToScreen = true);
        /// @brief displays a 2 digit integer on the lower 7 segment display
        /// @param Num takes input as float and rounds to nearest int
        /// @param flushToScreen if false will only update the stored display state which will be displayed the next time the display is written to
        void SetLowerDisplayInt(float Num, bool flushToScreen = true);
        void SetDisplayFurnishings(DisplayFurnishings val);

        /// @brief writes the display buffer data to the screen
        /// @param data full 16 byte array of data representing the state of the entire display
        /// @param ForceSerialWrites if false will overwrite any inflight display data that has not been flushed to the display so that we skip frames instead of performing back to back writes
        void WriteScreen(uint8_t* data, bool ForceSerialWrites = false);
        /// @brief initializes the display at startup or after reset/power loss
        void Init_Display(void);
        /// @brief puts the screen to sleep then removes power from the display
        void Sleep_Display(void);
        /// @brief powers the screen on off then on again
        void Reset_Display(void);
        /// @brief will flash the screen between white and black to clear any artifacts or ghosting
        /// @param forceFullRefresh if true will perform more flashes to better clear the screen at the expense of taking longer
        void FullRefreshScreen(bool forceFullRefresh = false);
        /// @brief Writes whatever the most up to date data is to the display
        void FlushToScreen(void);
        /// @brief display refresh timings change with temperature so the display driver needs to know the rough temperature of the display
        /// @param currentTemp 
        void CompensateForTemperature(uint8_t currentTemp);

        DisplayFurnishings _displayFurnishings;

    protected:
        static const uint64_t UINT_64_MAX = 0xfffffffffffffff; // one byte short because we are adding ~10 seconds worth of uS to this value when the screen is active and dont want an overflow

        static const uint8_t UPDATES_BETWEEN_REFRESH = 10;
        static const uint32_t DISPLAY_TIMEOUT = 30 * 1000 * 1000; // 30 second timeout

        static const uint8_t adds_com = 0x3C;
        static const uint8_t adds_data = 0x3D;

        InternalGPIOPin* Busy_pin_;
        InternalGPIOPin* Reset_pin_;

        uint8_t ScreenBuffer[8][16];
        bool ScreenDataRequired[8];
        uint8_t ScreenBufferIndex = 0;
        uint8_t ScreenBufferHead = 0;
        uint8_t ScreenBufferLength = 0;
        static const uint8_t ScreenBufferModulus = 0b0111;

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


        bool initting = false;
        bool BufferOverflow = false;
        bool displayAsleep = false;

        float displayedUpper = 0;
        uint8_t displayedLower = 0;
        bool DisplayOutOfDate = true;

        uint8_t Upper_tenthsPlace = 0;
        uint8_t Upper_onesPlace = 0;
        uint8_t Upper_tensPlace = 0;
        uint8_t Upper_HundredsPlace = 0;

        uint8_t Lower_tenthsPlace = 0;
        uint8_t Lower_onesPlace = 0;
        uint8_t Lower_tensPlace = 0;

        uint8_t Current_EPD_Temperature_Compensation = 25; // room temp 25 c

        bool AddActionStart();
        /// @brief add an action to be performed sequentially in the order they were added
        /// @param action (callback_function) action to be performed for this entry
        /// @param delay amount of time to wait after this action has completed before the next action acn run
        /// @param Id optional identifier for this specific action
        void AddAction(callback_function action, uint16_t delay, uint16_t Id = 0);
        /// @brief add an action to be performed sequentially in the order they were added
        /// @param action (callback_withReturn_function) action to be performed for this entry, will be called repeatedly until it returns true
        /// @param delay amount of time to wait after this action has completed before the next action acn run
        /// @param Id optional identifier for this specific action
        /// @param MaxRunTime if set to a positive number will not allow this task to run for longer than the specified time
        void AddAction(callback_withReturn_function action, uint16_t delay, uint16_t Id = 0, int16_t MaxRunTime = -1);
        /// @brief call after adding to the action queue to clean up any potential buffer overflows
        /// @details if the buffer overflowed empties it then calls init display and if there were any screen updates buffered writes the last update to the screen
        void CleanupQueueAndRestart(void);
        void AddScreenUpdateActions(void);

        /// @brief grab the oldest frame from the buffer write it to the display and advance the buffer index
        void EPD_Write_Screen();
        /// @brief the E-Paper has 2 separate i2c addresses so before each i2c write set the address to the correct one
        /// @param address the i2c address needed for this operation
        /// @param data pointer to an array to store the bytes
        /// @param len length of the buffer = number of bytes to send
        void addressed_write(uint8_t address, const uint8_t* data, size_t len);
        /// @brief setting reset high with the WaveShare provided dev board will turn on the 3.3V rail that powers the display
        void EPD_RST_ON() { this->Reset_pin_->digital_write(1); }
        /// @brief setting reset low with the WaveShare provided dev board will remove power from the display module
        void EPD_RST_OFF() { this->Reset_pin_->digital_write(0); }

        void EPD_PowerOn() { this->addressed_write(adds_com, new uint8_t[1] { 0x2B }, 1); }
        void EPD_POWER_OFF() { this->addressed_write(adds_com, new uint8_t[1] { 0x28 }, 3); }

        void EPD_Boost() { this->addressed_write(adds_com, new uint8_t[2] { 0xA7, 0xE0 }, 2); } // boost, TSON
        void EPD_Temperature1();
        void EPD_Temperature2();

        /* 5 waveform better ghosting, Boot waveform EPD_1in9_lut_5S*/
        void EPD_ClearScreen_Brush1() { this->addressed_write(adds_com, new uint8_t[7] { 0x82, 0x28, 0x20, 0xA8, 0xA0, 0x50, 0x65 }, 7); }
        /* GC waveform, The brush waveform EPD_1in9_lut_GC*/
        void EPD_ClearScreen_Brush2() { this->addressed_write(adds_com, new uint8_t[7] { 0x82, 0x20, 0x00, 0xA0, 0x80, 0x40, 0x63 }, 7); }
        /*  DU waveform white extinction diagram + black out diagram, Bureau of brush waveform EPD_1in9_lut_DU_WB*/
        void EPD_Default_Brush() { this->addressed_write(adds_com, new uint8_t[7] { 0x82, 0x80, 0x00, 0xC0, 0x80, 0x80, 0x62 }, 7); }
        // display off, HV OFF, sleep in
        void EPD_Screen_Sleep() { this->addressed_write(adds_com, new uint8_t[3] { 0xAE, 0x28, 0xAD }, 3); }


        void EPD_DEEP_SLEEP();
        uint8_t EPD_ReadBusy();
    };

} // namespace segmented_epaper
} // namespace esphome
