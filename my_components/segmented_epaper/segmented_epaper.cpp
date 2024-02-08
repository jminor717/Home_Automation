#include "segmented_epaper.h"
#include "esphome/components/i2c/i2c_bus.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace segmented_epaper {

    static const char* const TAG = "Segmented_E_paper";

    static const uint8_t UPPER_HUNDREDS = 0;
    static const uint8_t UPPER_TENS = 1;
    static const uint8_t UPPER_ONES = 3;
    static const uint8_t UPPER_TENTHS = 11;

    static const uint8_t LOWER_TENS = 5;
    static const uint8_t LOWER_ONES = 7;
    static const uint8_t LOWER_TENTHS = 9;
    // clang-format off
    static uint8_t DSPNUM_1in9_on[]   = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,      };  // all black
    static uint8_t DSPNUM_1in9_off[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      };  // all white
    static uint8_t DSPNUM_1in9_W3[]   = {0x00, 0xf5, 0x1f, 0xf5, 0x1f, 0xf5, 0x1f, 0xf5, 0x1f, 0xf5, 0x1f, 0xf5, 0x1f, 0x00, 0x00, 0x00,      };  // 3

    static const uint8_t DIGIT_OFF = 10;
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
        {0x00, 0x00}, // DIGIT_OFF
    } ;
    // clang-format on

    struct DisplayFurnishings {
        bool TempSymbol;
        bool TempIsFahrenheit;
        bool UpperDecimalPoint;
        bool LowerDecimalPoint;
        bool Humidity;
    } _displayFurnishings;

    // void HOT IRAM_ATTR Segmented_ePaperStore::d0_gpio_intr(Segmented_ePaperStore* arg)
    // {
    //     if (arg->d0.digital_read())
    //         return;
    //     arg->count++;
    //     arg->value <<= 1;
    //     arg->last_bit_time = millis();
    //     arg->done = false;
    // }

    float absVal(float number)
    {
        if (number < 0)
            return -number;
        else
            return number;
    }

    void Segmented_ePaper::SetUpperDisplayFloat(float Num, bool flush)
    {
        float dif = absVal(displayedUpper - Num);
        if (dif > 0.5) {
            // limit number of writes to display to prolong lifespan
            ESP_LOGD(TAG, "SET Upper Display: current:%f next:%f, flush:%d", displayedUpper, Num, flush);
            Upper_HundredsPlace = DIGIT_OFF;
            Upper_tenthsPlace = ((uint16_t)((Num * 10.0) + 0.5 - ((Num * 10.0) < 0))) % 10;
            Upper_onesPlace = ((uint16_t)Num) % 10;
            Upper_tensPlace = ((uint16_t)(Num / 10.0)) % 10;

            _displayFurnishings.UpperDecimalPoint = true;
            _displayFurnishings.TempSymbol = true;
            _displayFurnishings.TempIsFahrenheit = true;

            displayedUpper = Num;
            DisplayOutOfDate = true;
        }

        if (flush && DisplayOutOfDate) {
            FlushToScreen();
        }
    }

    void Segmented_ePaper::SetLowerDisplayInt(float Num, bool flush)
    {
        Num = Num + 0.5 - (Num < 0); // add 0.5 if x > 0
        uint8_t intNum = (int)Num;

        if (displayedLower != intNum) {
            ESP_LOGD(TAG, "SET Lower Display: current:%d next:%f", displayedLower, Num);
            Lower_onesPlace = ((uint16_t)intNum) % 10;
            Lower_tensPlace = ((uint16_t)(intNum / 10)) % 10;
            Lower_tenthsPlace = DIGIT_OFF;

            _displayFurnishings.LowerDecimalPoint = false;

            displayedLower = intNum;
            DisplayOutOfDate = true;
        }
        if (flush && DisplayOutOfDate) {
            FlushToScreen();
        }
    }

    void Segmented_ePaper::FlushToScreen()
    {
        uint8_t dispVal[16] = { 0 };
        // clang-format off
        memcpy(dispVal + UPPER_HUNDREDS, SegmentNumbers[Upper_HundredsPlace] + 1, sizeof(uint8_t)); // hundreds place only uses one byte since it can only be empty or 1
        memcpy(dispVal + UPPER_TENS,     SegmentNumbers[Upper_tensPlace],         sizeof(uint8_t) * 2);
        memcpy(dispVal + UPPER_ONES,     SegmentNumbers[Upper_onesPlace],         sizeof(uint8_t) * 2);
        memcpy(dispVal + UPPER_TENTHS,   SegmentNumbers[Upper_tenthsPlace],       sizeof(uint8_t) * 2);

        memcpy(dispVal + LOWER_TENS,     SegmentNumbers[Lower_tensPlace],         sizeof(uint8_t) * 2);
        memcpy(dispVal + LOWER_ONES,     SegmentNumbers[Lower_onesPlace],         sizeof(uint8_t) * 2);
        memcpy(dispVal + LOWER_TENTHS,   SegmentNumbers[Lower_tenthsPlace],       sizeof(uint8_t) * 2);
        // clang-format on

        if (_displayFurnishings.TempSymbol) {
            if (_displayFurnishings.TempIsFahrenheit)
                dispVal[13] = 0x06; // fahrenheit symbol 0b0000_0110
            else
                dispVal[13] = 0x05; // Celsius symbol    0b0000_0101
        }
        if (_displayFurnishings.LowerDecimalPoint)
            dispVal[8] |= 0xf0; // lower decimal place
        if (_displayFurnishings.UpperDecimalPoint)
            dispVal[4] |= 0xf0; // upper decimal place
        if (_displayFurnishings.Humidity)
            dispVal[10] |= 0xf0; // humidity symbol

        ESP_LOGD(TAG, "FlushToScreen: Up:%d|%d|%d|.|%d  Low:%d|%d|.|%d ", Upper_HundredsPlace, Upper_tensPlace, Upper_onesPlace, Upper_tenthsPlace, Lower_tensPlace, Lower_onesPlace, Lower_tenthsPlace);

        /*
         ✔ The decimal point and % are the 5th place in the 4th, 8th, and 10th positions respectively.
         ✔ The 13th position can display Celsius (0 x 05) and Fahrenheit (0 x 06),
         ❌ the 0th position is the bottom line, the 1st position is the middle line, the 2nd position is other displays,
         ❌ the 3rd position shows the Bluetooth logo, and the 4th position Bit shows the power identification, number 14 is useless.
        */
        DisplayOutOfDate = false;
        WriteScreen(dispVal);
    }

    void Segmented_ePaper::setup()
    {
        this->Busy_pin_->setup();
        this->Reset_pin_->setup();
        Init_Display();
        CompensateForTemperature(25);
        FullRefreshScreen(true);
        // this->store_.d0 = this->d0_pin_->to_isr();
        // this->store_.d1 = this->d1_pin_->to_isr();
        // this->d0_pin_->attach_interrupt(Segmented_ePaperStore::d0_gpio_intr, &this->store_, gpio::INTERRUPT_FALLING_EDGE);
        // this->d1_pin_->attach_interrupt(Segmented_ePaperStore::d1_gpio_intr, &this->store_, gpio::INTERRUPT_FALLING_EDGE);
    }

    void Segmented_ePaper::loop()
    {
        uint64_t currentTime = esp_timer_get_time();
        if (queueLength > 0 && currentTime > canRunNextActionAt) {
            DisplayAction runningAction = actionQueue[queueIndex];
            InactiveSince = UINT_64_MAX; // set to max val so that we don't unintentional sleep the display

            if (runningActionId != runningAction.actionId) {
                runningActionId = runningAction.actionId;
                currentActionStartedAt = currentTime;
            }

            bool actionComplete = true;
            if (runningAction.UseReturn) {
                actionComplete = (bool)(*this.*runningAction.returnFunction)();
                if (runningAction.maxRunTime >= 0 && !actionComplete) {
                    actionComplete = currentTime > currentActionStartedAt + (runningAction.maxRunTime * 1000);
                    if (actionComplete) {
                        ESP_LOGD(TAG, "Polling action timed out");
                    }
                }
            } else {
                ESP_LOGD(TAG, "running action %d Len %d, next action in %d", queueIndex, queueLength, runningAction.delayAfter);
                // https://stackoverflow.com/questions/7869716/dereferencing-a-member-pointer-error-cannot-be-used-as-member-pointer
                // https://www.reddit.com/r/Cplusplus/comments/9t8417/error_must_use_or_or_to_call_pointertomember/
                (*this.*runningAction.function)();
            }

            if (actionComplete) {
                currentTime = esp_timer_get_time();
                canRunNextActionAt = currentTime + (runningAction.delayAfter * 1000);

                queueIndex = (queueIndex + 1) & queueModulus;
                queueLength--;
                if (queueLength == 0) {
                    InactiveSince = currentTime;
                }
            }
        }
        if (!displayAsleep && currentTime > InactiveSince + DISPLAY_TIMEOUT) {
            // only sleep after a delay so that we don't have to constantly re-init the display as the user is slowly updating the setpoint
            Sleep_Display();
        }
    }

    /// @brief writes the display buffer data to the screen
    /// @param data full 16 byte array of data representing the state of the entire display
    /// @param fullBlack weird detail from the driver provided by WaveShare when we are writing DSPNUM_1in9_on to the screen
    /// @param ForceSerialWrites if false will overwrite any inflight display data that has not been flushed to the display so that we skip frames instead of performing back to back writes
    void Segmented_ePaper::WriteScreen(uint8_t* data, bool fullBlack, bool ForceSerialWrites)
    {
        bool SetBrushBackToDefault = false;
        if (displayAsleep) {
            // if the display was put into sleep mode we need to re-init before we can write to the display
            Init_Display();
            // first write after init should be made with the with the clear screen brush
            AddAction(&Segmented_ePaper::EPD_ClearScreen_Brush2, 0);
            UpdatesTillFullRefresh = UPDATES_BETWEEN_REFRESH; // using this brush effectively clears any ghosting so a full refresh can be pushed back
            SetBrushBackToDefault = true;
        }

        UpdatesTillFullRefresh--;
        bool updateNeeded = UpdatesTillFullRefresh <= 1;
        if (updateNeeded) {
            // after a ceratin number of partial updates we should perform a full refresh to clear any artifacts that may have built up
            TimeOfLastFullUpdate = esp_timer_get_time();
            FullRefreshScreen(false);
        }

        if (fullBlack) {
            data[15] = 0x03;
        } else {
            data[15] = 0x00;
        }

        int8_t overWriteableScreen = -1;
        for (size_t i = 0; i < ScreenBufferLength; i++) {
            uint8_t index = (i + ScreenBufferIndex) & ScreenBufferModulus;
            if (!ScreenDataRequired[index]) {
                overWriteableScreen = index;
                break;
            }
        }

        // if we are forcing serial writes 
        // or not already in the middle of updating the display 
        // or if none the in flight updates can be overriden
        // then add this update to the head of the screen buffer
        if (ForceSerialWrites || !updatingDisplay || overWriteableScreen == -1) {
            updatingDisplay = true;
            if (ScreenBufferLength >= ScreenBufferModulus) {
                // bork
                ESP_LOGE(TAG, "ScreenBuffer overflow");
                return;
            }

            memcpy(ScreenBuffer[ScreenBufferHead], data, sizeof(uint8_t) * 16);
            ScreenDataRequired[ScreenBufferHead] = ForceSerialWrites;
            ScreenBufferHead = (ScreenBufferHead + 1) & ScreenBufferModulus;
            ScreenBufferLength++;

            AddScreenUpdateActions();
            CleanupQueueAndRestart();
        } else {
            // if one of the screens can be overriden replace its buffer
            memcpy(ScreenBuffer[overWriteableScreen], data, sizeof(uint8_t) * 16);
            ESP_LOGI(TAG, "Bypass display update tasks Tasks");
        }
        if (SetBrushBackToDefault){
            AddAction(&Segmented_ePaper::EPD_Default_Brush, 0);
        }
    }

    void Segmented_ePaper::AddScreenUpdateActions()
    {
        AddAction(&Segmented_ePaper::EPD_Write_Screen, 1500);
        // AddAction(&Segmented_ePaper::EPD_ReadBusy, 10, 0, 1500);
        AddAction(&Segmented_ePaper::EPD_Screen_Sleep, 500);
        // AddAction(&Segmented_ePaper::EPD_ReadBusy, 10, 0, 500);
    }

    void Segmented_ePaper::Init_Display()
    {
        if (!initting) {
            initting = true;
            ESP_LOGD(TAG, "HEY ##### ###### #### Display init started");
            displayAsleep = false;
            Reset_Display();
            AddAction(&Segmented_ePaper::EPD_PowerOn, 10);
            AddAction(&Segmented_ePaper::EPD_Boost, 10);
            // CompensateForTemperature(25);
        }
    }

    /// @brief puts the screen to sleep then removes power from the display
    void Segmented_ePaper::Sleep_Display()
    {
        AddAction(&Segmented_ePaper::EPD_POWER_OFF, 2000);
        AddAction(&Segmented_ePaper::EPD_DEEP_SLEEP, 2000);
        AddAction(&Segmented_ePaper::EPD_RST_OFF, 20);
        displayAsleep = true;
        CleanupQueueAndRestart();
    }

    /// @brief powers the screen on off then on again
    void Segmented_ePaper::Reset_Display()
    {
        AddAction(&Segmented_ePaper::EPD_RST_ON, 200);
        AddAction(&Segmented_ePaper::EPD_RST_OFF, 20);
        AddAction(&Segmented_ePaper::EPD_RST_ON, 200);
        CleanupQueueAndRestart();
    }

    /// @brief will flash the screen between white and black to clear any artifacts or ghosting
    /// @param forceFullRefresh if true will perform more flashes to better clear the screen at the expense of taking longer
    void Segmented_ePaper::FullRefreshScreen(bool forceFullRefresh)
    {
        UpdatesTillFullRefresh = UPDATES_BETWEEN_REFRESH; // make sure we dont call full refresh again when calling write screen
        if (forceFullRefresh) {
            AddAction(&Segmented_ePaper::EPD_ClearScreen_Brush1, 0);
            WriteScreen(DSPNUM_1in9_off, false, true);
            AddAction(&Segmented_ePaper::EPD_ClearScreen_Brush2, 0);
            WriteScreen(DSPNUM_1in9_on, true, true);
            WriteScreen(DSPNUM_1in9_off, false, true);
        }else{
            AddAction(&Segmented_ePaper::EPD_ClearScreen_Brush2, 0);
            WriteScreen(DSPNUM_1in9_off, false, true);
        }
        AddAction(&Segmented_ePaper::EPD_Default_Brush, 0);
        CleanupQueueAndRestart();
    }

    void Segmented_ePaper::CompensateForTemperature(uint8_t currentTemp)
    {
        ESP_LOGD(TAG, "temp Compensation %d", currentTemp);
        Current_EPD_Temperature_Compensation = currentTemp;
        AddAction(&Segmented_ePaper::EPD_Temperature1, 10);
        AddAction(&Segmented_ePaper::EPD_Temperature2, 10);
        CleanupQueueAndRestart();
    }

    bool Segmented_ePaper::AddActionStart()
    {
        if (BufferOverflow) {
            return false;
        }
        if (queueLength >= queueModulus) {
            // bork
            ESP_LOGE(TAG, "actionQueue overflow");
            BufferOverflow = true;
            return false;
        }
        return true;
    }

    /// @brief add an action to be performed sequentially in the order they were added
    /// @param action (callback_function) action to be performed for this entry
    /// @param delay amount of time to wait after this action has completed before the next action acn run
    /// @param Id optional identifier for this specific action
    void Segmented_ePaper::AddAction(callback_function action, uint16_t delay, uint16_t Id)
    {
        if (AddActionStart()) {
            DisplayAction _action;
            _action.UseReturn = false;
            _action.function = action;
            _action.delayAfter = delay;
            _action.actionId = Id;
            actionQueue[queueHead] = _action;
            queueHead = (queueHead + 1) & queueModulus;
            queueLength++;
            ESP_LOGI(TAG, "insert  action %d Len %d, running index %d", queueHead, queueLength, queueIndex);
        }
    }

    /// @brief add an action to be performed sequentially in the order they were added
    /// @param action (callback_withReturn_function) action to be performed for this entry, will be called repeatedly until it returns true
    /// @param delay amount of time to wait after this action has completed before the next action acn run
    /// @param Id optional identifier for this specific action
    /// @param MaxRunTime if set to a positive number will not allow this task to run for longer than the specified time
    void Segmented_ePaper::AddAction(callback_withReturn_function action, uint16_t delay, uint16_t Id, int16_t MaxRunTime)
    {
        if (AddActionStart()) {
            DisplayAction _action;
            _action.UseReturn = true;
            _action.returnFunction = action;
            _action.delayAfter = delay;
            _action.actionId = Id;
            _action.maxRunTime = MaxRunTime;
            actionQueue[queueHead] = _action;
            queueHead = (queueHead + 1) & queueModulus;
            queueLength++;
        }
    }

    /// @brief call after adding to the action queue to clean up any potential buffer overflows
    /// @details if the buffer overflowed empties it then calls init display and if there were any screen updates buffered writes the last update to the screen
    void Segmented_ePaper::CleanupQueueAndRestart()
    {
        if (BufferOverflow) {
            BufferOverflow = false;
            ESP_LOGI(TAG, "Reseting Display Queue");
            queueIndex = 0;
            queueHead = 0;
            queueLength = 0;
            Init_Display();
            canRunNextActionAt = esp_timer_get_time();
            if (ScreenBufferLength > 0) {
                ScreenBufferIndex = (ScreenBufferHead - 1) & ScreenBufferModulus;
                ScreenBufferLength = 1;
                AddScreenUpdateActions();
            }
        }
    }

    /// @brief grab the oldest frame from the buffer write it to the display and advance the buffer index
    void Segmented_ePaper::EPD_Write_Screen()
    {
        if (ScreenBufferLength <= 1) {
            // if this is the last remaining update set updatingDisplay to false
            updatingDisplay = false;
        }

        this->addressed_write(adds_com, new uint8_t[5] { 0xAC, 0x2B, 0x40, 0xA9, 0xA8 }, 5); // Close the sleep, turn on the power,  Write RAM address, Turn on the first SRAM, Shut down the first SRAM

        // ESP_LOGD(TAG, "WRITE:   %d,%d,%d,%d,%d,%d,%d,%d,%d", dispNumber, dispVal[0], dispVal[1], dispVal[2], dispVal[3], dispVal[4], dispVal[5], dispVal[6], dispVal[7], dispVal[8]);
        this->addressed_write(adds_data, ScreenBuffer[ScreenBufferIndex], 16);

        ScreenBufferLength--;
        ScreenBufferIndex = (ScreenBufferIndex + 1) & ScreenBufferModulus;

        ESP_LOGI(TAG, "screen Updated, %d updates remaining, next Index %d", ScreenBufferLength, ScreenBufferIndex);

        this->addressed_write(adds_com, new uint8_t[3] { 0xAB, 0xAA, 0xAF }, 3); // Turn on the second SRAM, Shut down the second SRAM, display on
    }

    /// @brief the E-Paper has 2 separate i2c addresses so before each i2c write set the address to the correct one
    /// @param address the i2c address needed for this operation
    /// @param data pointer to an array to store the bytes
    /// @param len length of the buffer = number of bytes to send
    void Segmented_ePaper::addressed_write(uint8_t address, const uint8_t* data, size_t len)
    {
        this->set_i2c_address(address);
        this->write(data, len);
    }

    void Segmented_ePaper::EPD_Temperature1() { this->addressed_write(adds_com, new uint8_t[3] { 0x7E, 0x81, 0xB4 }, 3); }
    void Segmented_ePaper::EPD_Temperature2()
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
        this->addressed_write(adds_com, data, 2);
        initting = false;
    }

    void Segmented_ePaper::EPD_DEEP_SLEEP()
    {
        ESP_LOGD(TAG, "Display Asleep");
        this->addressed_write(adds_com, new uint8_t[1] { 0xAD }, 1); // DEEP_SLEEP
    }

    uint8_t Segmented_ePaper::EPD_ReadBusy()
    {
        // busy pin high indicates display is not busy and the next action can start
        return (uint8_t)Busy_pin_->digital_read();
        // while (1) { //=1 BUSY;
        //     if (digitalRead(EPD_BUSY_PIN) == 1)
        //         break;
        //     delay(1);
        // }
    }

    void Segmented_ePaper::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Segmented_ePaper reader:");
        LOG_PIN("  Reset pin: ", this->Reset_pin_);
        LOG_PIN("  Busy : ", this->Busy_pin_);
    }

} // namespace segmented_epaper
} // namespace esphome
