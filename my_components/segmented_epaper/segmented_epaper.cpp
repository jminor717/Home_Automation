#include "segmented_epaper.h"
#include "esphome/components/i2c/i2c_bus.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace segmented_epaper {

    static const char* const TAG = "Segmented_E_paper";

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

    void Segmented_ePaper::SetUpperDisplayFloat(float Num)
    {
        float dif = absVal(displayedUpper - Num);
        if (dif > 0.5) {
            // limit number of writes to display to prolong lifespan
            ESP_LOGD(TAG, "SET Upper Display: current:%f next:%f", displayedUpper, Num);
            Upper_tenthsPlace = ((uint16_t)((Num * 10.0) + 0.5 - ((Num * 10.0) < 0))) % 10;
            Upper_onesPlace = ((uint16_t)Num) % 10;
            Upper_tensPlace = ((uint16_t)(Num / 10.0)) % 10;

            UpdateScreen();

            displayedUpper = Num;
        }
    }

    void Segmented_ePaper::SetLowerDisplayInt(float setpoint)
    {
        setpoint = setpoint + 0.5 - (setpoint < 0); // add 0.5 if x > 0
        uint8_t intSetpoint = (int)setpoint;

        if (displayedLower != intSetpoint) {
            ESP_LOGD(TAG, "SET Setpoint: current:%d next:%f", displayedLower, setpoint);
            Lower_onesPlace = ((uint16_t)intSetpoint) % 10;
            Lower_tensPlace = ((uint16_t)(intSetpoint / 10)) % 10;

            UpdateScreen();

            displayedLower = intSetpoint;
        }
    }

    void Segmented_ePaper::UpdateScreen()
    {
        uint8_t dispVal[16] = { 0 };
        // memcpy(dispVal, CurrentDisplay, sizeof(uint8_t) * 16);
        memcpy(dispVal + UPPER_TENS, SegmentNumbers[Upper_tensPlace], sizeof(uint8_t) * 2);
        memcpy(dispVal + UPPER_ONES, SegmentNumbers[Upper_onesPlace], sizeof(uint8_t) * 2);
        memcpy(dispVal + UPPER_TENTHS, SegmentNumbers[Upper_tenthsPlace], sizeof(uint8_t) * 2);

        memcpy(dispVal + LOWER_TENS, SegmentNumbers[Lower_tensPlace], sizeof(uint8_t) * 2);
        memcpy(dispVal + LOWER_ONES, SegmentNumbers[Lower_onesPlace], sizeof(uint8_t) * 2);
        memcpy(dispVal + LOWER_TENTHS, SegmentNumbers[DIGIT_OFF], sizeof(uint8_t) * 2);

        dispVal[13] = 0x06; // fahrenheit symbol
        dispVal[4] |= 0xf0; // decimal place

        WriteScreen(dispVal);
    }

    void Segmented_ePaper::setup()
    {
        this->Busy_pin_->setup();
        this->Reset_pin_->setup();
        Init_Display();
        FullRefreshScreen();
        // this->store_.d0 = this->d0_pin_->to_isr();
        // this->store_.d1 = this->d1_pin_->to_isr();
        // this->d0_pin_->attach_interrupt(Segmented_ePaperStore::d0_gpio_intr, &this->store_, gpio::INTERRUPT_FALLING_EDGE);
        // this->d1_pin_->attach_interrupt(Segmented_ePaperStore::d1_gpio_intr, &this->store_, gpio::INTERRUPT_FALLING_EDGE);
    }

    void Segmented_ePaper::loop()
    {
        uint64_t currentTime = micros();
        if (queueLength > 0 && currentTime > canRunNextActionAt) {
            DisplayAction runningAction = actionQueue[queueIndex];
            InactiveSince = UINT_64_MAX; // set to max val so that we don't unintentional sleep the display

            if (runningActionId != runningAction.actionId){
                runningActionId = runningAction.actionId;
                currentActionStartedAt = currentTime;
            }

            // ESP_LOGD(TAG, "running action %d at %d, %d remaining, next action in %d", queueIndex, currentTime, queueLength, runningAction.delayAfter);

            bool actionComplete = true;
            if (runningAction.UseReturn) {
                actionComplete = (*this.*runningAction.returnFunction)();
                if (runningAction.maxRunTime >= 0 && !actionComplete) {
                     actionComplete = currentTime > currentActionStartedAt + (runningAction.maxRunTime * 1000);
                     if(actionComplete){
                         ESP_LOGD(TAG, "Polling action timed out");
                     }
                }
            }else{
                // https://stackoverflow.com/questions/7869716/dereferencing-a-member-pointer-error-cannot-be-used-as-member-pointer
                // https://www.reddit.com/r/Cplusplus/comments/9t8417/error_must_use_or_or_to_call_pointertomember/
                (*this.*runningAction.function)();
            }


            if(actionComplete){
                currentTime = micros();
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

    void Segmented_ePaper::WriteScreen(uint8_t* data, bool fullBlack)
    {
        UpdatesTillFullRefresh--;
        bool updateNeeded = UpdatesTillFullRefresh <= 1 || micros() > TimeOfLastFullUpdate + TIME_BETWEEN_FULL_UPDATES;

        if (displayAsleep || updateNeeded) {
            Init_Display();
            AddAction(&Segmented_ePaper::EPD_Default_Brush, 0);
        }
        if (updateNeeded) {
            TimeOfLastFullUpdate = micros();
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

            AddAction(&Segmented_ePaper::EPD_Write_Screen, 1);
            AddAction(&Segmented_ePaper::EPD_ReadBusy, 10, 0, 1500);
            AddAction(&Segmented_ePaper::EPD_Screen_Sleep, 1);
            AddAction(&Segmented_ePaper::EPD_ReadBusy, 10, 500);
            CleanupQueueAndRestart();
        } else {
            memcpy(ScreenBuffer[ScreenBufferIndex], data, sizeof(uint8_t) * 16);
        }
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
            CompensateForTemperature(25);
        }
    }

    void Segmented_ePaper::Sleep_Display()
    {
        AddAction(&Segmented_ePaper::EPD_POWER_OFF, 2000);
        AddAction(&Segmented_ePaper::EPD_DEEP_SLEEP, 2000);
        AddAction(&Segmented_ePaper::EPD_RST_OFF, 20);
        displayAsleep = true;
        CleanupQueueAndRestart();
    }
    void Segmented_ePaper::Reset_Display()
    {
        AddAction(&Segmented_ePaper::EPD_RST_ON, 200);
        AddAction(&Segmented_ePaper::EPD_RST_OFF, 20);
        AddAction(&Segmented_ePaper::EPD_RST_ON, 200);
        CleanupQueueAndRestart();
    }

    void Segmented_ePaper::FullRefreshScreen()
    {
        AddAction(&Segmented_ePaper::EPD_ClearScreen_Brush1, 0);
        WriteScreen(DSPNUM_1in9_off);
        AddAction(&Segmented_ePaper::EPD_ClearScreen_Brush2, 0);
        WriteScreen(DSPNUM_1in9_on, true);
        WriteScreen(DSPNUM_1in9_off);
        AddAction(&Segmented_ePaper::EPD_Default_Brush, 0);
        TimeOfLastFullUpdate = micros();
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
        }
    }

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

    void Segmented_ePaper::CleanupQueueAndRestart()
    {
        if (BufferOverflow) {
            BufferOverflow = false;
            ESP_LOGI(TAG, "Reseting Display Queue");
            queueIndex = 0;
            queueHead = 0;
            queueLength = 0;
            TimeOfLastFullUpdate = micros();
            Init_Display();
            canRunNextActionAt = micros();
            if (ScreenBufferLength > 0) {
                ScreenBufferLength = 1;
                AddAction(&Segmented_ePaper::EPD_Write_Screen, 1);
                AddAction(&Segmented_ePaper::EPD_ReadBusy, 10, 0, 1500);
                AddAction(&Segmented_ePaper::EPD_Screen_Sleep, 1);
                AddAction(&Segmented_ePaper::EPD_ReadBusy, 10, 500);
            }
        }
    }

    // grab the oldest frame from the buffer and write it to the display
    void Segmented_ePaper::EPD_Write_Screen()
    {
        this->addressed_write(adds_com, new uint8_t[5] { 0xAC, 0x2B, 0x40, 0xA9, 0xA8 }, 5); // Close the sleep, turn on the power,  Write RAM address, Turn on the first SRAM, Shut down the first SRAM

        // ESP_LOGD(TAG, "WRITE:   %d,%d,%d,%d,%d,%d,%d,%d,%d", dispNumber, dispVal[0], dispVal[1], dispVal[2], dispVal[3], dispVal[4], dispVal[5], dispVal[6], dispVal[7], dispVal[8]);
        updatingDisplay = false;
        this->addressed_write(adds_data, ScreenBuffer[ScreenBufferIndex], 16);
        memcpy(CurrentDisplay, ScreenBuffer[ScreenBufferHead], sizeof(uint8_t) * 16);

        ScreenBufferLength--;
        ScreenBufferIndex = (ScreenBufferIndex + 1) & ScreenBufferModulus;

        this->addressed_write(adds_com, new uint8_t[3] { 0xAB, 0xAA, 0xAF }, 3); // Turn on the second SRAM, Shut down the second SRAM, display on
    }

    void Segmented_ePaper::addressed_write(uint8_t address, const uint8_t* data, size_t len)
    {
        this->set_i2c_address(address);
        this->write(data, len);
    }

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
        return Busy_pin_->digital_read();
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
