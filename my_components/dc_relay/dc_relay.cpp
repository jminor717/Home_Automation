#include "dc_relay.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dc_relay {

    static const char* const TAG = "Dc_Relay";

    void Dc_Relay::setup()
    {
        // this->Busy_pin_->setup();
        // this->Reset_pin_->setup();
        //this->stop_poller();
    }

    // void Dc_Relay::loop()
    // {
    // }

    void Dc_Relay::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Dc_Relay reader:");
    //     LOG_PIN("  Reset pin: ", this->Reset_pin_);
    //     LOG_PIN("  Busy : ", this->Busy_pin_);
    }

    /*

        xTaskCreate(I2SAudioSpeaker::player_task, "speaker_task", 8192, (void *) this, 0, &this->player_task_handle_);

        void I2SAudioSpeaker::player_task(void *params)


    */

    void Dc_Relay::update()
    {
        // ESP_LOGV(TAG, "Received sensor reading with sequence #%d", sensor_reading.sequence_num);
    }

    // float Circuit::get_calibrated_power(int32_t raw_power) const
    // {
    //     return (raw_power * 1.02) / correction_factor;
    // }

} // namespace dc_relay
} // namespace esphome
