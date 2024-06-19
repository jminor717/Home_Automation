#include "pic_programer.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pic_programer {

    static const char* const TAG = "Pic_Programer";

    void Pic_Programer::setup()
    {
        this->Busy_pin_->setup();
        this->Reset_pin_->setup();
    }

    void Pic_Programer::loop()
    {
    }

    void Pic_Programer::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Pic_Programer reader:");
        LOG_PIN("  Reset pin: ", this->Reset_pin_);
        LOG_PIN("  Busy : ", this->Busy_pin_);
    }

} // namespace pic_programer
} // namespace esphome
