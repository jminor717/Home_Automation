#include "esphome.h"
using namespace esphome;

class PressurePeakFilter {
public:
    struct settings_t {
        int filter_queue_depth = 240;
        int send_every = 240;
        float outlier_threshold = 0.03;
        int amount_to_send_after_outlier = 20;
    } settings;

    struct data_t {
        std::deque<float> filter_queue;
        int last_sent = 60;
        int needs_sent_after_outlier = 0;
    } data;

    optional<float> apply_filter(float x)
    {
        float sum = 0;
        size_t valid_count = 0;
        // find running average to check for outliers
        for (auto v : data.filter_queue) {
            if (!std::isnan(v)) {
                sum += v;
                valid_count++;
            }
        }
        float average = NAN;
        if (valid_count) {
            average = sum / valid_count;
        } else {
            // if no values in queue, just return the current value and add to queue
            data.filter_queue.push_back(x);
            return x;
        }

        data.filter_queue.push_back(x);
        // if the current value is an outlier, we will send the next 20 values
        if (std::abs(x - average) > settings.outlier_threshold) {
            ESP_LOGVV("filt", "outL: %f", x);
            data.needs_sent_after_outlier = settings.amount_to_send_after_outlier;
            data.last_sent = 0;
            return x;
        }

        if (data.needs_sent_after_outlier > 0) {
            data.needs_sent_after_outlier--;
            ESP_LOGVV("filt", "outL sent: %f", x);
            return x;
        }

        // if we have enough values in the queue, pop the oldest one
        if (data.filter_queue.size() >= settings.filter_queue_depth) {
            data.filter_queue.pop_front();
        }
        // send the average every 30 seconds, this average doesn't include the current X but this should be good enough
        if (data.last_sent >= settings.send_every) {
            data.last_sent = 0;
            ESP_LOGVV("filt", "AVEe: %f", average);
            return average;
        } else {
            data.last_sent++;
        }
        return {};
    }

} peakFilter;