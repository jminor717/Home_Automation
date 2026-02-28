#ifndef pressure_peak_filter_h
#define pressure_peak_filter_h

#include "esphome.h"
using namespace esphome;

class PressurePeakFilter {
public:
    struct settings_t {
        int filter_queue_depth = 240;
        int send_every = 240;
        float outlier_threshold = 0.06;
        int amount_to_send_after_outlier = 20;
        int amount_to_send_before_outlier = 0;
        float ema_alpha = -1; // if value is negative use simple average instead of ema
    } settings;

    struct data_t {
        std::deque<float> filter_queue;
        int last_sent = 60;
        int needs_sent_after_outlier = 0;
        float ema_value = NAN;
    } data;

    PressurePeakFilter(int send_every, float outlier_threshold, int amount_to_send_after_outlier, int amount_to_send_before_outlier, float ema_alpha)
    {
        settings.ema_alpha = ema_alpha;
        settings.filter_queue_depth = send_every;
        settings.send_every = send_every;
        settings.outlier_threshold = outlier_threshold;
        settings.amount_to_send_after_outlier = amount_to_send_after_outlier;
        settings.amount_to_send_before_outlier = amount_to_send_before_outlier;
        data.last_sent = send_every / 2; // start by sending an average after half the interval
    }

    PressurePeakFilter(int depth, int send_every, float outlier_threshold, int amount_to_send_after_outlier)
    {
        settings.filter_queue_depth = depth;
        settings.send_every = send_every;
        settings.outlier_threshold = outlier_threshold;
        settings.amount_to_send_after_outlier = amount_to_send_after_outlier;
        data.last_sent = send_every / 2; // start by sending an average after half the interval
    }

    optional<float> apply_filter(float x)
    {
        float average = NAN;

        if (settings.ema_alpha > 0) {
            // calculate ema
            if (std::isnan(data.ema_value)) {
                data.ema_value = x;
                data.filter_queue.push_back(x);
                return x; // if no values in queue, just return the current value and add to queue
            }
            average = (x * settings.ema_alpha) + (data.ema_value * (1.0 - settings.ema_alpha));
            data.ema_value = average;
        } else {
            // calculate simple average
            float sum = 0;
            size_t valid_count = 0;
            // find running average to check for outliers
            for (auto v : data.filter_queue) {
                if (!std::isnan(v)) {
                    sum += v;
                    valid_count++;
                }
            }
            if (valid_count) {
                average = sum / valid_count;
            } else {
                data.filter_queue.push_back(x);
                return x; // if no values in queue, just return the current value and add to queue
            }
        }


        data.filter_queue.push_back(x);
        // if we have enough values in the queue, pop the oldest one
        if (data.filter_queue.size() >= settings.filter_queue_depth) {
            data.filter_queue.pop_front();
        }

        // if the current value is an outlier, we will send the next 20 values
        if (std::abs(x - average) > settings.outlier_threshold) {
            ESP_LOGVV("filt", "outL: %f", x);
            data.needs_sent_after_outlier = settings.amount_to_send_after_outlier + settings.amount_to_send_before_outlier;
            data.last_sent = 0;
            return data.filter_queue[(data.filter_queue.size() - 1) - settings.amount_to_send_before_outlier]; // send the value before the outlier
        }

        if (data.needs_sent_after_outlier > 0) {
            data.needs_sent_after_outlier--;
            ESP_LOGVV("filt", "outL sent: %f", x);
            return data.filter_queue[(data.filter_queue.size() - 1) - settings.amount_to_send_before_outlier]; // send the value before the outlier
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
};

#endif
// End of Header file