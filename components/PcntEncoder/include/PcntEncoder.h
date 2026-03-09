#pragma once
#include <stdint.h>

// IMPORTANTS : doivent être avant tout usage de gpio_num_t ou pcnt types
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

#include "IEncoder.h"

class PcntEncoder : public IEncoder {
//class PcntEncoder {
public:
    PcntEncoder(gpio_num_t pulsePin, gpio_num_t ctrlPin,
                float ticks_per_rev, uint32_t glitch_ns = 0);

    esp_err_t init();
    void reset();

    // Fiable, cumulatif
    int32_t getDelta();

    inline int32_t getAndClearDeltaTicks() { return getDelta(); }

    inline float ticksPerRev() const { return ticks_per_rev_; }
    inline void  setTicksPerRev(float tpr) { ticks_per_rev_ = tpr; }

private:
    void recenterIfNearLimit(int current_total);

    gpio_num_t pulse_pin_;
    gpio_num_t ctrl_pin_;
    float      ticks_per_rev_;
    uint32_t   glitch_ns_;

    pcnt_unit_handle_t    unit_    = nullptr;
    pcnt_channel_handle_t channel_ = nullptr;

    int32_t last_total_ = 0;
};
