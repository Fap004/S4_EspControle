#pragma once

#include "IEncoder.h"          // ton interface (disponible dans le composant)
#include "esp_err.h"
#include <cstdint>

// ESP-IDF v5.x : bons headers
#include "driver/gpio.h"       // gpio_num_t
#include "driver/pulse_cnt.h"  // driver PCNT moderne (handles & APIs)

/**
 * Encodeur quadrature basé sur PCNT (pulse_cnt).
 * A = pulse (edge), B = control (level)
 */
class PcntEncoder : public IEncoder {
public:
    // pulsePin = A, ctrlPin = B
    PcntEncoder(gpio_num_t pulsePin, gpio_num_t ctrlPin,
                float ticks_per_rev, uint32_t glitch_ns = 1000);

    esp_err_t init();
    void reset() override;
    int32_t getAndClearDeltaTicks() override;
    float ticksPerRev() const override { return ticks_per_rev_; }

private:
    gpio_num_t pulse_pin_;
    gpio_num_t ctrl_pin_;
    float      ticks_per_rev_;
    uint32_t   glitch_ns_;

    pcnt_unit_handle_t    unit_    = nullptr;
    pcnt_channel_handle_t ch_edge_ = nullptr; // edge = A, level = B
};