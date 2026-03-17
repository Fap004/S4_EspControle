#pragma once
#include <cstdint>
#include <algorithm>
#include <cmath>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_check.h"

class PcntEncoder {
public:
    PcntEncoder(gpio_num_t pinA, gpio_num_t pinB,
                float    ticks_per_rev,
                uint32_t glitch_ns   = 150000,
                bool     use_pullups = true,
                bool     inverted    = false,
                int      high_limit  =  32767,
                int      low_limit   = -32768);

    // Cycle de vie
    esp_err_t init();
    void      deinit();
    void      reset();

    // Lecture
    int32_t getDelta();                         // delta ticks depuis le dernier appel
    inline int64_t totalTicks() const { return accum_total_; }

    // Vitesse (dt>0), LPF exponentiel si lpf=true
    float rpm(float dt_s, bool lpf = true);

    // Réglages runtime
    inline void  setInverted(bool inv)     { inverted_ = inv; }
    inline bool  inverted()  const         { return inverted_; }
    inline void  setGlitchNs(uint32_t ns)  { glitch_ns_ = ns; }
    inline void  setLpAlpha(float a)       { lp_alpha_ = std::clamp(a, 0.0f, 1.0f); }
    inline float lpAlpha()   const         { return lp_alpha_; }
    inline float ticksPerRev() const       { return ticks_per_rev_; }
    inline void  setTicksPerRev(float tpr) { if (tpr > 0.f) ticks_per_rev_ = tpr; }

    // Compat
    int32_t getAndClearDeltaTicks();

    inline bool isInitialized() const { return unitA_ != nullptr; }

private:
    esp_err_t configGpioInputs_();
    void      recenterIfNearLimit_(int raw);

private:
    // GPIOs
    gpio_num_t pinA_;
    gpio_num_t pinB_;

    // Paramètres
    float    ticks_per_rev_;
    uint32_t glitch_ns_;
    bool     use_pullups_;
    bool     inverted_;
    int      high_limit_;
    int      low_limit_;

    // PCNT — une seule unité, deux canaux
    pcnt_unit_handle_t    unitA_ = nullptr;
    pcnt_channel_handle_t chA_   = nullptr;
    pcnt_channel_handle_t chB_   = nullptr;

    // État
    int     last_raw_A_  = 0;
    int64_t accum_total_ = 0;
    float   rpm_lpf_     = 0.0f;
    float   lp_alpha_    = 0.70f;
    bool    started_     = false;
};