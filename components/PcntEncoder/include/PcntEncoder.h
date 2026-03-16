#pragma once
#include <stdint.h>
#include <algorithm>
#include <cmath>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_check.h"

// Optionnel : si tu as une interface commune
// #include "IEncoder.h"
// class PcntEncoderQuadrature : public IEncoder {
class PcntEncoder {
public:
    /**
     * Quadrature A/B via PCNT v5 (2 canaux).
     *
     * @param pinA          GPIO voie A
     * @param pinB          GPIO voie B
     * @param ticks_per_rev TPR par tour d’ARBRE MOTEUR (ex: 17 PPR × 4 = 68 ticks/turn si x4)
     * @param glitch_ns     Filtre anti-glitch (ns). Reco: 100000–200000 (100–200 µs). 0 = off.
     * @param use_pullups   Active les pull-ups internes sur A/B
     * @param inverted      Inverse le signe global (si roue/montage inversés)
     * @param high_limit    Limite HW PCNT (par défaut ±32767)
     * @param low_limit
     */
    PcntEncoder(gpio_num_t pinA, gpio_num_t pinB,
                          float ticks_per_rev,
                          uint32_t glitch_ns = 150000,
                          bool use_pullups   = true,
                          bool inverted      = false,
                          int high_limit     =  32767,
                          int low_limit      = -32768);

    // Cycle de vie
    esp_err_t init();
    void      deinit();
    void      reset();

    // Lecture delta ticks depuis le dernier appel (protégé overflow, A+B)
    int32_t   getDelta();

    // Total SW 64 bits (depuis reset)
    inline int64_t totalTicks() const { return accum_total_; }

    // Vitesse RPM (LPF exponentiel si lpf=true). Nécessite dt réel > 0.
    float     rpm(float dt_s, bool lpf = true);

    // Réglages
    inline void  setInverted(bool inv)        { inverted_ = inv; }
    inline bool  inverted() const             { return inverted_; }
    inline void  setGlitchNs(uint32_t ns)     { glitch_ns_ = ns; }
    inline void  setLpAlpha(float a)          { lp_alpha_ = std::clamp(a, 0.0f, 1.0f); }
    inline float lpAlpha() const              { return lp_alpha_; }
    inline float ticksPerRev() const          { return ticks_per_rev_; }
    inline void  setTicksPerRev(float tpr)    { if (tpr > 0.f) ticks_per_rev_ = tpr; }

    int32_t getAndClearDeltaTicks();

private:
    esp_err_t configGpioInputs_();
    void      recenterIfNearLimit_(int rawA, int rawB);

private:
    // GPIOs
    gpio_num_t pinA_;
    gpio_num_t pinB_;

    // Paramètres
    float     ticks_per_rev_;
    uint32_t  glitch_ns_;
    bool      use_pullups_;
    bool      inverted_;
    int       high_limit_;
    int       low_limit_;

    // PCNT
    pcnt_unit_handle_t    unitA_   = nullptr;
    pcnt_unit_handle_t    unitB_   = nullptr;
    pcnt_channel_handle_t chA_     = nullptr;
    pcnt_channel_handle_t chB_     = nullptr;

    // États
    int   last_raw_A_  = 0;
    int   last_raw_B_  = 0;
    int64_t accum_total_ = 0;

    float rpm_lpf_     = 0.0f;
    float lp_alpha_    = 0.70f;
    bool  started_     = false;
};