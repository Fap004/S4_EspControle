#pragma once
#include <stdint.h>
#include <algorithm>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "IMotorDriver.h"   // enum class MotorDir { Forward, Reverse, Brake }; + interface

#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC ((gpio_num_t)-1)
#endif

// Driver LEDC pour pont en H (L298N/TB6612/...)
// - INA / INB : broches de sens
// - PWM      : sortie PWM LEDC
// - SEL0     : optionnel (certaines cartes)
class LedcHBridgeDriver : public IMotorDriver
{
public:
    enum class DecayMode { Coast, Brake }; // Comportement au duty=0

    struct Pins {
        gpio_num_t inA;
        gpio_num_t inB;
        gpio_num_t pwm;
        gpio_num_t sel0 = GPIO_NUM_NC; // optionnel
    };

    struct Config {
        ledc_mode_t    speed_mode = LEDC_LOW_SPEED_MODE; // recommandé
        ledc_timer_t   timer      = LEDC_TIMER_0;         // timer (peut être partagé)
        ledc_channel_t channel    = LEDC_CHANNEL_0;       // canal du moteur
        uint32_t       freq_hz    = 20000;                // 20 kHz
        uint8_t        duty_bits  = 10;                   // 0..1023
        DecayMode      idle       = DecayMode::Coast;     // coast (0,0) ou brake (1,1) au stop
        bool           invert_dir = false;                // inversion logique
    };

    LedcHBridgeDriver(const Pins& pins, const Config& cfg);

    // À appeler au boot
    esp_err_t init();

    // ---------------- IMotorDriver (obligatoires) ----------------
    void setDuty(float duty01) override;           // 0..1 (conserve le sens courant)
    void setDirection(MotorDir dir) override;      // Forward / Reverse / (Brake ignoré ici)
    // -------------------------------------------------------------

    // Helpers non virtuels (optionnels)
    void setDutySigned(float dutySigned);          // [-1..+1] -> signe = sens
    void brake(bool on);                           // force (1,1) si on=true, sinon (0,0)
    void stop();                                   // duty=0 + état idle selon cfg.idle
    float lastDuty01() const { return last_duty01_; }
    MotorDir direction() const { return dir_; }

private:
    static esp_err_t ensureTimerConfigured(ledc_mode_t mode,
                                           ledc_timer_t timer,
                                           uint32_t freq,
                                           uint8_t duty_bits);

    void     applyPinsForDirection(MotorDir d);
    uint32_t dutyToRaw(float duty01) const;

    Pins     pins_;
    Config   cfg_;
    MotorDir dir_         = MotorDir::Forward;
    float    last_duty01_ = 0.0f;
    bool     initialized_ = false;
};