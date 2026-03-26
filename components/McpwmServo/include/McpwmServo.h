#pragma once
#include "IServo.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_err.h"

class McpwmServo : public IServo {
public:
    struct Config {
        mcpwm_unit_t  unit      = MCPWM_UNIT_0;
        mcpwm_timer_t timer     = MCPWM_TIMER_2;   // TIMER_0 et 1 pris par les moteurs
        uint32_t      freq_hz   = 500;              // 500 Hz comme demandé
        float         duty_init = 0.50f;            // 50% au démarrage = position neutre
        // Limites physiques du servo en microsecondes (pour setAngleDeg)
        uint32_t      pw_min_us = 1000;             // -maxDeg → 1 ms
        uint32_t      pw_max_us = 2000;             // +maxDeg → 2 ms
        float         angle_max = 30.0f;            // degrés max de chaque côté
    };

    McpwmServo(gpio_num_t pin, const Config& cfg = s_default_cfg);

    esp_err_t init();

    // IServo — angle en degrés, ex: -30..+30
    void setAngleDeg(float angle) override;

    // Accès direct au duty cycle [0.0 .. 1.0]
    void setDuty(float duty01);

    // Accès direct à la largeur d'impulsion en µs
    void setPulseUs(uint32_t us);

private:
    static const Config s_default_cfg;
    gpio_num_t pin_;
    Config     cfg_;
    bool       initialized_ = false;
};