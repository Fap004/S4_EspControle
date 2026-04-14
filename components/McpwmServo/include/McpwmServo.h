#pragma once
#include "IServo.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_err.h"

class McpwmServo : public IServo {
public:
    struct Config {
        mcpwm_unit_t  unit      = MCPWM_UNIT_0;
        mcpwm_timer_t timer     = MCPWM_TIMER_2;
        uint32_t      freq_hz   = 50;
        float         duty_init = 0.0f;
        uint32_t      pw_min_us = 500;
        uint32_t      pw_max_us = 2500;
        float         angle_max = 135.0f;
    };

    McpwmServo(gpio_num_t pin, const Config& cfg = s_default_cfg);

    esp_err_t init();

    // IServo
    void setAngleDeg(float angle) override;

    // Fige le signal au neutre (1500µs) sans couper le PWM
    void holdNeutral();

    // Bas niveau
    void setDuty(float duty01);
    void setPulseUs(uint32_t us);

private:
    static const Config s_default_cfg;
    gpio_num_t pin_;
    Config     cfg_;
    bool       initialized_ = false;
};