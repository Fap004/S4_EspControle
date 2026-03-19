#pragma once
#include <stdint.h>
#include <algorithm>
#include <cmath>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_err.h"
#include "IMotorDriver.h"

#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC ((gpio_num_t)-1)
#endif

class McpwmHBridgeDriver : public IMotorDriver
{
public:
    enum class DecayMode { Coast, Brake };

    struct Pins {
        gpio_num_t inA;
        gpio_num_t inB;
        gpio_num_t pwm;
        gpio_num_t sel0 = GPIO_NUM_NC;   // optionnel (VNH7070)
    };

    struct Config {
        mcpwm_unit_t     unit        = MCPWM_UNIT_0;
        mcpwm_timer_t    timer       = MCPWM_TIMER_0;
        mcpwm_operator_t op          = MCPWM_OPR_A;
        uint32_t         freq_hz     = 20000;
        // ⚠️ duty_min et zero_eps DOIVENT être identiques pour éviter une double
        //    dead-zone. Cohérent avec WheelController::Config::duty_min = 0.08f.
        float            duty_min    = 0.08f;   // seuil unique : en dessous → idle
        float            zero_eps    = 0.08f;   // gardé pour setDutySigned(), même valeur
        uint32_t         deadtime_us = 150;
        DecayMode        idle        = DecayMode::Coast;
        bool             invert_dir  = false;
    };

    McpwmHBridgeDriver(const Pins& pins, const Config& cfg);

    esp_err_t init();

    // IMotorDriver
    void setDuty(float duty01) override;        // [0..1]
    void setDirection(MotorDir dir) override;   // Forward / Reverse

    // Helpers
    void setDutySigned(float u);                // [-1..+1]
    void brake(bool on);
    void stop();
    void setConfig(const Config& cfg);

private:
    void applyPinsForDirection(MotorDir d);
    void applyIdle();
    static mcpwm_io_signals_t ioSignalFrom(mcpwm_timer_t t, mcpwm_operator_t op);

    Pins     pins_;
    Config   cfg_;
    bool     initialized_ = false;
    MotorDir dir_         = MotorDir::Forward;
    float    last_duty01_ = 0.0f;
};
