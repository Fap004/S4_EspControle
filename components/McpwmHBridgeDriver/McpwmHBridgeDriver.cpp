#include "McpwmHBridgeDriver.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_check.h"     // pour ESP_RETURN_ON_ERROR(...)
#include "esp_rom_sys.h"   // pour esp_rom_delay_us(...)


static const char* TAG = "McpwmHBridgeDriver";

McpwmHBridgeDriver::McpwmHBridgeDriver(const Pins& pins, const Config& cfg)
: pins_(pins), cfg_(cfg)
{}

esp_err_t McpwmHBridgeDriver::init()
{
    // 1) GPIO direction
    gpio_config_t io = {};
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;

    const gpio_num_t outs[2] = { pins_.inA, pins_.inB };
    for (gpio_num_t g : outs) {
        if (g != GPIO_NUM_NC) {
            io.pin_bit_mask = 1ULL << g;
            ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config");
        }
    }
    if (pins_.sel0 != GPIO_NUM_NC) {
        io.pin_bit_mask = 1ULL << pins_.sel0;
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config(sel0)");
        gpio_set_level(pins_.sel0, 0);
    }

    // 2) GPIO PWM
    ESP_RETURN_ON_ERROR(
        mcpwm_gpio_init(cfg_.unit, MCPWM0A, pins_.pwm), TAG, "mcpwm_gpio_init");

    // 3) MCPWM configuration (freq, 0% duty au départ)
    mcpwm_config_t m = {};
    m.frequency = cfg_.freq_hz;         // 20 kHz (conforme VNH7070) [3](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html)
    m.cmpr_a   = 0.0f;                  // duty A en %
    m.cmpr_b   = 0.0f;                  // non utilisé
    m.counter_mode = MCPWM_UP_COUNTER;
    m.duty_mode    = MCPWM_DUTY_MODE_0;
    ESP_RETURN_ON_ERROR(mcpwm_init(cfg_.unit, cfg_.timer, &m), TAG, "mcpwm_init");

    // 4) idle initial
    applyIdle();

    initialized_ = true;
    dir_ = MotorDir::Forward;
    last_duty01_ = 0.0f;
    return ESP_OK;
}

void McpwmHBridgeDriver::applyIdle()
{
    if (cfg_.idle == DecayMode::Brake) { gpio_set_level(pins_.inA, 1); gpio_set_level(pins_.inB, 1); }
    else                               { gpio_set_level(pins_.inA, 0); gpio_set_level(pins_.inB, 0); }
}

void McpwmHBridgeDriver::applyPinsForDirection(MotorDir d)
{
    bool forward = (d == MotorDir::Forward);
    if (cfg_.invert_dir) forward = !forward;
    gpio_set_level(pins_.inA, forward ? 1 : 0);
    gpio_set_level(pins_.inB, forward ? 0 : 1);
}

void McpwmHBridgeDriver::setDirection(MotorDir d)
{
    if (!initialized_) { ESP_LOGW(TAG, "setDirection() before init()"); return; }
    if (d == MotorDir::Brake) { ESP_LOGW(TAG, "Brake ignored; use brake(true)"); return; }
    if (d != dir_) {
        // dead-time logiciel: coupe PWM (0%), idle, petite pause, puis change le sens
        mcpwm_set_duty   (cfg_.unit, cfg_.timer, cfg_.op, 0.0f);
        mcpwm_set_duty_type(cfg_.unit, cfg_.timer, cfg_.op, MCPWM_DUTY_MODE_0);
        applyIdle();
        if (cfg_.deadtime_us) esp_rom_delay_us(cfg_.deadtime_us);
        dir_ = d;
        applyPinsForDirection(dir_);
        // réappliqué lors du prochain setDuty()
    }
}

void McpwmHBridgeDriver::setDuty(float duty01)
{
    if (!initialized_) { ESP_LOGW(TAG, "setDuty() before init()"); return; }
    duty01 = std::clamp(duty01, 0.0f, 1.0f);
    if (duty01 < cfg_.zero_eps) {
        last_duty01_ = 0.0f;
        mcpwm_set_duty(cfg_.unit, cfg_.timer, cfg_.op, 0.0f);
        mcpwm_set_duty_type(cfg_.unit, cfg_.timer, cfg_.op, MCPWM_DUTY_MODE_0);
        applyIdle();
        return;
    }
    duty01 = std::max(duty01, cfg_.duty_min);
    last_duty01_ = duty01;

    applyPinsForDirection(dir_);
    // MCPWM attend un pourcentage (0..100)
    mcpwm_set_duty(cfg_.unit, cfg_.timer, cfg_.op, duty01 * 100.0f);
    mcpwm_set_duty_type(cfg_.unit, cfg_.timer, cfg_.op, MCPWM_DUTY_MODE_0);
}

void McpwmHBridgeDriver::setDutySigned(float u)
{
    if (!initialized_) { ESP_LOGW(TAG, "setDutySigned() before init()"); return; }
    u = std::clamp(u, -1.0f, 1.0f);
    if (std::fabs(u) < cfg_.zero_eps) { setDuty(0.0f); return; }
    setDirection(u >= 0.0f ? MotorDir::Forward : MotorDir::Reverse);
    setDuty(std::fabs(u));
}

void McpwmHBridgeDriver::brake(bool on)
{
    if (!initialized_) { ESP_LOGW(TAG, "brake() before init()"); return; }
    // coupe PWM puis force l’état
    mcpwm_set_duty(cfg_.unit, cfg_.timer, cfg_.op, 0.0f);
    mcpwm_set_duty_type(cfg_.unit, cfg_.timer, cfg_.op, MCPWM_DUTY_MODE_0);
    if (on) { gpio_set_level(pins_.inA, 1); gpio_set_level(pins_.inB, 1); }
    else    { gpio_set_level(pins_.inA, 0); gpio_set_level(pins_.inB, 0); }
}

void McpwmHBridgeDriver::stop()
{
    setDuty(0.0f);
}