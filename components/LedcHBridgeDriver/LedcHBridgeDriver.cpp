#include "LedcHBridgeDriver.h"

#include <cmath>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_idf_version.h"

static const char* TAG = "LedcHBridgeDriver";

// Évite de reconfigurer plusieurs fois le même timer LEDC
static bool s_timer_configured[LEDC_SPEED_MODE_MAX][LEDC_TIMER_MAX] = {};

esp_err_t LedcHBridgeDriver::ensureTimerConfigured(ledc_mode_t mode,
                                                   ledc_timer_t timer,
                                                   uint32_t freq,
                                                   uint8_t duty_bits)
{
    if (s_timer_configured[mode][timer]) return ESP_OK;

    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode      = mode;
    tcfg.timer_num       = timer;
    tcfg.duty_resolution = static_cast<ledc_timer_bit_t>(duty_bits);
    tcfg.freq_hz         = freq;
    tcfg.clk_cfg         = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&tcfg);
    if (err == ESP_OK) {
        s_timer_configured[mode][timer] = true;
    } else {
        ESP_LOGE(TAG, "LEDC timer config failed (mode=%d,timer=%d): %d",
                 (int)mode, (int)timer, (int)err);
    }
    return err;
}

LedcHBridgeDriver::LedcHBridgeDriver(const Pins& pins, const Config& cfg)
: pins_(pins), cfg_(cfg)
{}

esp_err_t LedcHBridgeDriver::init()
{
    // 1) GPIO out
    gpio_config_t io = {};
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;

    const gpio_num_t outs[3] = { pins_.inA, pins_.inB, pins_.pwm };
    for (gpio_num_t g : outs) {
        if (g != GPIO_NUM_NC) {
            io.pin_bit_mask = 1ULL << g;
            ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io));
        }
    }
    if (pins_.sel0 != GPIO_NUM_NC) {
        io.pin_bit_mask = 1ULL << pins_.sel0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io));
        gpio_set_level(pins_.sel0, 0); // adapter si ton shield demande 1
    }

    // 2) Timer LEDC
    esp_err_t err = ensureTimerConfigured(cfg_.speed_mode, cfg_.timer, cfg_.freq_hz, cfg_.duty_bits);
    if (err != ESP_OK) return err;

    // 3) Canal LEDC
    ledc_channel_config_t ch = {};
    ch.gpio_num    = pins_.pwm;
    ch.speed_mode  = cfg_.speed_mode;
    ch.channel     = cfg_.channel;
    ch.timer_sel   = cfg_.timer;
    ch.duty        = 0;
    ch.hpoint      = 0;
    ch.intr_type   = LEDC_INTR_DISABLE;
    err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed (ch=%d): %d", (int)cfg_.channel, (int)err);
        return err;
    }

    // 4) État idle (coast/brake)
    if (cfg_.idle == DecayMode::Brake) {
        gpio_set_level(pins_.inA, 1);
        gpio_set_level(pins_.inB, 1);
    } else {
        gpio_set_level(pins_.inA, 0);
        gpio_set_level(pins_.inB, 0);
    }

    initialized_  = true;
    last_duty01_  = 0.0f;
    dir_          = MotorDir::Forward;
    return ESP_OK;
}

void LedcHBridgeDriver::applyPinsForDirection(MotorDir d)
{
    // Direction logique (avec inversion possible)
    bool forward = (d == MotorDir::Forward);
    if (cfg_.invert_dir) forward = !forward;

    // Pont en H : (1,0) = avant ; (0,1) = arrière
    gpio_set_level(pins_.inA, forward ? 1 : 0);
    gpio_set_level(pins_.inB, forward ? 0 : 1);
}

void LedcHBridgeDriver::setDirection(MotorDir d)
{
    if (!initialized_) {
        ESP_LOGW(TAG, "setDirection() called before init()");
        return;
    }
    // Si on te passe MotorDir::Brake ici, on l’ignore : le freinage se fait via brake(true)
    if (d == MotorDir::Brake) {
        // Option : on pourrait appeler brake(true) si tu veux
        ESP_LOGW(TAG, "MotorDir::Brake ignored in setDirection(); use brake(true) instead");
        return;
    }
    dir_ = d;
    applyPinsForDirection(d);
}

uint32_t LedcHBridgeDriver::dutyToRaw(float duty01) const
{
    duty01 = std::clamp(duty01, 0.0f, 1.0f);
    const uint32_t max = (1u << cfg_.duty_bits) - 1u;
    return static_cast<uint32_t>(std::round(duty01 * max));
}

void LedcHBridgeDriver::setDuty(float duty01)
{
    if (!initialized_) {
        ESP_LOGW(TAG, "setDuty() called before init()");
        return;
    }

    duty01 = std::clamp(duty01, 0.0f, 1.0f);
    last_duty01_ = duty01;

    // duty=0 -> état idle configuré
    if (duty01 == 0.0f) {
        if (cfg_.idle == DecayMode::Brake) {
            gpio_set_level(pins_.inA, 1);
            gpio_set_level(pins_.inB, 1);
        } else {
            gpio_set_level(pins_.inA, 0);
            gpio_set_level(pins_.inB, 0);
        }
    } else {
        // Reposer les broches selon le sens courant
        applyPinsForDirection(dir_);
    }

    const uint32_t raw = dutyToRaw(duty01);
    esp_err_t err = ledc_set_duty(cfg_.speed_mode, cfg_.channel, raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed: %d", (int)err);
        return;
    }
    err = ledc_update_duty(cfg_.speed_mode, cfg_.channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed: %d", (int)err);
        return;
    }
}

void LedcHBridgeDriver::setDutySigned(float dutySigned)
{
    if (!initialized_) {
        ESP_LOGW(TAG, "setDutySigned() called before init()");
        return;
    }

    dutySigned = std::clamp(dutySigned, -1.0f, 1.0f);
    if (dutySigned == 0.0f) {
        setDuty(0.0f);
        return;
    }
    const bool fwd = (dutySigned >= 0.0f);
    setDirection(fwd ? MotorDir::Forward : MotorDir::Reverse);
    setDuty(std::fabs(dutySigned));
}

void LedcHBridgeDriver::brake(bool on)
{
    if (!initialized_) {
        ESP_LOGW(TAG, "brake() called before init()");
        return;
    }
    // Mettre duty=0 avant de toucher aux broches
    setDuty(0.0f);

    if (on) {
        gpio_set_level(pins_.inA, 1);
        gpio_set_level(pins_.inB, 1);
    } else {
        gpio_set_level(pins_.inA, 0);
        gpio_set_level(pins_.inB, 0);
    }
}

void LedcHBridgeDriver::stop()
{
    setDuty(0.0f); // appliquera idle (coast/brake) selon cfg
}