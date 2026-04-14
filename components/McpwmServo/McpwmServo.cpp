#include "McpwmServo.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>

static const char* TAG = "McpwmServo";
const McpwmServo::Config McpwmServo::s_default_cfg{};

McpwmServo::McpwmServo(gpio_num_t pin, const Config& cfg)
    : pin_(pin), cfg_(cfg) {}

esp_err_t McpwmServo::init()
{
    const mcpwm_io_signals_t sig =
        (cfg_.timer == MCPWM_TIMER_0) ? MCPWM0A :
        (cfg_.timer == MCPWM_TIMER_1) ? MCPWM1A : MCPWM2A;

    ESP_RETURN_ON_ERROR(
        mcpwm_gpio_init(cfg_.unit, sig, pin_),
        TAG, "mcpwm_gpio_init");

    mcpwm_config_t m = {};
    m.frequency    = cfg_.freq_hz;
    m.cmpr_a       = 0.0f;
    m.cmpr_b       = 0.0f;
    m.counter_mode = MCPWM_UP_COUNTER;
    m.duty_mode    = MCPWM_DUTY_MODE_0;

    ESP_RETURN_ON_ERROR(
        mcpwm_init(cfg_.unit, cfg_.timer, &m),
        TAG, "mcpwm_init");

    mcpwm_start(cfg_.unit, cfg_.timer);
    initialized_ = true;

    // ✅ Neutre 1500µs dès le boot → servo centré et stable
    holdNeutral();
    vTaskDelay(pdMS_TO_TICKS(300));

    const uint32_t neutral_us = (cfg_.pw_min_us + cfg_.pw_max_us) / 2;
    ESP_LOGI(TAG, "Servo OK: gpio=%d freq=%uHz pw=[%u,%u]us neutral=%uus",
        (int)pin_, (unsigned)cfg_.freq_hz,
        cfg_.pw_min_us, cfg_.pw_max_us, neutral_us);

    return ESP_OK;
}

void McpwmServo::holdNeutral()
{
    if (!initialized_) return;
    const uint32_t neutral_us = (cfg_.pw_min_us + cfg_.pw_max_us) / 2;
    setPulseUs(neutral_us);
}

void McpwmServo::setDuty(float duty01)
{
    if (!initialized_) return;
    duty01 = std::clamp(duty01, 0.0f, 1.0f);
    mcpwm_set_duty(cfg_.unit, cfg_.timer, MCPWM_OPR_A, duty01 * 100.0f);
    mcpwm_set_duty_type(cfg_.unit, cfg_.timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void McpwmServo::setPulseUs(uint32_t us)
{
    if (!initialized_ || cfg_.freq_hz == 0) return;
    const float period_us = 1'000'000.0f / (float)cfg_.freq_hz;
    const float duty = (float)us / period_us;
    setDuty(duty);
}

void McpwmServo::setAngleDeg(float angle)
{
    if (!initialized_) return;
    angle = std::clamp(angle, -cfg_.angle_max, +cfg_.angle_max);
    const float t = (angle + cfg_.angle_max) / (2.0f * cfg_.angle_max);
    const uint32_t us =
        (uint32_t)(cfg_.pw_min_us + t * (float)(cfg_.pw_max_us - cfg_.pw_min_us));
    setPulseUs(us);
}