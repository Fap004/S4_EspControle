#include "McpwmHBridgeDriver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_rom_sys.h"

static const char* TAG = "McpwmHBridgeDriver";

// ──────────────────────────────────────────────────────────────────────────────
McpwmHBridgeDriver::McpwmHBridgeDriver(const Pins& pins, const Config& cfg)
    : pins_(pins), cfg_(cfg)
{}

// ──────────────────────────────────────────────────────────────────────────────
mcpwm_io_signals_t
McpwmHBridgeDriver::ioSignalFrom(mcpwm_timer_t t, mcpwm_operator_t op)
{
    const bool isA = (op == MCPWM_OPR_A);
    switch (t) {
        case MCPWM_TIMER_0: return isA ? MCPWM0A : MCPWM0B;
        case MCPWM_TIMER_1: return isA ? MCPWM1A : MCPWM1B;
        case MCPWM_TIMER_2: return isA ? MCPWM2A : MCPWM2B;
        default:            return MCPWM0A;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
esp_err_t McpwmHBridgeDriver::init()
{
    // 1) GPIO direction (INA / INB / SEL0 en sortie)
    gpio_config_t io = {};
    io.intr_type    = GPIO_INTR_DISABLE;
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;

    const gpio_num_t outs[2] = { pins_.inA, pins_.inB };
    for (gpio_num_t g : outs) {
        if (g != GPIO_NUM_NC) {
            io.pin_bit_mask = (1ULL << g);
            ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config");
        }
    }
    if (pins_.sel0 != GPIO_NUM_NC) {
        io.pin_bit_mask = (1ULL << pins_.sel0);
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config(sel0)");
        gpio_set_level(pins_.sel0, 0);
    }

    // 2) Mapping GPIO PWM → signal MCPWM (MCPWMxA/B selon timer/op)
    const mcpwm_io_signals_t sig = ioSignalFrom(cfg_.timer, cfg_.op);
    ESP_RETURN_ON_ERROR(mcpwm_gpio_init(cfg_.unit, sig, pins_.pwm),
                        TAG, "mcpwm_gpio_init");

    // 3) Configuration MCPWM (fréquence, 0 % au départ)
    mcpwm_config_t m = {};
    m.frequency    = cfg_.freq_hz;
    m.cmpr_a       = 0.0f;
    m.cmpr_b       = 0.0f;
    m.counter_mode = MCPWM_UP_COUNTER;
    m.duty_mode    = MCPWM_DUTY_MODE_0;
    ESP_RETURN_ON_ERROR(mcpwm_init(cfg_.unit, cfg_.timer, &m), TAG, "mcpwm_init");

    mcpwm_start(cfg_.unit, cfg_.timer);

    // 4) État initial
    applyIdle();
    initialized_ = true;
    dir_         = MotorDir::Forward;
    last_duty01_ = 0.0f;

    ESP_LOGI(TAG, "MCPWM init OK (unit=%d,timer=%d,op=%d) pwm_gpio=%d freq=%u Hz",
             (int)cfg_.unit, (int)cfg_.timer, (int)cfg_.op,
             (int)pins_.pwm, (unsigned)cfg_.freq_hz);
    return ESP_OK;
}

// ──────────────────────────────────────────────────────────────────────────────
void McpwmHBridgeDriver::applyIdle()
{
    if (cfg_.idle == DecayMode::Brake) {
        gpio_set_level(pins_.inA, 1);
        gpio_set_level(pins_.inB, 1);
    } else {
        gpio_set_level(pins_.inA, 0);
        gpio_set_level(pins_.inB, 0);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
void McpwmHBridgeDriver::applyPinsForDirection(MotorDir d)
{
    bool forward = (d == MotorDir::Forward);
    if (cfg_.invert_dir) forward = !forward;
    gpio_set_level(pins_.inA, forward ? 1 : 0);
    gpio_set_level(pins_.inB, forward ? 0 : 1);
}

// ──────────────────────────────────────────────────────────────────────────────
void McpwmHBridgeDriver::setDirection(MotorDir d)
{
    if (!initialized_) { ESP_LOGW(TAG, "setDirection() before init()"); return; }
    if (d == MotorDir::Brake) { ESP_LOGW(TAG, "Brake ignored; use brake(true)"); return; }

    if (d != dir_) {
        // Dead-time logiciel : coupe PWM, idle, pause, puis change le sens
        mcpwm_set_duty(cfg_.unit, cfg_.timer, cfg_.op, 0.0f);
        mcpwm_set_duty_type(cfg_.unit, cfg_.timer, cfg_.op, MCPWM_DUTY_MODE_0);
        applyIdle();
        if (cfg_.deadtime_us) esp_rom_delay_us(cfg_.deadtime_us);

        dir_ = d;
        applyPinsForDirection(dir_);
        // duty sera ré-appliqué par le prochain appel à setDuty()
    }
}

// ──────────────────────────────────────────────────────────────────────────────
void McpwmHBridgeDriver::setDuty(float duty01)
{
    if (!initialized_) { ESP_LOGW(TAG, "setDuty() before init()"); return; }

    duty01 = std::clamp(duty01, 0.0f, 1.0f);

    // Dead-zone unifiée : une seule comparaison avec duty_min
    // (zero_eps supprimé pour éviter la double dead-zone 0.05 + 0.08 = 0.13)
    if (duty01 < cfg_.duty_min) {
        last_duty01_ = 0.0f;
        mcpwm_set_duty(cfg_.unit, cfg_.timer, cfg_.op, 0.0f);
        mcpwm_set_duty_type(cfg_.unit, cfg_.timer, cfg_.op, MCPWM_DUTY_MODE_0);
        applyIdle();
        ESP_LOGI(TAG, "dir=%s duty=0.00 (idle)",
                 dir_ == MotorDir::Forward ? "F" : "R");
        return;
    }

    last_duty01_ = duty01;
    applyPinsForDirection(dir_);
    mcpwm_set_duty(cfg_.unit, cfg_.timer, cfg_.op, duty01 * 100.0f);
    mcpwm_set_duty_type(cfg_.unit, cfg_.timer, cfg_.op, MCPWM_DUTY_MODE_0);

    ESP_LOGI(TAG, "dir=%s duty=%.2f",
             dir_ == MotorDir::Forward ? "F" : "R",
             last_duty01_);
}

// ──────────────────────────────────────────────────────────────────────────────
void McpwmHBridgeDriver::setDutySigned(float u)
{
    if (!initialized_) { ESP_LOGW(TAG, "setDutySigned() before init()"); return; }
    u = std::clamp(u, -1.0f, 1.0f);
    if (std::fabs(u) < cfg_.duty_min) { setDuty(0.0f); return; }
    setDirection(u >= 0.0f ? MotorDir::Forward : MotorDir::Reverse);
    setDuty(std::fabs(u));
}

// ──────────────────────────────────────────────────────────────────────────────
void McpwmHBridgeDriver::brake(bool on)
{
    if (!initialized_) { ESP_LOGW(TAG, "brake() before init()"); return; }
    mcpwm_set_duty(cfg_.unit, cfg_.timer, cfg_.op, 0.0f);
    mcpwm_set_duty_type(cfg_.unit, cfg_.timer, cfg_.op, MCPWM_DUTY_MODE_0);
    if (on) { gpio_set_level(pins_.inA, 1); gpio_set_level(pins_.inB, 1); }
    else    { gpio_set_level(pins_.inA, 0); gpio_set_level(pins_.inB, 0); }
}

// ──────────────────────────────────────────────────────────────────────────────
void McpwmHBridgeDriver::stop()
{
    setDuty(0.0f);
}

// ──────────────────────────────────────────────────────────────────────────────
void McpwmHBridgeDriver::setConfig(const Config& cfg) { cfg_ = cfg; }
