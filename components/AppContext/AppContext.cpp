#include "AppContext.h"
#include "esp_log.h"

// Pour renseigner les champs unit/timer/op du driver MCPWM (API legacy)
#include "driver/mcpwm.h"

static const char* TAG = "AppContext";

esp_err_t appctx_init(AppContext& ctx)
{
    // ======================
    //  MCPWM - Moteur gauche
    // ======================
    ctx.L_cfg.unit        = MCPWM_UNIT_0;
    ctx.L_cfg.timer       = MCPWM_TIMER_0;     // PWM0A
    ctx.L_cfg.op          = MCPWM_OPR_A;
    ctx.L_cfg.freq_hz     = 20000;             // 20 kHz (VNH7070BAS OK)
    ctx.L_cfg.duty_min    = 0.10f;             // 10% : vaincre frottements/quantification
    ctx.L_cfg.zero_eps    = 0.05f;             // 5% : zone morte autour de 0
    ctx.L_cfg.deadtime_us = 150;               // dead-time logiciel lors inversion
    ctx.L_cfg.idle        = McpwmHBridgeDriver::DecayMode::Coast;
    ctx.L_cfg.invert_dir  = false;

    // =====================
    //  MCPWM - Moteur droit
    // =====================
    ctx.R_cfg.unit        = MCPWM_UNIT_0;
    ctx.R_cfg.timer       = MCPWM_TIMER_1;     // PWM1A (évite conflit avec gauche)
    ctx.R_cfg.op          = MCPWM_OPR_A;
    ctx.R_cfg.freq_hz     = 20000;
    ctx.R_cfg.duty_min    = 0.10f;
    ctx.R_cfg.zero_eps    = 0.05f;
    ctx.R_cfg.deadtime_us = 150;
    ctx.R_cfg.idle        = McpwmHBridgeDriver::DecayMode::Coast;
    ctx.R_cfg.invert_dir  = false;

    // === Init drivers moteurs ===
    ESP_ERROR_CHECK(ctx.motor_left.init());
    ESP_ERROR_CHECK(ctx.motor_right.init());

    // === Init encodeurs (idempotent) ===
    ESP_ERROR_CHECK(ctx.enc_left.init());
    ESP_ERROR_CHECK(ctx.enc_right.init());

    // === Queues ===
    ctx.q_cmd_vw = xQueueCreate(/*length*/8, sizeof(AppContext::CmdVW));
    ctx.q_tlm    = xQueueCreate(/*length*/8, sizeof(AppContext::Telemetry));
    if (!ctx.q_cmd_vw || !ctx.q_tlm) {
        ESP_LOGE(TAG, "Queue creation failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "AppContext init OK (MCPWM @ %u Hz)", ctx.L_cfg.freq_hz);
    return ESP_OK;
}