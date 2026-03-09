#include "AppContext.h"
#include "esp_log.h"

static const char* TAG = "AppContext";

esp_err_t appctx_init(AppContext& ctx)
{
    // LEDC gauche
    ctx.L_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ctx.L_cfg.timer      = LEDC_TIMER_0;
    ctx.L_cfg.channel    = LEDC_CHANNEL_0;
    ctx.L_cfg.freq_hz    = 20000;
    ctx.L_cfg.duty_bits  = 10;
    ctx.L_cfg.idle       = LedcHBridgeDriver::DecayMode::Coast;

    // LEDC droit
    ctx.R_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ctx.R_cfg.timer      = LEDC_TIMER_0;     // même timer
    ctx.R_cfg.channel    = LEDC_CHANNEL_1;   // canal différent
    ctx.R_cfg.freq_hz    = 20000;
    ctx.R_cfg.duty_bits  = 10;
    ctx.R_cfg.idle       = LedcHBridgeDriver::DecayMode::Coast;

    // Init moteurs
    ESP_ERROR_CHECK(ctx.motor_left.init());
    ESP_ERROR_CHECK(ctx.motor_right.init());

    // Init encodeurs (idempotent)
    ESP_ERROR_CHECK(ctx.enc_left.init());
    ESP_ERROR_CHECK(ctx.enc_right.init());

    // Queues
    ctx.q_cmd_vw = xQueueCreate(/*length*/8, sizeof(AppContext::CmdVW));
    ctx.q_tlm    = xQueueCreate(/*length*/8, sizeof(AppContext::Telemetry));
    if (!ctx.q_cmd_vw || !ctx.q_tlm) {
        ESP_LOGE(TAG, "Queue creation failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}