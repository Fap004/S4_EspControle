#include "AppContext.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_pm.h"

static const char* TAG = "AppContext";


esp_err_t appctx_init(AppContext& ctx)
{

    // 1) Config MCPWM
    // Gauche : Unit0 / Timer0 / A
    ctx.L_cfg.unit        = MCPWM_UNIT_0;
    ctx.L_cfg.timer       = MCPWM_TIMER_0;
    ctx.L_cfg.op          = MCPWM_OPR_A;
    ctx.L_cfg.freq_hz     = 20000;
    ctx.L_cfg.duty_min    = 0.10f;
    ctx.L_cfg.zero_eps    = 0.01f;
    ctx.L_cfg.deadtime_us = 150;
    ctx.L_cfg.idle        = McpwmHBridgeDriver::DecayMode::Coast;
    ctx.L_cfg.invert_dir  = false;  // n’inverse PAS le moteur ici

    // Droite : Unit0 / Timer1 / A
    ctx.R_cfg.unit        = MCPWM_UNIT_0;
    ctx.R_cfg.timer       = MCPWM_TIMER_1;
    ctx.R_cfg.op          = MCPWM_OPR_A;
    ctx.R_cfg.freq_hz     = 20000;
    ctx.R_cfg.duty_min    = 0.10f;
    ctx.R_cfg.zero_eps    = 0.01f;
    ctx.R_cfg.deadtime_us = 150;
    ctx.R_cfg.idle        = McpwmHBridgeDriver::DecayMode::Coast;
    ctx.R_cfg.invert_dir  = true;

    // ⚠️ Pousser la config dans les drivers AVANT init()
    ctx.motor_left.setConfig(ctx.L_cfg);
    ctx.motor_right.setConfig(ctx.R_cfg);

    // 2) Init moteurs
    ESP_RETURN_ON_ERROR(ctx.motor_left.init(),  TAG, "motor_left.init");
    ESP_RETURN_ON_ERROR(ctx.motor_right.init(), TAG, "motor_right.init");
    ESP_LOGI(TAG, "MCPWM ready: L{unit=%d,timer=%d,op=%d,pwm=%d} R{unit=%d,timer=%d,op=%d,pwm=%d}",
             (int)ctx.L_cfg.unit, (int)ctx.L_cfg.timer, (int)ctx.L_cfg.op, (int)ctx.L_PWM,
             (int)ctx.R_cfg.unit, (int)ctx.R_cfg.timer, (int)ctx.R_cfg.op, (int)ctx.R_PWM);

    // 3) Init encodeurs puis inversion (appliquée après init à chaque boot)
    ESP_RETURN_ON_ERROR(ctx.enc_left.init(),  TAG, "enc_left.init");
    ESP_RETURN_ON_ERROR(ctx.enc_right.init(), TAG, "enc_right.init");

    ctx.enc_left.setInverted(true);             // ← la gauche lisait négatif en Forward
    //ctx.enc_right.setInverted(true);         // ← active si la droite lit négatif
    
    ESP_LOGI(TAG, "Encoders ready: L{A=%d,B=%d,inv=%d} R{A=%d,B=%d,inv=%d}",
             (int)ctx.L_ENC_A, (int)ctx.L_ENC_B, (int)ctx.enc_left.inverted(),
             (int)ctx.R_ENC_A, (int)ctx.R_ENC_B, (int)ctx.enc_right.inverted());

    // 4) Queues COM (taille 1 pour xQueueOverwrite)
    if (!ctx.q_cmd_vw) ctx.q_cmd_vw = xQueueCreate(1, sizeof(AppContext::CmdVW));
    if (!ctx.q_tlm)    ctx.q_tlm    = xQueueCreate(1, sizeof(AppContext::Telemetry));
    if (!ctx.q_cmd_vw || !ctx.q_tlm) {
        ESP_LOGE(TAG, "Queue creation failed");
        return ESP_FAIL;
    }
    // 5) Init servo de direction (GPIO 23)
    ESP_RETURN_ON_ERROR(ctx.servo.init(), TAG, "servo.init");
    ESP_LOGI(TAG, "Servo direction OK: GPIO 23, 100 Hz, pw=[500,2500]us, angle_max=135deg");;

    // 6) Multiplexeur & watchdog (LOCAL par défaut)
    ctx.ctrl_mode        = AppContext::ControlMode::LOCAL;
    ctx.last_rx_cmd_tick = 0;  // pas encore de commande RX

    ESP_LOGI(TAG, "AppContext init OK (MCPWM @ %u Hz, mode=LOCAL)", ctx.L_cfg.freq_hz);
    return ESP_OK;
}