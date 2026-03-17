#include "ControlTasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "ControlTask";

static inline float ramp(float cur, float tgt, float dmax_per_s, float dt) {
    const float d = tgt - cur, step = dmax_per_s * dt;
    if (d >  step) return cur + step;
    if (d < -step) return cur - step;
    return tgt;
}

static void vTaskControlLoop(void* arg)
{
    AppContext* ctx = static_cast<AppContext*>(arg);

    const TickType_t period = pdMS_TO_TICKS(50); // 20 Hz (télémétrie + contrôle de base)
    TickType_t wake = xTaskGetTickCount();      // scheduling
    TickType_t last = wake;                     // dt

    ctx->pid_left.reset();
    ctx->pid_right.reset();

    // Consigne locale (sécurité) : faible et rampe douce
    float refL = 0.0f, refR = 0.0f;
    const float target_rpm = 200.0f;
    const float dRPMmax    = 1200.0f;

    AppContext::CmdVW last_cmd { .v_mps = 0.f, .omega = 0.f };

    for(;;)
    {
        const TickType_t now = xTaskGetTickCount();
        const float dt = (now - last) * (1.0f / configTICK_RATE_HZ);
        last = now;

        // RX non bloquant : garde la dernière commande
        AppContext::CmdVW cmd;
        while (xQueueReceive(ctx->q_cmd_vw, &cmd, 0) == pdTRUE) {
            last_cmd = cmd;
            ctx->last_rx_cmd_tick = now;
            ctx->ctrl_mode        = AppContext::ControlMode::REMOTE;
        }
        const bool rx_alive = (now - ctx->last_rx_cmd_tick) < ctx->RX_CMD_TIMEOUT;

        if (ctx->ctrl_mode == AppContext::ControlMode::REMOTE && rx_alive) {
            // Chemin (v, ω)
            ctx->drive.setVW(last_cmd.v_mps, last_cmd.omega);
            ctx->drive.update(dt);                //ne pas appeler wheel.update() ici
        }
        else
        {
            // Chemin LOCAL : RPM direct
            refL = ramp(refL, target_rpm, dRPMmax, dt);
            refR = ramp(refR, target_rpm, dRPMmax, dt);

            ctx->wheel_left.setTargetRpm(refL);
            ctx->wheel_right.setTargetRpm(refR);

            ctx->wheel_left.update(dt);
            ctx->wheel_right.update(dt);
        }

        // Publie toujours la TLM (overwrite = toujours la plus récente)
        AppContext::Telemetry tlm {
            .rpmL = ctx->wheel_left.measuredRpm(),
            .rpmR = ctx->wheel_right.measuredRpm()
        };
        (void)xQueueOverwrite(ctx->q_tlm, &tlm);

        vTaskDelayUntil(&wake, period);
    }
}

extern "C" void start_control_task(AppContext* ctx, UBaseType_t prio)
{
    xTaskCreate(vTaskControlLoop, "control_task", 4096, ctx, prio, nullptr);
}