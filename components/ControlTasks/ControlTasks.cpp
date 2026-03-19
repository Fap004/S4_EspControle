#include "ControlTasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static inline float ramp(float cur, float tgt, float dmax_per_s, float dt)
{
    const float d = tgt - cur, step = dmax_per_s * dt;
    if (d >  step) return cur + step;
    if (d < -step) return cur - step;
    return tgt;
}

static void vTaskControlLoop(void* arg)
{
    AppContext* ctx = static_cast<AppContext*>(arg);

    const TickType_t period = pdMS_TO_TICKS(10);   // 100 Hz
    TickType_t wake = xTaskGetTickCount();
    TickType_t last = wake;

    ctx->pid_left.reset();
    ctx->pid_right.reset();

    float refL = 0.0f, refR = 0.0f;
    const float target_rpm = -0.0f;//0 en remote
    const float dRPMmax    = 4000.0f;

    AppContext::CmdVW last_cmd { .v_mps = 0.f, .omega = 0.f };
    AppContext::ControlMode prev_mode = ctx->ctrl_mode;

    // Suivi de la consigne précédente pour détecter le passage à 0
    float prev_v = 0.0f;

    for (;;)
    {
        const TickType_t now = xTaskGetTickCount();
        const float dt = (now - last) * (1.0f / configTICK_RATE_HZ);
        last = now;

        // ── RX non bloquant ───────────────────────────────────────────────────
        AppContext::CmdVW cmd;
        while (xQueueReceive(ctx->q_cmd_vw, &cmd, 0) == pdTRUE)
        {
            last_cmd              = cmd;
            ctx->last_rx_cmd_tick = now;
            ctx->ctrl_mode        = AppContext::ControlMode::REMOTE;
        }

        const bool rx_alive = (now - ctx->last_rx_cmd_tick) < ctx->RX_CMD_TIMEOUT;

        if (!rx_alive && ctx->ctrl_mode == AppContext::ControlMode::REMOTE)
            ctx->ctrl_mode = AppContext::ControlMode::LOCAL;

        // ── Transition de mode → reset PID ───────────────────────────────────
        if (ctx->ctrl_mode != prev_mode)
        {
            ctx->pid_left.reset();
            ctx->pid_right.reset();
            refL = 0.0f;
            refR = 0.0f;
            prev_v    = 0.0f;
            prev_mode = ctx->ctrl_mode;
        }

        // ── Contrôle ──────────────────────────────────────────────────────────
        if (ctx->ctrl_mode == AppContext::ControlMode::REMOTE)
        {
            // Reset PID quand la consigne passe à 0 (stop)
            // Évite le coup en sens inverse causé par l'intégrale accumulée
            const bool stopping = (last_cmd.v_mps == 0.0f && last_cmd.omega == 0.0f);
            const bool was_moving = (prev_v != 0.0f);

            if (stopping && was_moving)
            {
                ctx->pid_left.reset();
                ctx->pid_right.reset();
            }
            prev_v = last_cmd.v_mps;

            ctx->drive.setVW(last_cmd.v_mps, last_cmd.omega);
            ctx->drive.update(dt);
        }
        else
        {
            refL = ramp(refL, target_rpm, dRPMmax, dt);
            refR = ramp(refR, target_rpm, dRPMmax, dt);

            ctx->wheel_left.setTargetRpm(refL);
            ctx->wheel_right.setTargetRpm(refR);

            ctx->wheel_left.update(dt);
            ctx->wheel_right.update(dt);
        }

        // ── Télémétrie ────────────────────────────────────────────────────────
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