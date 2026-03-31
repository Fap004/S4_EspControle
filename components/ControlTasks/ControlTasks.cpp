#include "ControlTasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────
// Utilitaire rampe RPM
// ─────────────────────────────────────────────────────────────
static inline float ramp(float cur, float tgt, float dmax_per_s, float dt)
{
    const float d = tgt - cur;
    const float step = dmax_per_s * dt;
    if (d >  step) return cur + step;
    if (d < -step) return cur - step;
    return tgt;
}

// ─────────────────────────────────────────────────────────────
// TASK 1 : CONTROL / WHEELS (100 Hz)
// ─────────────────────────────────────────────────────────────
static void vTaskWheelControlLoop(void* arg)
{
    AppContext* ctx = static_cast<AppContext*>(arg);

    const TickType_t period = pdMS_TO_TICKS(10);   // ✅ 100 Hz
    TickType_t wake = xTaskGetTickCount();
    TickType_t last = wake;

    ctx->pid_left.reset();
    ctx->pid_right.reset();

    float refL = 0.0f;
    float refR = 0.0f;
    const float target_rpm = 0.0f;
    const float dRPMmax    = 4000.0f;

    AppContext::CmdVW last_cmd { .v_mps = 0.f, .omega = 0.f };
    AppContext::ControlMode prev_mode = ctx->ctrl_mode;
    float prev_v = 0.0f;

    for (;;)
    {
        const TickType_t now = xTaskGetTickCount();
        const float dt = (now - last) * (1.0f / configTICK_RATE_HZ);
        last = now;

        // ── Réception RX (non bloquante) ───────────────────
        AppContext::CmdVW cmd;
        while (xQueueReceive(ctx->q_cmd_vw, &cmd, 0) == pdTRUE)
        {
            last_cmd              = cmd;
            ctx->last_rx_cmd_tick = now;
            ctx->ctrl_mode        = AppContext::ControlMode::REMOTE;
        }

        const bool rx_alive =
            (now - ctx->last_rx_cmd_tick) < ctx->RX_CMD_TIMEOUT;

        if (!rx_alive && ctx->ctrl_mode == AppContext::ControlMode::REMOTE)
            ctx->ctrl_mode = AppContext::ControlMode::LOCAL;

        // ── Transition de mode ─────────────────────────────
        if (ctx->ctrl_mode != prev_mode)
        {
            ctx->pid_left.reset();
            ctx->pid_right.reset();
            refL = refR = 0.0f;
            prev_v = 0.0f;
            prev_mode = ctx->ctrl_mode;
        }

        // ── CONTRÔLE ──────────────────────────────────────
        if (ctx->ctrl_mode == AppContext::ControlMode::REMOTE)
        {
            const bool stopping =
                (last_cmd.v_mps == 0.0f && last_cmd.omega == 0.0f);
            const bool was_moving = (prev_v != 0.0f);

            if (stopping && was_moving)
            {
                ctx->pid_left.reset();
                ctx->pid_right.reset();
            }

            prev_v = last_cmd.v_mps;

            // ── MOTEURS
            ctx->drive.setVW(last_cmd.v_mps, last_cmd.omega);
            ctx->drive.update(dt);

            // ── SERVO : Calcul UNIQUEMENT (PAS de PWM ici)
            float steer_rad = 0.0f;
            if (std::fabs(last_cmd.v_mps) > 0.01f)
            {
                const float wb = ctx->drive.geometry().wheel_base_m;
                steer_rad = std::atan(last_cmd.omega * wb / last_cmd.v_mps);
            }

            float steer_deg = steer_rad * 180.0f / static_cast<float>(M_PI);
            ctx->steering.setTargetAngle(steer_deg);
        }
        else
        {
            // ── MODE LOCAL : arrêt sécurisé
            refL = ramp(refL, target_rpm, dRPMmax, dt);
            refR = ramp(refR, target_rpm, dRPMmax, dt);

            ctx->wheel_left.setTargetRpm(refL);
            ctx->wheel_right.setTargetRpm(refR);
            ctx->wheel_left.update(dt);
            ctx->wheel_right.update(dt);

            // Direction locale forcée
            ctx->steering.setTargetAngle(20.0f);
        }

        // ── Télémétrie ─────────────────────────────────────
        AppContext::Telemetry tlm {
            .rpmL = ctx->wheel_left.measuredRpm(),
            .rpmR = ctx->wheel_right.measuredRpm()
        };
        (void)xQueueOverwrite(ctx->q_tlm, &tlm);

        vTaskDelayUntil(&wake, period);
    }
}

// ─────────────────────────────────────────────────────────────
// TASK 2 : SERVO / STEERING (50 Hz FIXE)
// ─────────────────────────────────────────────────────────────
static void vTaskSteeringLoop(void* arg)
{
    AppContext* ctx = static_cast<AppContext*>(arg);

    const TickType_t period = pdMS_TO_TICKS(20);   // ✅ 50 Hz STRICT
    TickType_t wake = xTaskGetTickCount();

    for (;;)
    {
        ctx->steering.update();    // ✅ PWM stable, synchro période
        vTaskDelayUntil(&wake, period);
    }
}

// ─────────────────────────────────────────────────────────────
// STARTERS DES TASKS
// ─────────────────────────────────────────────────────────────
extern "C" void start_wheel_task(AppContext* ctx, UBaseType_t prio)
{
    xTaskCreate(
        vTaskWheelControlLoop,
        "wheel_control_task",
        4096,
        ctx,
        prio,
        nullptr
    );
}

extern "C" void start_steering_task(AppContext* ctx, UBaseType_t prio)
{
    xTaskCreate(
        vTaskSteeringLoop,
        "steering_servo_task",
        2048,
        ctx,
        prio,
        nullptr
    );
}