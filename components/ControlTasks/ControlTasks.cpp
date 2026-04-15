#include "ControlTasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────
// Dead zones
// ─────────────────────────────────────────────────────────────
#define STOP_THRESHOLD_MPS  0.5f
#define STOP_THRESHOLD_DEG  1.0f
#define MAX_STEER_DEG       30.0f
#define STEER_RAMP_DEG_S    120.0f
#define ZERO_CROSS_ZONE_MPS  0.3f   // zone autour de 0 (±0.3 m/s)


// ─────────────────────────────────────────────────────────────
// Utilitaire rampe
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

    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t wake = xTaskGetTickCount();
    TickType_t last = wake;

    ctx->pid_left.reset();
    ctx->pid_right.reset();

    float refL = 0.0f;
    float refR = 0.0f;
    const float target_rpm = 0.0f;
    const float dRPMmax    = 4000.0f;

    float v_ref    = 0.0f;
    const float v_ramp_accel = 1.0f;   // m/s² — garde ta valeur actuelle
    const float v_ramp_brake = 4.0f;   // m/s² — freinage plus agressif

    float steer_ref = 0.0f;

    AppContext::CmdVW last_cmd { 0.f, 0.f };
    AppContext::ControlMode prev_mode = ctx->ctrl_mode;
    float prev_v = 0.0f;

    for (;;)
    {
        const TickType_t now = xTaskGetTickCount();
        const float dt = (now - last) * (1.0f / configTICK_RATE_HZ);
        last = now;

        // ── Réception RX ───────────────────
        AppContext::CmdVW cmd;
        while (xQueueReceive(ctx->q_cmd_vw, &cmd, 0) == pdTRUE)
        {
            last_cmd = cmd;
            ctx->last_rx_cmd_tick = now;
            ctx->ctrl_mode = AppContext::ControlMode::REMOTE;
        }

        const bool rx_alive =
            (now - ctx->last_rx_cmd_tick) < ctx->RX_CMD_TIMEOUT;

        if (!rx_alive && ctx->ctrl_mode == AppContext::ControlMode::REMOTE)
            ctx->ctrl_mode = AppContext::ControlMode::LOCAL;

        // ── Transition de mode ─────────────
        if (ctx->ctrl_mode != prev_mode)
        {
            ctx->pid_left.reset();
            ctx->pid_right.reset();
            refL = refR = 0.0f;
            v_ref     = 0.0f;
            steer_ref = 0.0f;
            prev_v    = 0.0f;

            if (ctx->ctrl_mode == AppContext::ControlMode::REMOTE)
            {
                ctx->steering.enable();     // ← servo actif en REMOTE
            }
            else
            {
                ctx->steering.setTargetAngle(0.0f);  // ← recentrer avant de couper
                ctx->steering.disable();             // ← servo muet en LOCAL
            }

            prev_mode = ctx->ctrl_mode;
        }

        // ── CONTRÔLE ───────────────────────
        if (ctx->ctrl_mode == AppContext::ControlMode::REMOTE)
        {
            float v = (std::fabs(last_cmd.v_mps) < STOP_THRESHOLD_MPS)
                        ? 0.0f : last_cmd.v_mps;

            float steer = (std::fabs(last_cmd.steer_deg) < STOP_THRESHOLD_DEG)
                            ? 0.0f : last_cmd.steer_deg;

            steer = std::clamp(steer, -MAX_STEER_DEG, MAX_STEER_DEG);

            if (v == 0.0f && prev_v != 0.0f)
            {
                ctx->pid_left.reset();
                ctx->pid_right.reset();
                v_ref = 0.0f;
            }
            prev_v = v;

            // ── Rampe intelligente autour de 0 ──────────────────────
            float v_ramp;

            // Distance à 0 (zone critique)
            const float v_abs = std::fabs(v_ref);
            const bool near_zero = v_abs < ZERO_CROSS_ZONE_MPS;

            // Changement de signe ? (ex: + → − ou − → +)
            const bool sign_change =
                (v_ref > 0.0f && v < 0.0f) ||
                (v_ref < 0.0f && v > 0.0f);

            if (near_zero || sign_change)
            {
                v_ramp = v_ramp_brake;
            }
            else
            {
                v_ramp = v_ramp_accel;
            }

            v_ref = ramp(v_ref, v, v_ramp, dt);

            steer_ref = ramp(steer_ref, steer, STEER_RAMP_DEG_S, dt);

            float steer_rad = steer_ref * static_cast<float>(M_PI) / 180.0f;

            ctx->drive.setVSteer(v_ref, steer_rad);
            ctx->drive.update(dt);

            ctx->steering.setTargetAngle(steer_ref);
        }
        else
        {
            // ── MODE LOCAL ─────────────────
            refL = ramp(refL, target_rpm, dRPMmax, dt);
            refR = ramp(refR, target_rpm, dRPMmax, dt);

            ctx->wheel_left.setTargetRpm(refL);
            ctx->wheel_right.setTargetRpm(refR);
            ctx->wheel_left.update(dt);
            ctx->wheel_right.update(dt);

            // Le servo est disabled → vTaskSteeringLoop n'écrit rien
        }

        // ── Télémétrie ─────────────────────
        AppContext::Telemetry tlm {
            .rpmL = ctx->wheel_left.measuredRpm(),
            .rpmR = ctx->wheel_right.measuredRpm()
        };
        xQueueOverwrite(ctx->q_tlm, &tlm);

        vTaskDelayUntil(&wake, period);
    }
}

// ─────────────────────────────────────────────────────────────
// TASK 2 : SERVO / STEERING (50 Hz FIXE)
// ─────────────────────────────────────────────────────────────
static void vTaskSteeringLoop(void* arg)
{
    AppContext* ctx = static_cast<AppContext*>(arg);

    const TickType_t period = pdMS_TO_TICKS(20);
    TickType_t wake = xTaskGetTickCount();

    for (;;)
    {
        ctx->steering.update();   // ← no-op si disabled
        vTaskDelayUntil(&wake, period);
    }
}

// ─────────────────────────────────────────────────────────────
// STARTERS
// ─────────────────────────────────────────────────────────────
extern "C" void start_wheel_task(AppContext* ctx, UBaseType_t prio)
{
    xTaskCreate(vTaskWheelControlLoop, "wheel_control_task", 4096, ctx, prio, nullptr);
}

extern "C" void start_steering_task(AppContext* ctx, UBaseType_t prio)
{
    xTaskCreate(vTaskSteeringLoop, "steering_servo_task", 2048, ctx, prio, nullptr);
}