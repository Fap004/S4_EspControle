#include "ControlTasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────
// Dead zones
// ─────────────────────────────────────────────────────────────
#define STOP_THRESHOLD_MPS  0.05f   // ← en dessous = arrêt forcé
#define STOP_THRESHOLD_DEG  1.0f    // ← en dessous = direction neutre
#define MAX_STEER_DEG       28.0f    // marge mécanique
#define STEER_RAMP_DEG_S    120.0f    // vitesse max servo (°/s)

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

    const TickType_t period = pdMS_TO_TICKS(10);   // 100 Hz
    TickType_t wake = xTaskGetTickCount();
    TickType_t last = wake;

    ctx->pid_left.reset();
    ctx->pid_right.reset();

    float refL = 0.0f;
    float refR = 0.0f;
    const float target_rpm = 0.0f;
    const float dRPMmax    = 4000.0f;

    float v_ref = 0.0f;
    const float v_ramp = 0.6f;   // m/s²

    float steer_ref = 0.0f;      // ✅ RAMPE SERVO

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
            v_ref = 0.0f;
            steer_ref = 0.0f;
            prev_v = 0.0f;
            prev_mode = ctx->ctrl_mode;
        }

        // ── CONTRÔLE ───────────────────────
        if (ctx->ctrl_mode == AppContext::ControlMode::REMOTE)
        {
            // ✅ Dead‑zone VITESSE seule
            float v = (std::fabs(last_cmd.v_mps) < STOP_THRESHOLD_MPS)
                        ? 0.0f
                        : last_cmd.v_mps;

            // ✅ Dead‑zone ANGLE seule
            float steer = (std::fabs(last_cmd.steer_deg) < STOP_THRESHOLD_DEG)
                            ? 0.0f
                            : last_cmd.steer_deg;

            // ✅ Saturation mécanique
            steer = std::clamp(steer, -MAX_STEER_DEG, MAX_STEER_DEG);

            // ✅ Reset PID si arrêt réel
            if (v == 0.0f && prev_v != 0.0f)
            {
                ctx->pid_left.reset();
                ctx->pid_right.reset();
                v_ref = 0.0f;
            }

            prev_v = v;

            // ✅ Rampe vitesse
            v_ref = ramp(v_ref, v, v_ramp, dt);

            // ✅ RAMPE SERVO (POINT CRITIQUE)
            steer_ref = ramp(steer_ref, steer, STEER_RAMP_DEG_S, dt);

            float steer_rad =
                steer_ref * static_cast<float>(M_PI) / 180.0f;

            // ✅ Ackermann AVEC angle rampé
            ctx->drive.setVSteer(v_ref, steer_rad);
            ctx->drive.update(dt);

            // ✅ Servo AVEC angle rampé
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

            steer_ref = ramp(steer_ref, 0.0f, STEER_RAMP_DEG_S, dt);
            ctx->steering.setTargetAngle(steer_ref);
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

    const TickType_t period = pdMS_TO_TICKS(20);   // ✅ 50 Hz STRICT
    TickType_t wake = xTaskGetTickCount();

    for (;;)
    {
        ctx->steering.update();
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