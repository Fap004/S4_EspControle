#include "ControlTasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// ───────────────────────────────────────────────────────────────────────────────
// Configuration du mode test
// Active/disable build-time (décommente pour activer par défaut)
#define CONTROL_TEST_ENABLE 1

// Runtime switch : false = fonctionnement normal (RX)
// true  = séquence de test sans COM
static bool g_test_mode =
#ifdef CONTROL_TEST_ENABLE
    true;
#else
    false;
#endif
// ───────────────────────────────────────────────────────────────────────────────

static const char* TAG = "ControlTask";

// Limiteur de pente (rampe) : borne la variation par dt
static inline float ramp(float current, float target, float dmax_per_s, float dt)
{
    const float delta = target - current;
    const float step  = dmax_per_s * dt;
    if (delta >  step) return current + step;
    if (delta < -step) return current - step;
    return target;
}

static void apply_test_sequence(AppContext* ctx, float dt_s, TickType_t* plast)
{
    // Période de la boucle (10 ms) — cohérente avec la tâche
    const TickType_t period = pdMS_TO_TICKS(10);

    // Séquence : chaque phase dure ~2 s (200 * 10 ms)
    // 0: avance, 1: recule, 2: stop, 3: rotation gauche, 4: rotation droite, 5: stop
    static int phase   = 0;
    static int counter = 0;

    // Consigne cibles (v, w)
    float v_target = 0.0f;   // m/s
    float w_target = 0.0f;   // rad/s

    switch (phase)
    {
        case 0: v_target = +0.6f; w_target =  0.0f;  break; // avancer
        case 1: v_target = -0.6f; w_target =  0.0f;  break; // reculer
        case 2: v_target =  0.0f; w_target =  0.0f;  break; // stop
        case 3: v_target =  0.0f; w_target = +1.2f;  break; // rotation gauche
        case 4: v_target =  0.0f; w_target = -1.2f;  break; // rotation droite
        case 5: v_target =  0.0f; w_target =  0.0f;  break; // stop
        default: phase = 0; break;
    }

    // États lissés (persistent entre appels)
    static float v = 0.0f;
    static float w = 0.0f;

    // RAMPING (limites d'accélération)
    const float dv_max = 0.8f;  // m/s²  (↗↘ vitesse linéaire)
    const float dw_max = 2.5f;  // rad/s² (↗↘ vitesse angulaire)

    v = ramp(v, v_target, dv_max, dt_s);
    w = ramp(w, w_target, dw_max, dt_s);

    // Appliquer la consigne lissée via DriveBase (centralisation)
    ctx->drive.setVW(v, w);
    ctx->drive.update(dt_s);

    if (++counter >= 200) {   // ≈ 2 s par phase
        counter = 0;
        phase   = (phase + 1) % 6;
        ESP_LOGI(TAG, "[TEST] phase=%d (v=%.2f m/s, w=%.2f rad/s)", phase, v, w);
    }

    vTaskDelayUntil(plast, period);
}

static void vTaskControlLoop(void* arg)
{
    AppContext* ctx = static_cast<AppContext*>(arg);
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
    const float dt = 0.010f;

    ctx->pid_left.reset();
    ctx->pid_right.reset();

    TickType_t last = xTaskGetTickCount();

    // Dernière consigne connue (si rien en queue, on garde ça)
    AppContext::CmdVW last_cmd { .v_mps = 0.0f, .omega = 0.0f };

    // États lissés aussi en mode normal, pour éviter les à-coups venant du RX
    float v_smooth = 0.0f;
    float w_smooth = 0.0f;
    const float dv_max_norm = 0.8f;  // mêmes limites par défaut
    const float dw_max_norm = 2.5f;

    for (;;)
    {
        // ── MODE TEST LOCAL (aucune COM requise) ───────────────────────────────
        if (g_test_mode)
        {
            apply_test_sequence(ctx, dt, &last);
            continue; // on ignore la RX dans ce mode
        }

        // ── MODE NORMAL : lecture de la queue RX (garde la dernière) ──────────
        AppContext::CmdVW cmd;
        while (xQueueReceive(ctx->q_cmd_vw, &cmd, 0) == pdTRUE)
        {
            last_cmd = cmd;
        }

        // Lissage (rampe) aussi en mode normal pour supprimer les steps RX
        v_smooth = ramp(v_smooth, last_cmd.v_mps,   dv_max_norm, dt);
        w_smooth = ramp(w_smooth, last_cmd.omega,   dw_max_norm, dt);

        // Appliquer la consigne haut niveau (centralisation DriveBase)
        ctx->drive.setVW(v_smooth, w_smooth);

        // Mise à jour des deux roues via DriveBase
        ctx->drive.update(dt);

        // Télémétrie (optionnel)
        AppContext::Telemetry tlm {
            .rpmL = ctx->wheel_left.measuredRpm(),
            .rpmR = ctx->wheel_right.measuredRpm()
        };
        // Choisis selon la taille de ta queue tlm :
        // (void)xQueueOverwrite(ctx->q_tlm, &tlm);   // si longueur == 1
        (void)xQueueSend(ctx->q_tlm, &tlm, 0);        // sinon, en non-bloquant

        vTaskDelayUntil(&last, period);
    }
}

extern "C" void start_control_task(AppContext* ctx, UBaseType_t prio)
{
    xTaskCreate(vTaskControlLoop, "control_task", 4096, ctx, prio, nullptr);
}