#include "ControlTasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "ControlTask";

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

    for (;;)
    {
        // 1) Drainer la queue pour ne garder que la dernière consigne
        AppContext::CmdVW cmd;
        while (xQueueReceive(ctx->q_cmd_vw, &cmd, 0) == pdTRUE) {
            last_cmd = cmd;
        }

        // 2) Appliquer la consigne haut niveau (centralisation dans DriveBase)
        ctx->drive.setVW(last_cmd.v_mps, last_cmd.omega);

        // 3) Mettre à jour les deux roues via DriveBase
        ctx->drive.update(dt);

        // 4) Télémétrie (optionnel)
        AppContext::Telemetry tlm {
            .rpmL = ctx->wheel_left.measuredRpm(),
            .rpmR = ctx->wheel_right.measuredRpm()
        };
        // Choisis l'une des deux selon ta taille de queue :
        // (void)xQueueOverwrite(ctx->q_tlm, &tlm);    // si longueur == 1
        (void)xQueueSend(ctx->q_tlm, &tlm, 0);        // sinon, en non-bloquant

        vTaskDelayUntil(&last, period);
    }
}

extern "C" void start_control_task(AppContext* ctx, UBaseType_t prio)
{
    xTaskCreate(vTaskControlLoop, "control_task", 4096, ctx, prio, nullptr);
}
