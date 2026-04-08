#include "ComTasks.h"
#include "protocol.h"
#include "com.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

struct RxTxArgs {
    AppContext* ctx;
    uint8_t     peer[6];
};

// ─────────────────────────────────────────────
// RX TASK — reçoit CMD (télécommande → robot)
// ─────────────────────────────────────────────
static void vTaskRx(void* arg)
{
    RxTxArgs* a = (RxTxArgs*)arg;
    AppContext* ctx = a->ctx;
    delete a;

    uint8_t  data[MSG_DATA_LEN];
    size_t   len;
    uint16_t seq;

    const float v_max = 30.0f; // ← vitesse max du robot en m/s (à ajuster)

    for (;;)
    {
        if (!com_read_msg_wait(data, &len, &seq, portMAX_DELAY))
            continue;

        // Extraire le uint16_t CMD
        uint16_t w = ((uint16_t)data[0] << 8) | data[1];

        uint8_t speed8, angle6, dir2;
        proto_unpack_cmd(w, &speed8, &angle6, &dir2);

        // Vitesse : 0–255 → 0–100% → m/s
        float pct = (speed8 / 255.0f) * 100.0f;
        if (dir2 == PROTO_DIR_REV) pct = -pct;
        float v = (pct / 100.0f) * v_max;

        // Angle : 0–63 → -30° à +30°
        float steer_deg = (angle6 / 63.0f) * 60.0f - 30.0f;

        AppContext::CmdVW cmd{ v, steer_deg };
        xQueueOverwrite(ctx->q_cmd_vw, &cmd);

        vTaskDelay(1); // safety yield
    }
}

// ─────────────────────────────────────────────
// TX TASK — envoie TLM (robot → télécommande)
// ─────────────────────────────────────────────
static void vTaskTx(void* arg)
{
    RxTxArgs* a = (RxTxArgs*)arg;
    AppContext* ctx = a->ctx;
    uint8_t peer[6];
    memcpy(peer, a->peer, 6);
    delete a;

    const TickType_t period = pdMS_TO_TICKS(100); // 10 Hz
    TickType_t last = xTaskGetTickCount();

    for (;;)
    {
        // Lire télémétrie
        AppContext::Telemetry tlm;
        if (xQueuePeek(ctx->q_tlm, &tlm, 0) != pdTRUE) {
            tlm.rpmL = ctx->wheel_left.measuredRpm();
            tlm.rpmR = ctx->wheel_right.measuredRpm();
        }

        // Calculer vitesse totale en km/h depuis RPM moyen
        float rpmAvg = (tlm.rpmL + tlm.rpmR) / 2.0f;
        float v_mps  = (rpmAvg / 60.0f) * (2.0f * M_PI * ctx->geom.wheel_radius_m);

        proto_tlm_t tlm_pkt;
        tlm_pkt.speed_x100 = (int16_t)((v_mps * 3.6f) * 100.0f); // km/h × 100
        tlm_pkt.unit       = PROTO_UNIT_KMH;
        tlm_pkt.reserved   = 0;

        uint8_t data[MSG_DATA_LEN] = {};
        memcpy(data, &tlm_pkt, sizeof(proto_tlm_t)); // 4 octets

        com_send(peer, data, sizeof(data));
        vTaskDelayUntil(&last, period);
    }
}

// ─────────────────────────────────────────────
extern "C" void start_rx_task(AppContext* ctx,
                               UBaseType_t prio,
                               const uint8_t peer[6],
                               uint8_t channel)
{
    com_init(channel);
    com_add_peer(peer);

    auto* a = new RxTxArgs{ ctx, {0} };
    memcpy(a->peer, peer, 6);
    xTaskCreate(vTaskRx, "rx_task", 4096, a, prio, nullptr);
}

extern "C" void start_tx_task(AppContext* ctx,
                               UBaseType_t prio,
                               const uint8_t peer[6])
{
    auto* a = new RxTxArgs{ ctx, {0} };
    memcpy(a->peer, peer, 6);
    xTaskCreate(vTaskTx, "tx_task", 4096, a, prio, nullptr);
}