#include "ComTasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "protocol.h"
#include "com.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// ──────────────────────────────────────────────────────────────────────────────
// Helper : RPM → entier 13 bits [0..8191]
static uint16_t rpm_to_u13(float rpm, float rpm_max)
{
    if (rpm < 0.0f)      rpm = 0.0f;
    if (rpm > rpm_max)   rpm = rpm_max;
    return (uint16_t)((rpm / rpm_max) * 8191.0f + 0.5f);
}

// Bloc d'arguments passé aux tâches
struct RxTxArgs {
    AppContext* ctx;
    uint8_t     peer[6];
};

// ──────────────────────────────────────────────────────────────────────────────
// RX : reçoit une commande (v, ω) depuis le poste de contrôle
//      et la pousse dans q_cmd_vw pour que ControlTask la consomme
static void vTaskRx(void* arg)
{
    RxTxArgs*   a   = static_cast<RxTxArgs*>(arg);
    AppContext*  ctx = a->ctx;
    uint8_t peer_mac[6];
    memcpy(peer_mac, a->peer, 6);
    delete a;

    uint8_t  data[MSG_DATA_LEN];
    size_t   len;
    uint16_t seq;

    for (;;)
    {
        if (!com_read_msg_wait(data, &len, &seq, portMAX_DELAY)) continue;
        if (len != MSG_DATA_LEN) continue;

        uint16_t w = ((uint16_t)data[0] << 8) | data[1];
        uint16_t speed13;
        uint8_t  dir2, type1;
        proto_unpack(w, &speed13, &dir2, &type1);

        if (type1 == PROTO_TYPE_CMD)
        {
            const float rpm_max = 8000.0f;
            float rpm = (speed13 / 8191.0f) * rpm_max;
            if (dir2 == PROTO_DIR_REV) rpm = -rpm;

            // Convertit RPM → vitesse linéaire (ω=0 : ligne droite)
            const float omega_w = rpm * (2.0f * (float)M_PI / 60.0f); // rad/s roue
            const float v       = omega_w * ctx->geom.wheel_radius_m;

            AppContext::CmdVW cmd { .v_mps = v, .omega = 0.0f };

            // q_cmd_vw est de longueur 1 → xQueueOverwrite toujours valide
            (void)xQueueOverwrite(ctx->q_cmd_vw, &cmd);
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// TX : envoie la télémétrie (RPM moyen) au poste de contrôle à ~100 Hz
static void vTaskTx(void* arg)
{
    RxTxArgs*   a   = static_cast<RxTxArgs*>(arg);
    AppContext*  ctx = a->ctx;
    uint8_t peer_mac[6];
    memcpy(peer_mac, a->peer, 6);
    delete a;

    const TickType_t period = pdMS_TO_TICKS(50);   // 100 Hz
    TickType_t last = xTaskGetTickCount();

    for (;;)
    {
        AppContext::Telemetry tlm;

        // xQueuePeek : lit SANS consommer — ControlTask reste maître de la queue
        if (xQueuePeek(ctx->q_tlm, &tlm, 0) != pdTRUE)
        {
            tlm.rpmL = ctx->wheel_left.measuredRpm();
            tlm.rpmR = ctx->wheel_right.measuredRpm();
        }

        const float    rpm_avg = 0.5f * (tlm.rpmL + tlm.rpmR);
        const uint16_t u13     = rpm_to_u13(fabsf(rpm_avg), 3000.0f);
//        const uint16_t w       = proto_pack(u13, PROTO_DIR_NONE, PROTO_TYPE_TLM);

//        uint8_t data[MSG_DATA_LEN] = {
//            (uint8_t)(w >> 8),
//            (uint8_t)(w & 0xFF)
//        };
        const uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        const int16_t  rpmL      = (int16_t)tlm.rpmL;
        const int16_t  rpmR      = (int16_t)tlm.rpmR;
        uint8_t data[MSG_DATA_LEN];
        data[0] = (timestamp >> 24) & 0xFF;
        data[1] = (timestamp >> 16) & 0xFF;
        data[2] = (timestamp >>  8) & 0xFF;
        data[3] =  timestamp        & 0xFF;
        data[4] = (rpmL >> 8) & 0xFF;
        data[5] =  rpmL       & 0xFF;
        data[6] = (rpmR >> 8) & 0xFF;
        data[7] =  rpmR       & 0xFF;

        const esp_err_t err = com_send(peer_mac, data, sizeof(data));
        if (err != ESP_OK)
            printf("[TX] com_send err=%d\n", (int)err);

        vTaskDelayUntil(&last, period);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
extern "C" void start_rx_task(AppContext* ctx, UBaseType_t prio,
                               const uint8_t peer_mac[6], uint8_t channel)
{
    com_init(channel);
    com_add_peer(peer_mac);

    uint8_t my_mac[6];
    com_get_mac(my_mac);
    printf("[NODE] My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           my_mac[0], my_mac[1], my_mac[2],
           my_mac[3], my_mac[4], my_mac[5]);

    auto* a = new RxTxArgs{ ctx, {0} };
    memcpy(a->peer, peer_mac, 6);
    xTaskCreate(vTaskRx, "rx_task", 4096, a, prio, nullptr);
}

extern "C" void start_tx_task(AppContext* ctx, UBaseType_t prio,
                               const uint8_t peer_mac[6])
{
    auto* a = new RxTxArgs{ ctx, {0} };
    memcpy(a->peer, peer_mac, 6);
    xTaskCreate(vTaskTx, "tx_task", 4096, a, prio, nullptr);
}