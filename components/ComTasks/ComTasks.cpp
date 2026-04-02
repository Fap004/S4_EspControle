#include "ComTasks.h"
#include "protocol.h"
#include "com.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

static uint16_t rpm_to_u13(float rpm, float rpm_max)
{
    if (rpm < 0) rpm = 0;
    if (rpm > rpm_max) rpm = rpm_max;
    return (uint16_t)((rpm / rpm_max) * 8191.0f + 0.5f);
}

struct RxTxArgs {
    AppContext* ctx;
    uint8_t     peer[6];
};

// ─────────────────────────────────────────────
// RX TASK
// ─────────────────────────────────────────────
static void vTaskRx(void* arg)
{
    RxTxArgs* a = (RxTxArgs*)arg;
    AppContext* ctx = a->ctx;
    delete a;

    uint8_t data[MSG_DATA_LEN];
    size_t len;
    uint16_t seq;

    for (;;)
    {
        if (!com_read_msg_wait(data, &len, &seq, portMAX_DELAY))
            continue;

        uint16_t w = ((uint16_t)data[0] << 8) | data[1];
        uint16_t speed13;
        uint8_t dir2, type1;

        proto_unpack(w, &speed13, &dir2, &type1);
        if (type1 != PROTO_TYPE_CMD)
            continue;

        float rpm = (speed13 / 8191.0f) * 8000.0f;
        if (dir2 == PROTO_DIR_REV)
            rpm = -rpm;

        float omega = rpm * (2.0f * M_PI / 60.0f);
        float v     = omega * ctx->geom.wheel_radius_m;

        AppContext::CmdVW cmd{ v, 0.0f };
        xQueueOverwrite(ctx->q_cmd_vw, &cmd);

        vTaskDelay(1); // safety yield
    }
}

// ─────────────────────────────────────────────
// TX TASK
// ─────────────────────────────────────────────
static void vTaskTx(void* arg)
{
    RxTxArgs* a = (RxTxArgs*)arg;
    AppContext* ctx = a->ctx;
    uint8_t peer[6];
    memcpy(peer, a->peer, 6);
    delete a;

    const TickType_t period = pdMS_TO_TICKS(100);
    TickType_t last = xTaskGetTickCount();

    for (;;)
    {
        AppContext::Telemetry tlm;
        if (xQueuePeek(ctx->q_tlm, &tlm, 0) != pdTRUE) {
            tlm.rpmL = ctx->wheel_left.measuredRpm();
            tlm.rpmR = ctx->wheel_right.measuredRpm();
        }

        uint8_t data[MSG_DATA_LEN];
        uint32_t t = xTaskGetTickCount() * portTICK_PERIOD_MS;

        data[0] = (t >> 24);
        data[1] = (t >> 16);
        data[2] = (t >> 8);
        data[3] = t;
        data[4] = (int16_t)tlm.rpmL >> 8;
        data[5] = (int16_t)tlm.rpmL;
        data[6] = (int16_t)tlm.rpmR >> 8;
        data[7] = (int16_t)tlm.rpmR;

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

    auto* a = new RxTxArgs{ ctx,{0} };
    memcpy(a->peer, peer, 6);
    xTaskCreate(vTaskRx, "rx_task", 4096, a, prio, nullptr);
}

extern "C" void start_tx_task(AppContext* ctx,
                               UBaseType_t prio,
                               const uint8_t peer[6])
{
    auto* a = new RxTxArgs{ ctx,{0} };
    memcpy(a->peer, peer, 6);
    xTaskCreate(vTaskTx, "tx_task", 4096, a, prio, nullptr);
}