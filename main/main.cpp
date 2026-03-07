#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "com.h"
#include "protocol.h"     // PROTO_* + proto_pack/proto_unpack
#include "PcntEncoder.h"  // encodeur PCNT (pulse_cnt)

// MAC du peer (à adapter)
static const uint8_t peer_mac[6] = { 0x20, 0x6E, 0xF1, 0x0D, 0x4D, 0xB8 };

// Consignes (MAJ en RX)
static volatile uint16_t g_cmd_speed = 0;            // 0..8191
static volatile uint8_t  g_cmd_dir   = PROTO_DIR_NONE;

// Encodeur 1 : A=22, B=20
static PcntEncoder s_enc1(GPIO_NUM_22, GPIO_NUM_20, /*ticks/rev*/ 2048.0f /* à ajuster */);
// Encodeur 2 : A=21, B=19
static PcntEncoder s_enc2(GPIO_NUM_21, GPIO_NUM_19, /*ticks/rev*/ 2048.0f /* à ajuster */);

// ---------- RX : reçoit commandes ----------
static void rx_task(void *arg)
{
    uint8_t  data[MSG_DATA_LEN];
    size_t   len;
    uint16_t seq;

    TickType_t last_print = xTaskGetTickCount();
    const TickType_t print_period = pdMS_TO_TICKS(250);

    uint32_t rx_count = 0, lost_count = 0, dup_count = 0;
    uint16_t last_seq = 0;
    bool     have_last = false;

    for (;;)
    {
        if (com_read_msg_wait(data, &len, &seq, portMAX_DELAY))
        {
            if (len != MSG_DATA_LEN) continue;

            uint16_t w = ((uint16_t)data[0] << 8) | data[1];
            uint16_t speed13; uint8_t dir2; uint8_t type1;
            proto_unpack(w, &speed13, &dir2, &type1);

            // pertes/duplicatas
            if (have_last)
            {
                uint16_t expected = (uint16_t)(last_seq + 1);
                if (seq == last_seq) {
                    dup_count++;
                } else if (seq != expected) {
                    uint16_t delta = (uint16_t)(seq - expected);
                    lost_count += (uint32_t)delta;
                }
            }
            last_seq = seq; have_last = true;
            rx_count++;

            if (type1 == PROTO_TYPE_CMD)
            {
                g_cmd_speed = speed13;
                g_cmd_dir   = dir2;
                // TODO: brancher g_cmd_* dans ton PID ici si nécessaire
            }

            TickType_t now = xTaskGetTickCount();
            if (now - last_print >= print_period)
            {
                printf("[RX] seq=%u | total=%u | perdus=%u | dups=%u | cmd: speed=%u dir=%u\n",
                       seq, (unsigned)rx_count, (unsigned)lost_count, (unsigned)dup_count,
                       (unsigned)g_cmd_speed, (unsigned)g_cmd_dir);
                last_print = now;
            }
        }
    }
}

// ---------- Mesure vitesse : 100 Hz ----------
static volatile float g_rpm_1 = 0.0f;
static volatile float g_rpm_2 = 0.0f;

static void speed_task(void *arg)
{
    const float dt_s = 0.010f; // 10 ms → 100 Hz
    const float alpha = 0.3f;  // lissage simple
    TickType_t last = xTaskGetTickCount();

    for (;;)
    {
        int32_t d1 = s_enc1.getAndClearDeltaTicks();
        int32_t d2 = s_enc2.getAndClearDeltaTicks();

        float rpm1 = (d1 / s_enc1.ticksPerRev()) / dt_s * 60.0f;
        float rpm2 = (d2 / s_enc2.ticksPerRev()) / dt_s * 60.0f;

        // Filtre 1er ordre
        g_rpm_1 = alpha * rpm1 + (1.0f - alpha) * g_rpm_1;
        g_rpm_2 = alpha * rpm2 + (1.0f - alpha) * g_rpm_2;

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

// Helper : map RPM → 13 bits (0..8191)
static inline uint16_t rpm_to_u13(float rpm, float rpm_max)
{
    if (rpm < 0) rpm = 0;
    if (rpm > rpm_max) rpm = rpm_max;
    return (uint16_t)((rpm / rpm_max) * 8191.0f + 0.5f);
}

// ---------- TX : envoie télémétrie (100 Hz) ----------
static void tx_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(10); // ~100 Hz
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        // Ex : moyenne des deux roues
        float rpm_avg = 0.5f * (g_rpm_1 + g_rpm_2);

        // Encoder la TLM : TYPE=1, DIR optionnel
        uint16_t u13 = rpm_to_u13(fabsf(rpm_avg), /*rpm_max*/ 3000.0f); // adapte ta plage
        uint16_t w   = proto_pack(u13, PROTO_DIR_NONE, PROTO_TYPE_TLM);

        uint8_t data[MSG_DATA_LEN] = { (uint8_t)(w >> 8), (uint8_t)(w & 0xFF) };

        esp_err_t err = com_send(peer_mac, data, sizeof(data));
        if (err != ESP_OK) {
            printf("[TX] com_send err=%d\n", (int)err);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------- MAIN ----------
extern "C" void app_main(void)
{
    const uint8_t channel = 1;
    com_init(channel);

    uint8_t my_mac[6];
    com_get_mac(my_mac);
    printf("[NODE] My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           my_mac[0], my_mac[1], my_mac[2],
           my_mac[3], my_mac[4], my_mac[5]);

    com_add_peer(peer_mac);

    // Init PCNT encoders
    ESP_ERROR_CHECK(s_enc1.init());
    ESP_ERROR_CHECK(s_enc2.init());

    // Lancer les tâches
    xTaskCreate(rx_task,    "rx_task",    4096, NULL, 6, NULL);
    xTaskCreate(speed_task, "speed_task", 3072, NULL, 7, NULL); // calcule rpm
    xTaskCreate(tx_task,    "tx_task",    4096, NULL, 5, NULL); // envoie TLM
}