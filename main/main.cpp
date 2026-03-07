#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>

#include "com.h"
#include "protocol.h"   // doit définir PROTO_* et proto_pack/proto_unpack

// Remplace par la MAC de l'autre ESP (peer)
static const uint8_t peer_mac[6] = { 0x20, 0x6E, 0xF1, 0x0D, 0x4D, 0xB8 };

// Consignes mises à jour par RX (atomicité OK pour 16/8 bits sur ESP32)
static volatile uint16_t g_cmd_speed = 0;            // 0..8191
static volatile uint8_t  g_cmd_dir   = PROTO_DIR_NONE;

// Exemple de source de vitesse réelle (à remplacer par ton capteur)
static uint16_t read_speed_real_13bits()
{
    static uint16_t v = 0;
    v = (v + 37) & 0x1FFF;
    return v;
}

// ===================== TX TASK =====================
void tx_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(10); // ~100 Hz
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint16_t speed_real = read_speed_real_13bits();

        // TYPE=1 (télémétrie), DIR=optionnel (ici NONE)
        uint16_t w = proto_pack(speed_real, PROTO_DIR_NONE, PROTO_TYPE_TLM);
        uint8_t data[MSG_DATA_LEN] = { (uint8_t)(w >> 8), (uint8_t)(w & 0xFF) };

        esp_err_t err = com_send(peer_mac, data, sizeof(data));
        if (err != ESP_OK) {
            printf("[TX] com_send err=%d\n", (int)err);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

// ===================== RX TASK =====================
void rx_task(void *arg)
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

            // Statistiques pertes/duplicatas
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

// ===================== MAIN =====================
extern "C" void app_main(void)
{
    const uint8_t channel = 1;
    com_init(channel);

    uint8_t my_mac[6];
    com_get_mac(my_mac);
    printf("[NODE] My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);

    com_add_peer(peer_mac);

    // RX un peu prioritaire si critique
    xTaskCreate(rx_task, "rx_task", 4096, NULL, 6, NULL);
    xTaskCreate(tx_task, "tx_task", 4096, NULL, 5, NULL);
}