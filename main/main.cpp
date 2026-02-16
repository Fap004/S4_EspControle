// main.cpp (projet unique)

// ====== SELECTEUR DE ROLE ======
// Commenter pour compiler en RECEPTEUR
#define ROLE_TX   1
// ===============================

#include "com.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>

// ⚠️ Si ROLE_TX est défini, mets ici la MAC DU RECEPTEUR (imprimée par le RX au boot)
static const uint8_t peer_mac[6] = { 0x20, 0x6E, 0xF1, 0x09, 0xB3, 0xA0 }; // <-- remplace si besoin

extern "C" void app_main(void)
{
    const uint8_t channel = 1;     // commun aux deux rôles
    com_init(channel);              // init commune

#ifdef ROLE_TX
    // ----------- EMETTEUR -----------
    uint8_t my_mac[6];
    com_get_mac(my_mac);
    printf("[TX] My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    printf("[TX] Target MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           peer_mac[0], peer_mac[1], peer_mac[2], peer_mac[3], peer_mac[4], peer_mac[5]);

    com_add_peer(peer_mac);

    uint16_t     vitesse = 0;
    const uint8_t id     = 3;
    const TickType_t period = pdMS_TO_TICKS(2); // ~200 Hz

    while (true) {
        uint16_t packed = ((vitesse & 0x1FFF) << 3) | (id & 0x07);
        uint8_t data[MSG_DATA_LEN] = {
            (uint8_t)((packed >> 8) & 0xFF),
            (uint8_t)(packed & 0xFF)
        };
        esp_err_t err = com_send(peer_mac, data, sizeof(data));
        if (err != ESP_OK) {
            printf("[TX] esp_now_send err=%d\n", (int)err);
        }
        vitesse = (uint16_t)((vitesse + 1) % 32001);
        vTaskDelay(period);
    }

#else
    // ----------- RECEPTEUR (bloquant) -----------
    uint8_t my_mac[6];
    com_get_mac(my_mac);
    printf("[RX] My MAC: %02X:%02X:%02X:%02X:%02X:%02X  (configure cette MAC dans l'émetteur)\n",
           my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);

    uint8_t  data[MSG_DATA_LEN];
    size_t   len;
    uint16_t seq;

    TickType_t last_print = xTaskGetTickCount();
    uint32_t   rx_count   = 0;

    const TickType_t wait_timeout = pdMS_TO_TICKS(20);   // attend jusqu'à 20 ms un message
    const TickType_t print_period = pdMS_TO_TICKS(250);  // logs rares

    while (true) {
        // Attente bloquante → laisse respirer l'IDLE/Wi‑Fi naturellement
        if (com_read_msg_wait(data, &len, &seq, wait_timeout)) {
            uint16_t packed  = ((uint16_t)data[0] << 8) | data[1];
            uint16_t vitesse = (packed >> 3) & 0x1FFF;
            uint8_t  id      = packed & 0x07;
            rx_count++;

            TickType_t now = xTaskGetTickCount();
            if (now - last_print >= print_period) {
                printf("[RX] Dernier: ID=%u | Vitesse=%u | seq=%u | total=%u\n",
                       id, vitesse, (unsigned)seq, (unsigned)rx_count);
                last_print = now;
            }
        }
        // pas besoin de vTaskDelay ici; xQueueReceive cède déjà le CPU en cas d'absence de message
    }
#endif
}
