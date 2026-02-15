#include "com.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdio.h>

// Callback pour la réception ESP-NOW
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len)
{
    printf("Received %d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n",
           len,
           recv_info->src_addr[0], recv_info->src_addr[1],
           recv_info->src_addr[2], recv_info->src_addr[3],
           recv_info->src_addr[4], recv_info->src_addr[5]);

    // Copier le message dans un buffer pour l'afficher
    char msg[len + 1];
    memcpy(msg, data, len);
    msg[len] = '\0';
    printf("Message: %s\n", msg);
}

// Callback pour l'envoi ESP-NOW
static void espnow_send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    // info->des_addr contient l'adresse MAC du destinataire sur ESP32-C6
    printf("Sent to %02X:%02X:%02X:%02X:%02X:%02X, status: %d\n",
           info->des_addr[0], info->des_addr[1], info->des_addr[2],
           info->des_addr[3], info->des_addr[4], info->des_addr[5],
           status);
}

void com_init(void)
{
    // 1. Initialisation NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2. Initialisation TCP/IP et Wi-Fi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. Initialiser le mode Wi-Fi en station
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Fixer le canal (doit correspondre au peer si nécessaire)
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // 4. Initialisation ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // 5. Enregistrer les callbacks
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
}

// Ajouter un peer ESP-NOW
void com_add_peer(const uint8_t mac[6])
{
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 1;   // 0 = utiliser le canal actif du Wi-Fi
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// Envoyer un message ESP-NOW
void com_send(const uint8_t mac[6], const uint8_t *data, size_t len)
{
    ESP_ERROR_CHECK(esp_now_send(mac, data, len));
}
