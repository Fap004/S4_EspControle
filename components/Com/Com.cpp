#include "Com.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_crc.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <string.h>

static const char* TAG = "COM";

// Queue RX de messages ESP-NOW
static QueueHandle_t msg_queue = NULL;

// Sémaphore TX (un seul envoi à la fois)
static SemaphoreHandle_t tx_sem = NULL;

// Canal Wi-Fi
static uint8_t s_channel = 1;

// ─────────────────────────────────────────────────────────────
// CALLBACK TX
// ─────────────────────────────────────────────────────────────
static void espnow_send_cb(const esp_now_send_info_t* info,
                           esp_now_send_status_t status)
{
    (void)info;
    (void)status;
    if (tx_sem)
        xSemaphoreGive(tx_sem);
}

// ─────────────────────────────────────────────────────────────
// CALLBACK RX — ISR SAFE
// ─────────────────────────────────────────────────────────────
static void espnow_recv_cb(const esp_now_recv_info_t* info,
                           const uint8_t* data, int len)
{
    (void)info;
    if (!data || len != sizeof(msg_t) || !msg_queue)
        return;

    BaseType_t hpw = pdFALSE;
    msg_t m;
    memcpy(&m, data, sizeof(msg_t));

    xQueueSendFromISR(msg_queue, &m, &hpw);
    portYIELD_FROM_ISR(hpw);
}

// ─────────────────────────────────────────────────────────────
void com_get_mac(uint8_t mac[6])
{
    esp_wifi_get_mac(WIFI_IF_STA, mac);
}

// ─────────────────────────────────────────────────────────────
void com_init(uint8_t channel)
{
    s_channel = channel;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    msg_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(msg_t));
    configASSERT(msg_queue != NULL);

    tx_sem = xSemaphoreCreateBinary();
    configASSERT(tx_sem != NULL);
    xSemaphoreGive(tx_sem);

    ESP_LOGI(TAG, "ESPNOW ready on channel %u", s_channel);
}

// ─────────────────────────────────────────────────────────────
void com_add_peer(const uint8_t mac[6])
{
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = s_channel;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// ─────────────────────────────────────────────────────────────
static uint16_t s_send_seq = 0;

esp_err_t com_send(const uint8_t mac[6],
                   const uint8_t* data, size_t len)
{
    if (len > MSG_DATA_LEN)
        len = MSG_DATA_LEN;

    msg_t m = {};
    m.seq = s_send_seq++;
    memcpy(m.data, data, len);
    m.crc = esp_crc16_le(UINT16_MAX,
                         (uint8_t*)&m.seq,
                         sizeof(m.seq) + MSG_DATA_LEN);

    if (xSemaphoreTake(tx_sem, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "tx_sem timeout, forcing send");
        xSemaphoreGive(tx_sem);
    }

    esp_err_t err = esp_now_send(mac, (uint8_t*)&m, sizeof(msg_t));
    if (err != ESP_OK)
        xSemaphoreGive(tx_sem);

    return err;
}

// ─────────────────────────────────────────────────────────────
// BLOQUANT — utilisé par ComTasks
// ─────────────────────────────────────────────────────────────
static int read_msg_internal(uint8_t* out_data,
                             size_t* out_len,
                             uint16_t* out_seq,
                             TickType_t timeout)
{
    if (!msg_queue)
        return 0;

    msg_t m;
    if (xQueueReceive(msg_queue, &m, timeout) != pdTRUE)
        return 0;

    uint16_t crc = esp_crc16_le(UINT16_MAX,
                               (uint8_t*)&m.seq,
                               sizeof(m.seq) + MSG_DATA_LEN);
    if (crc != m.crc)
        return 0;

    if (out_seq)  *out_seq = m.seq;
    if (out_len)  *out_len = MSG_DATA_LEN;
    if (out_data) memcpy(out_data, m.data, MSG_DATA_LEN);
    return 1;
}

int com_read_msg(uint8_t* out_data,
                 size_t* out_len,
                 uint16_t* out_seq)
{
    return read_msg_internal(out_data, out_len, out_seq, 0);
}

int com_read_msg_wait(uint8_t* out_data,
                      size_t* out_len,
                      uint16_t* out_seq,
                      TickType_t ticks_to_wait)
{
    return read_msg_internal(out_data, out_len, out_seq,
                             ticks_to_wait);
}