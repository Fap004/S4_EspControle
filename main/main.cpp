#include "AppContext.h"
#include "ControlTasks.h"
#include "ComTasks.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{
    static AppContext ctx;  // durée de vie globale (statique)

    // 1) Init des drivers/encodeurs/queues
    ESP_ERROR_CHECK(appctx_init(ctx));

    // 2) (Option) Démarrer les tâches COM (ESPNOW)
    // Remets ta MAC “peer” si tu l’utilises
    const uint8_t peer_mac[6]   = { 0x20, 0x6E, 0xF1, 0x09, 0xB3, 0xA0 };
    const uint8_t wifi_channel  = 1;
    start_rx_task(&ctx, 2, peer_mac, wifi_channel);
    start_tx_task(&ctx, 2, peer_mac);

    // 3) Démarrer la tâche de contrôle (100 Hz)
    start_control_task(&ctx,3);
    
    ESP_LOGI(TAG, "app_main: init OK, tasks started");
    // app_main() retourne, FreeRTOS prend la main.
}

/*#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static void vTaskOpenLoopLeft(void* arg)
{
    // Adapte ces #define à TES pins (celles d’AppContext)
    const gpio_num_t INA = GPIO_NUM_4;   // L_INA
    const gpio_num_t INB = GPIO_NUM_5;   // L_INB
    const gpio_num_t SEL = GPIO_NUM_6;   // L_SEL0
    const gpio_num_t PWM = GPIO_NUM_0;   // L_PWM

    gpio_config_t io = {};
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL<<INA) | (1ULL<<INB) | (1ULL<<SEL) | (1ULL<<PWM);
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // 1) Forward plein pot : INA=1, INB=0, PWM=1 (continu = 100% duty)
    gpio_set_level(SEL, 0);  // SEL0 peu importe pour tourner; 0 par défaut
    gpio_set_level(INA, 1);
    gpio_set_level(INB, 0);
    gpio_set_level(PWM, 1);
    ESP_LOGI("OPEN", "LEFT FORWARD FULL (2s)...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 2) Stop
    gpio_set_level(PWM, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 3) Reverse plein pot : INA=0, INB=1, PWM=1
    gpio_set_level(INA, 0);
    gpio_set_level(INB, 1);
    gpio_set_level(PWM, 1);
    ESP_LOGI("OPEN", "LEFT REVERSE FULL (2s)...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Stop et fin
    gpio_set_level(PWM, 0);
    ESP_LOGI("OPEN", "DONE");
    vTaskDelete(nullptr);
}

extern "C" void app_main(void)
{
    xTaskCreate(vTaskOpenLoopLeft, "open_L", 2048, nullptr, 8, nullptr);
}
*/