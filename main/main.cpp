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
    //start_rx_task(&ctx, 2, peer_mac, wifi_channel);
    //start_tx_task(&ctx, 2, peer_mac);

    // 3) Démarrer la tâche de contrôle (100 Hz)
    start_wheel_task(&ctx,3);
    start_steering_task(&ctx,3);
    
    ESP_LOGI(TAG, "app_main: init OK, tasks started");
    // app_main() retourne, FreeRTOS prend la main.
}
/*
#include "AppContext.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SERVO_TEST";

extern "C" void app_main(void)
{
    static AppContext ctx;

    //ESP_LOGI(TAG, "Init AppContext...");
    ESP_ERROR_CHECK(appctx_init(ctx));

    //ESP_LOGI(TAG, "=== SERVO TEST START ===");

    while (true)
    {
        //ESP_LOGI(TAG, "Servo -> CENTRE (0 deg)");
        ctx.steering.setTargetAngle(0.0f);
        ctx.steering.update();
        vTaskDelay(pdMS_TO_TICKS(2000));

        //ESP_LOGI(TAG, "Servo -> DROITE (+20 deg)");
        ctx.steering.setTargetAngle(20.0f);
        ctx.steering.update();
        vTaskDelay(pdMS_TO_TICKS(2000));

        //ESP_LOGI(TAG, "Servo -> GAUCHE (-20 deg)");
        ctx.steering.setTargetAngle(-20.0f);
        ctx.steering.update();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}*/