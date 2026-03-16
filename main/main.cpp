#include "AppContext.h"
#include "ControlTasks.h"
#include "ComTasks.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MAIN";

// --- Petite tâche utilitaire pour logger l'encodeur gauche à 50 Hz ---
static void vTaskEncDebug(void* arg)
{
    AppContext* ctx = static_cast<AppContext*>(arg);
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        TickType_t now = xTaskGetTickCount();
        const float dt = (now - last) * (1.0f / configTICK_RATE_HZ);
        last = now;

        // Lecture delta + RPM filtrée
        const int32_t d  = ctx->enc_left.getDelta();
        const float   rpm = ctx->enc_left.rpm(dt, /*lpf=*/true);

        ESP_LOGI("ENC-L", "d=%ld rpm=%.2f", (long)d, rpm);

        vTaskDelayUntil(&last, period);
    }
}

extern "C" void app_main(void)
{
    static AppContext ctx;  // durée de vie globale (statique)

    // 1) Init des drivers/encodeurs/queues
    ESP_ERROR_CHECK(appctx_init(ctx));

    // 2) (Option) Démarrer les tâches COM (ESPNOW)
    // Remets ta MAC “peer” si tu l’utilises
    // const uint8_t peer_mac[6]   = { 0x20, 0x6E, 0xF1, 0x09, 0xB3, 0xA0 };
    // const uint8_t wifi_channel  = 1;
    // start_rx_task(&ctx, 6, peer_mac, wifi_channel);
    // start_tx_task(&ctx, 5, peer_mac);

    // 3) Démarrer la tâche de contrôle (100 Hz)
    start_control_task(&ctx, /*prio=*/7);

    // 4) (Option) petite tâche de debug encodeur (50 Hz)
    xTaskCreate(vTaskEncDebug, "enc_dbg", 3072, &ctx, /*prio=*/4, nullptr);

    ESP_LOGI(TAG, "app_main: init OK, tasks started");
    // app_main() retourne, FreeRTOS prend la main.
}
/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define INA   GPIO_NUM_1
#define INB   GPIO_NUM_10
#define SEL0  GPIO_NUM_11
#define PWM   GPIO_NUM_13

extern "C" void app_main(void)
{
    // GPIO
    gpio_set_direction(INA,  GPIO_MODE_OUTPUT);
    gpio_set_direction(INB,  GPIO_MODE_OUTPUT);
    gpio_set_direction(SEL0, GPIO_MODE_OUTPUT);
    gpio_set_level(SEL0, 1); // arbitraire pour test

    // PWM (LEDC)
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,  // 0..255
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 20000,             // ≤ 20 kHz (datasheet)
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer); // IMPORTANT

    ledc_channel_config_t channel = {
        .gpio_num       = PWM,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&channel);

    while (1) {
        // Sens 1
        gpio_set_level(INA, 1);
        gpio_set_level(INB, 0);

        // Coup de démarrage (100%)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(400));

        // Vitesse de croisière (~70%)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 180);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        printf("Forward\n");
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Stop
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Sens 2
        gpio_set_level(INA, 0);
        gpio_set_level(INB, 1);

        // Coup de démarrage
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(400));

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 180);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        printf("Reverse\n");
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Stop
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
*/