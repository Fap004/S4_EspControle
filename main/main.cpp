#include "init.h"
#include "com.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_err.h"

#define GPIO_EXT GPIO_NUM_8  // Sortie externe

extern "C" void app_main(void)
{
    // Initialise ESP-NOW
    com_init();
    printf("ESP B: Prêt à recevoir des messages...\n");

    // Initialise WS2812
    ws2812_init();

    // Configure GPIO externe
    gpio_reset_pin(GPIO_EXT);
    gpio_set_direction(GPIO_EXT, GPIO_MODE_OUTPUT);

    while (true) 
    {
        // Exemple : clignotement GPIO externe
        gpio_set_level(GPIO_EXT, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(GPIO_EXT, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Exemple : allume WS2812 en bleu
        ws2812_set_color(0, 0, 50);
        vTaskDelay(pdMS_TO_TICKS(500));
        ws2812_set_color(0, 0, 0); // éteint
    }
}