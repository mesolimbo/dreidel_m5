// main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char* TAG = "HELLO";

void app_main(void) {
    ESP_LOGI(TAG, "Boot count. Hello from ESP-IDF!");
    // Example: blink GPIO10 if your device exposes it; if not, just logs.
    const gpio_num_t PIN = GPIO_NUM_10;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);

    // ReSharper disable once CppDFAEndlessLoop
    while (true) {
        gpio_set_level(PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_set_level(PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
        ESP_LOGI(TAG, "Uptime tick");
    }
}
