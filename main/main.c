#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "climate_logger";

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-S3 climate_logger app started");
    vTaskSuspend(NULL);
}
