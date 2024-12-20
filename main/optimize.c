#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/temperature_sensor.h"
#include "esp_sleep.h"

void optimizeESP32ForAudio() {
    esp_err_t ret;

    // 1. Configure WiFi for minimal power (but keep ESP-NOW functionality)
    wifi_mode_t mode = WIFI_MODE_STA;  // Need STA mode for ESP-NOW
    ret = esp_wifi_set_mode(mode);
    if (ret != ESP_OK) {
        ESP_LOGE("OPTIMIZE", "Failed to set WiFi mode");
    }
    
    // Minimize WiFi power
    ret = esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power saving
    if (ret != ESP_OK) {
        ESP_LOGE("OPTIMIZE", "Failed to disable WiFi power save");
    }

    // Set WiFi inactive time (for station interface)
    ret = esp_wifi_set_inactive_time(WIFI_IF_STA, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("OPTIMIZE", "Failed to set WiFi inactive time");
    }
    
    // 2. Disable logging completely
    esp_log_level_set("*", ESP_LOG_NONE);

    // 3. Disable UART peripherals except for debug port
    uart_driver_delete(UART_NUM_1);
    uart_driver_delete(UART_NUM_2);

    // 4. Disable task watchdog
    esp_task_wdt_deinit();

    // 5. Lock CPU frequency to maximum
    esp_pm_lock_handle_t pm_lock;
    ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_lock", &pm_lock);
    if (ret == ESP_OK) {
        ret = esp_pm_lock_acquire(pm_lock);
        if (ret != ESP_OK) {
            ESP_LOGE("OPTIMIZE", "Failed to acquire PM lock");
        }
    }

    // 6. Disable automatic light sleep
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // 7. Use PSRAM for audio buffers if available
    uint8_t *audioBuffer = (uint8_t *)heap_caps_malloc(1024, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!audioBuffer) {
        ESP_LOGW("OPTIMIZE", "PSRAM allocation failed, using regular memory");
    } else {
        heap_caps_free(audioBuffer);
    }
}