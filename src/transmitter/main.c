#include "wifi_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

// ── WiFi credentials — update to match your network ───────────────────────────
#define WIFI_SSID "NewlabMember 2.4GHz Only"
#define WIFI_PASS "!Welcome2NewLab!"

void app_main(void)
{
    // NVS is required by the WiFi driver.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // GPIO ISR service must be installed before hx711_bus_init registers the DRDY ISR.
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    wifi_server_init(WIFI_SSID, WIFI_PASS);
    // wifi_server_init does not return until WiFi is connected and the
    // acquisition task is running.
}
