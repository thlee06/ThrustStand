#include "hx711.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <limits.h>
#include <string.h>

#define CLK_HALF_US      1        // 1 µs per half-cycle
#define DRDY_TIMEOUT_MS  500      // generous timeout covers 10SPS and 80SPS

static void IRAM_ATTR drdy_isr(void *arg)
{
    hx711_bus_t *bus = (hx711_bus_t *)arg;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(bus->drdy_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

void hx711_bus_init(hx711_bus_t *bus)
{
    bus->drdy_sem = xSemaphoreCreateBinary();

    // CLK output, idle low
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << bus->clk_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(bus->clk_pin, 0);

    // DOUT inputs — HX711 drives actively, no internal pull-up needed.
    // data_pins[0] gets a falling-edge ISR; others are polled briefly after.
    for (int i = 0; i < bus->channel_count; i++) {
        cfg.pin_bit_mask = (1ULL << bus->data_pins[i]);
        cfg.mode         = GPIO_MODE_INPUT;
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        cfg.intr_type    = (i == 0) ? GPIO_INTR_NEGEDGE : GPIO_INTR_DISABLE;
        gpio_config(&cfg);
    }

    // Attach ISR to data_pins[0] — fires the instant DRDY goes low.
    gpio_isr_handler_add(bus->data_pins[0], drdy_isr, (void *)bus);
}

int hx711_read_all(const hx711_bus_t *bus, int32_t *readings)
{
    // Drain any stale semaphore from a previous cycle.
    xSemaphoreTake(bus->drdy_sem, 0);

    // If DOUT is already low the conversion is done — no falling edge will come.
    // Otherwise wait for the ISR to fire.
    if (gpio_get_level(bus->data_pins[0]) != 0) {
        if (xSemaphoreTake(bus->drdy_sem, pdMS_TO_TICKS(DRDY_TIMEOUT_MS)) != pdTRUE) {
            for (int i = 0; i < bus->channel_count; i++) readings[i] = INT32_MIN;
            return 0;
        }
    }

    // Remaining channels share CLK so they should all be ready within ~1µs.
    // Spin-wait briefly (no vTaskDelay — this is microseconds, not ticks).
    for (int i = 1; i < bus->channel_count; i++) {
        int64_t t = esp_timer_get_time() + 2000; // 2ms max
        while (gpio_get_level(bus->data_pins[i]) != 0) {
            if (esp_timer_get_time() > t) break;
        }
    }

    // Clock 24 data bits, sampling all DOUT pins on each rising edge.
    int32_t raw[HX711_MAX_CH] = {0};
    for (int bit = 0; bit < 24; bit++) {
        gpio_set_level(bus->clk_pin, 1);
        esp_rom_delay_us(CLK_HALF_US);
        for (int i = 0; i < bus->channel_count; i++) {
            raw[i] = (raw[i] << 1) | gpio_get_level(bus->data_pins[i]);
        }
        gpio_set_level(bus->clk_pin, 0);
        esp_rom_delay_us(CLK_HALF_US);
    }

    // 25th pulse: selects Channel A, gain 128 for the next conversion.
    gpio_set_level(bus->clk_pin, 1);
    esp_rom_delay_us(CLK_HALF_US);
    gpio_set_level(bus->clk_pin, 0);
    esp_rom_delay_us(CLK_HALF_US);

    // Sign-extend and store.
    for (int i = 0; i < bus->channel_count; i++) {
        if (raw[i] & 0x800000) raw[i] |= (int32_t)0xFF000000;
        readings[i] = raw[i];
    }
    return bus->channel_count;
}
