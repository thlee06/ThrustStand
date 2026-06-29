#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ts_protocol.h"

// HX711 multi-channel driver with shared CLK.
// All HX711 chips share one CLK pin; each has its own DOUT pin.
// All chips are clocked simultaneously so readings are synchronised.
// Wire every RATE pin HIGH for 80 SPS mode.

#define HX711_MAX_CH  TS_MAX_CHANNELS   // up to 4

typedef struct {
    gpio_num_t      clk_pin;
    gpio_num_t      data_pins[HX711_MAX_CH];
    int             channel_count;
    SemaphoreHandle_t drdy_sem;          // posted by ISR on data_pins[0] falling edge
} hx711_bus_t;

// Configure GPIO pins and install DRDY interrupt on data_pins[0].
// Call gpio_install_isr_service(0) once before this.
void hx711_bus_init(hx711_bus_t *bus);

// Block until DRDY interrupt fires (data_pins[0] falling edge), verify all
// other channels are also ready, then clock 25 pulses and fill readings[].
// Returns channel_count on success, 0 on 500ms timeout.
int hx711_read_all(const hx711_bus_t *bus, int32_t *readings);
