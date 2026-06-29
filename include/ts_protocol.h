#pragma once
#include <stdint.h>

// ─── ESP-NOW Packet: Transmitter (Wroom) → Receiver (Nano) ──────────────────
// Payload: 2+2+4+1+1+4*4 = 26 bytes (well within 250-byte ESP-NOW limit)

#define TS_MAX_CHANNELS     4
#define TS_ESPNOW_MAGIC     0xA55A

typedef struct __attribute__((packed)) {
    uint16_t magic;                      // 0xA55A — sanity/version check
    uint16_t seq;                        // rolling counter; detect drops
    uint32_t timestamp_us;               // esp_timer_get_time() lower 32 bits
    uint8_t  channel_count;              // number of valid readings
    uint8_t  flags;                      // bit0=overrange, bit1=not-ready
    int32_t  readings[TS_MAX_CHANNELS];  // raw 24-bit values, sign-extended
} ts_espnow_packet_t;

