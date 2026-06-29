# ThrustStand DAQ

Real-time thrust measurement system for motor/propeller test stands. An ESP32 reads up to four HX711 load cell amplifiers and serves a live web dashboard over WiFi.

---

## Hardware

| Pin | Signal | Notes |
|-----|--------|-------|
| GPIO 4 | HX711 CLK | Shared across all chips |
| GPIO 21 | HX711 DOUT Ch0 | DRDY interrupt source |
| GPIO 5 | HX711 DOUT Ch1 | |
| GPIO 26 | HX711 DOUT Ch2 | 20 kg cell — displayed |
| GPIO 14 | HX711 DOUT Ch3 | 20 kg cell — displayed |

- Wire every HX711 `RATE` pin **HIGH** for 80 SPS mode.
- All chips share one CLK line so all channels are clocked simultaneously (synchronised readings).
- Currently Ch2 and Ch3 are the active displayed channels (both 20 kg rated).

---

## Firmware

Built with **PlatformIO + ESP-IDF** (`framework = espidf`).

### Configuration

Edit `src/transmitter/main.c` before flashing:

```c
#define WIFI_SSID "your-network"
#define WIFI_PASS "your-password"
```

### Build & Flash

```
pio run --target upload
pio device monitor
```

The serial monitor will print the ESP32's IP address once connected:

```
I (1234) wifi_server: IP address : 192.168.1.42
I (1234) wifi_server: Open: http://192.168.1.42
```

### Signal chain

```
HX711 DOUT (DRDY falling edge ISR)
  → hx711_read_all()          — bit-bang 25 CLK pulses, all channels simultaneous
  → 3-sample median filter    — rejects single-shot EMI/WiFi TX spikes
  → ring buffer (1000 samples ≈ 12.5 s at 80 SPS)
  → SSE push to browser       — sub-frame latency, no polling overhead
```

Failed reads (HX711 timeout) are dropped before the median and ring buffer — they never reach the display or CSV.

---

## Web Dashboard

Navigate to the ESP32's IP address in any browser on the same network.

### Calibration screen

1. Remove all load. Press **Tare 3s** for each channel — the firmware averages 3 seconds of samples to find the zero offset.
2. Apply a known mass. Enter its weight (kg) and moment arm (m, or 0 for direct force in N). Press **Cal 3s**.
3. Calibration values (offset, scale, unit) are saved in browser `localStorage` and survive page reloads. Tare is always required on power-on.
4. Press **Start Monitoring** once both channels are calibrated, or **Skip — Raw Counts** to go straight to the live view uncalibrated.

### Live view

**Display values** — Ch2, Ch3, and (in Parallel mode) their sum are shown in large numerals at the top.

**Mode toggle** — switches between two graph modes:

| Mode | Graph | Use when |
|------|-------|----------|
| Independent | Ch2 (black) + Ch3 (orange) | Cells measure different axes / redundant check |
| Parallel | Ch2 + Ch3 + Sum (blue, thick) | Both cells share the same load path; thrust = Ch2 + Ch3 |

In Parallel mode the y-axis scales to the combined maximum range (e.g. 392 N for two 20 kg cells) and the Sum trace is the primary measurement.

**Data collection buttons:**

| Button | Action |
|--------|--------|
| Recalibrate | Returns to calibration screen, discards any collected data |
| Start Collection | Clears the ring buffer and begins recording to ESP32 RAM |
| Stop Collection | Freezes the buffer; data is ready to download |
| Download CSV | Prompts for a filename, then downloads with calibration formula in header |

The sample counter in the top-right corner shows `N / 1000 samples` while collecting and `N samples (stopped)` after stopping. The ring buffer holds 1000 samples (~12.5 s at 80 SPS); oldest samples are overwritten once full.

### CSV format

```
# ThrustStand DAQ Export
# Formula: force = (raw_count - offset) * scale
# Ch2 (col ch2): offset=123456, scale=4.7190e-4, unit=N
# Ch3 (col ch3): offset=234567, scale=4.7085e-4, unit=N
seq,timestamp_us,flags,ch0,ch1,ch2,ch3
0,1000012,0,0,0,1482310,1491205
...
```

The formula header uses the calibration values that were active at download time. `ch0` and `ch1` are always recorded but not currently calibrated or displayed.

---

## Project structure

```
ThrustStand/
├── include/
│   └── ts_protocol.h       # ESP-NOW packet definition (shared header)
├── src/
│   ├── CMakeLists.txt
│   └── transmitter/
│       ├── main.c          # app_main: NVS init, GPIO ISR service, wifi_server_init
│       ├── hx711.h / .c    # Multi-channel HX711 driver (shared CLK, DRDY ISR)
│       ├── wifi_server.h
│       └── wifi_server.c   # HTTP server, acquisition task, embedded web UI
├── Assets/
│   └── avol_favicon.jpg
├── platformio.ini
└── sdkconfig.esp32_wroom
```

---

## HTTP endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | Web dashboard (single-page app) |
| GET | `/latest` | Latest packet as JSON (used by calibration screen) |
| GET | `/events` | Server-Sent Events stream — pushes every new reading |
| GET | `/data` | Ring buffer as CSV download |
| GET | `/status` | `{"count": N, "cap": 1000, "collecting": bool}` |
| POST | `/control` | Body `start` or `stop` — controls data collection |
