#pragma once

// Initialize WiFi (STA mode), connect to the given AP, start the HTTP/WebSocket
// server, and launch the HX711 acquisition task.
// Call once from app_main after NVS init and GPIO ISR service are installed.
//
// ssid     – 2.4 GHz AP SSID
// password – WPA2 passphrase (empty string "" for open network)
void wifi_server_init(const char *ssid, const char *password);
