#include "Transport.h"
#include "Config.h"
#include "LEDHandler.h"
#include <Arduino.h>
#include <esp_system.h>  // esp_restart()

// Default is WiFi; setup() overwrites this from NVS via loadTransportMode() before any
// transport-specific task branches on it.
volatile TransportMode transportMode = TRANSPORT_WIFI;

TransportMode loadTransportMode() {
#ifdef BLE_ONLY
    return TRANSPORT_BLE;
#else
    preferences.begin("sys", true);  // read-only
    uint8_t v = preferences.getUChar("transport", TRANSPORT_WIFI);
    preferences.end();
    return (v == TRANSPORT_BLE) ? TRANSPORT_BLE : TRANSPORT_WIFI;
#endif
}

void saveTransportMode(TransportMode mode) {
    preferences.begin("sys", false);
    preferences.putUChar("transport", (uint8_t)mode);
    preferences.end();
}

void requestTransportSwap() {
    TransportMode next = (transportMode == TRANSPORT_BLE) ? TRANSPORT_WIFI : TRANSPORT_BLE;
    saveTransportMode(next);
    Serial.printf("[TRANSPORT] swapping to %s, rebooting...\n",
                  next == TRANSPORT_BLE ? "BLE" : "WiFi");
    // Visual feedback before the reboot: blue = BLE, green = WiFi.
    if (next == TRANSPORT_BLE) setLedOverride(0, 0, 255, 1500);
    else setLedOverride(0, 255, 0, 1500);
    delay(700);
    esp_restart();
}
