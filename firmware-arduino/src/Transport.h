#ifndef TRANSPORT_H
#define TRANSPORT_H

// Chronicle device transport: WiFi+WebSocket (default) or BLE peripheral (OMI-compatible).
// Exactly ONE is active per boot. Switching is a reboot-swap: the desired mode is stored
// in NVS and the device restarts so the chosen stack comes up cleanly. We deliberately do
// NOT run WiFi and BLE at the same time -- this board has no PSRAM and the 2.4 GHz radio is
// shared, so coexistence costs RAM and jitters audio. Swap (one at a time) avoids all of it.
enum TransportMode { TRANSPORT_WIFI = 0, TRANSPORT_BLE = 1 };

extern volatile TransportMode transportMode;

// Read the persisted transport from NVS (defaults to WiFi, or BLE if built with -D BLE_ONLY).
TransportMode loadTransportMode();

// Persist the given mode (does not reboot).
void saveTransportMode(TransportMode mode);

// Toggle WiFi<->BLE in NVS and reboot into the other transport. Bound to the
// double-tap-hold gesture in touchTask (main.cpp).
void requestTransportSwap();

#endif
