#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>

// OMI-compatible BLE peripheral. Streams mic audio as Opus (one packet per notification,
// with a 3-byte header) on the OMI audio characteristic and emits OMI button events, so
// Chronicle's existing OMI BLE clients -- the React Native phone app and the macOS
// local-wearable-client -- connect with NO changes. See friend-lite-sdk/uuids.py and
// decoder.py for the contract this mirrors.
void bleSetup(const char *deviceName);
bool bleIsConnected();

// Notify one Opus packet (16 kHz mono, <= 60 ms / 960 samples). The 3-byte header that the
// clients strip is prepended internally.
void bleNotifyAudioFrame(const uint8_t *opus, size_t len);

// Notify a button event (1 = SINGLE_PRESS, 2 = DOUBLE_PRESS, 3 = LONG_PRESS).
void bleNotifyButton(uint32_t state);

#endif
