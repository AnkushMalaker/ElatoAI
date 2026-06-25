/**
 * BLEManager.cpp - OMI-compatible BLE peripheral for the Elato device.
 *
 * Mirrors the OMI GATT contract so Chronicle's existing OMI BLE clients connect unchanged:
 *   - Audio  19B10001 (NOTIFY): [3-byte header][Opus packet], 16 kHz mono, one packet/notify.
 *            The client strips the 3 bytes and Opus-decodes (friend-lite-sdk/decoder.py:
 *            Decoder(16000, 1), decode(data, 960)).
 *   - Codec  19B10002 (READ):   single byte 20 (Opus).
 *   - Button 23BA7925 (NOTIFY): 8 bytes = two LE uint32; first = state (struct "<II").
 *
 * Built on NimBLE-Arduino 2.x (h2zero). Only initialised when transportMode == TRANSPORT_BLE,
 * so in the WiFi build the BT controller is never started and WiFi keeps the full radio.
 */
#include "BLEManager.h"
#include "Config.h"
#include <NimBLEDevice.h>

// ── OMI GATT UUIDs (must match friend-lite-sdk/friend_lite/uuids.py exactly) ──
static const char *OMI_SERVICE_UUID        = "19B10000-E8F2-537E-4F6C-D104768A1214";
static const char *OMI_AUDIO_CHAR_UUID     = "19B10001-E8F2-537E-4F6C-D104768A1214";
static const char *OMI_CODEC_CHAR_UUID     = "19B10002-E8F2-537E-4F6C-D104768A1214";
static const char *OMI_BUTTON_SERVICE_UUID = "23BA7924-0000-1000-7450-346EAC492E92";
static const char *OMI_BUTTON_CHAR_UUID    = "23BA7925-0000-1000-7450-346EAC492E92";

static const uint8_t CODEC_OPUS = 20;  // friend-lite mapping: 0=PCM16, 1=PCM8, 20=Opus

static NimBLEServer *gServer = nullptr;
static NimBLECharacteristic *gAudioChar = nullptr;
static NimBLECharacteristic *gButtonChar = nullptr;
static volatile bool gConnected = false;
static uint16_t gSeq = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *s, NimBLEConnInfo &info) override {
        gConnected = true;
        // Tighten the connection interval for audio throughput/latency. Units are 1.25 ms,
        // so 12 -> 15 ms; latency 0; supervision timeout 400 -> 4 s.
        s->updateConnParams(info.getConnHandle(), 12, 12, 0, 400);
        if (deviceState != SLEEP) deviceState = LISTENING;
        Serial.println("[BLE] central connected");
    }
    void onDisconnect(NimBLEServer *s, NimBLEConnInfo &info, int reason) override {
        gConnected = false;
        if (deviceState != SLEEP) deviceState = IDLE;
        Serial.printf("[BLE] central disconnected (reason %d); re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

void bleSetup(const char *deviceName) {
    NimBLEDevice::init(deviceName);
    // Ask for a larger ATT MTU so a whole Opus packet rides in one notification. iOS still
    // negotiates ~185 (payload ~182) which is plenty for our ~120-byte (16 kbps) packets.
    NimBLEDevice::setMTU(247);
    NimBLEDevice::setPower(9);  // dBm (NimBLE-Arduino 2.x takes dBm, not an ESP_PWR_LVL enum)

    gServer = NimBLEDevice::createServer();
    gServer->setCallbacks(new ServerCallbacks());

    // Audio service: audio NOTIFY + codec READ.
    NimBLEService *audioSvc = gServer->createService(OMI_SERVICE_UUID);
    gAudioChar = audioSvc->createCharacteristic(OMI_AUDIO_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic *codecChar =
        audioSvc->createCharacteristic(OMI_CODEC_CHAR_UUID, NIMBLE_PROPERTY::READ);
    codecChar->setValue(&CODEC_OPUS, 1);
    audioSvc->start();

    // Button service.
    NimBLEService *btnSvc = gServer->createService(OMI_BUTTON_SERVICE_UUID);
    gButtonChar = btnSvc->createCharacteristic(OMI_BUTTON_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    btnSvc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(OMI_SERVICE_UUID);
    adv->setName(deviceName);
    adv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();
    Serial.printf("[BLE] advertising as \"%s\" (OMI-compatible, Opus 16 kHz)\n", deviceName);
}

bool bleIsConnected() { return gConnected; }

void bleNotifyAudioFrame(const uint8_t *opus, size_t len) {
    if (!gConnected || !gAudioChar || len == 0) return;
    static uint8_t frame[3 + 256];
    if (len > sizeof(frame) - 3) len = sizeof(frame) - 3;  // safety clamp
    frame[0] = (uint8_t)(gSeq & 0xFF);   // 3-byte header: LE uint16 packet seq + index byte.
    frame[1] = (uint8_t)(gSeq >> 8);     // Clients strip these 3 bytes before decoding.
    frame[2] = 0;                        // index within packet (we never fragment).
    gSeq++;
    memcpy(frame + 3, opus, len);
    gAudioChar->setValue(frame, len + 3);
    gAudioChar->notify();
}

void bleNotifyButton(uint32_t state) {
    if (!gConnected || !gButtonChar) return;
    // 8 bytes: first LE uint32 = state, second uint32 = 0 (matches SDK struct.unpack("<II")).
    uint8_t payload[8] = {0};
    payload[0] = (uint8_t)(state & 0xFF);
    payload[1] = (uint8_t)((state >> 8) & 0xFF);
    payload[2] = (uint8_t)((state >> 16) & 0xFF);
    payload[3] = (uint8_t)((state >> 24) & 0xFF);
    gButtonChar->setValue(payload, 8);
    gButtonChar->notify();
}
