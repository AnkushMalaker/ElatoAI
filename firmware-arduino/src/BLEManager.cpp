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
#include "Audio.h"
#include <NimBLEDevice.h>

// ── OMI GATT UUIDs (must match friend-lite-sdk/friend_lite/uuids.py exactly) ──
static const char *OMI_SERVICE_UUID        = "19B10000-E8F2-537E-4F6C-D104768A1214";
static const char *OMI_AUDIO_CHAR_UUID     = "19B10001-E8F2-537E-4F6C-D104768A1214";
static const char *OMI_CODEC_CHAR_UUID     = "19B10002-E8F2-537E-4F6C-D104768A1214";
// Elato-specific speaker downlink (no OMI equivalent — OMI devices have no speaker). The
// relay writes Opus 24 kHz frames here; framing is a 1-byte opcode (see SpeakerCharCallbacks).
static const char *ELATO_SPEAKER_CHAR_UUID = "19B10004-E8F2-537E-4F6C-D104768A1214";
static const char *OMI_BUTTON_SERVICE_UUID = "23BA7924-0000-1000-7450-346EAC492E92";
static const char *OMI_BUTTON_CHAR_UUID    = "23BA7925-0000-1000-7450-346EAC492E92";

static const uint8_t CODEC_OPUS = 20;  // friend-lite mapping: 0=PCM16, 1=PCM8, 20=Opus

// Speaker-write opcodes (byte 0 of every write to the speaker characteristic).
static const uint8_t SPK_OP_START = 0x01;  // speak-start: reset + arm playback (no payload)
static const uint8_t SPK_OP_END   = 0x02;  // speak-end: drain then stop (no payload)
static const uint8_t SPK_OP_STOP  = 0x03;  // speak-stop / barge-in: flush now (no payload)
static const uint8_t SPK_OP_AUDIO = 0x10;  // [0x10][flags][opus...]; flags bit0 = final fragment

static NimBLEServer *gServer = nullptr;
static NimBLECharacteristic *gAudioChar = nullptr;
static NimBLECharacteristic *gButtonChar = nullptr;
static NimBLECharacteristic *gSpeakerChar = nullptr;
static volatile bool gConnected = false;
static volatile bool gAudioSubscribed = false;  // central has enabled the audio CCCD
static uint16_t gSeq = 0;

// Reassembly buffer for fragmented Opus frames (writes can be smaller than a full packet at
// the negotiated MTU). Written only from the NimBLE host task (single writer), so no lock.
static uint8_t gSpkAsm[1024];
static size_t gSpkAsmLen = 0;

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
        gAudioSubscribed = false;
        if (deviceState != SLEEP) deviceState = IDLE;
        Serial.printf("[BLE] central disconnected (reason %d); re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

// Only stream audio once the central has subscribed (CCCD notify bit set). Notifying before
// the client finishes GATT discovery + subscribes floods the link during connection setup,
// which can stall CoreBluetooth's service discovery and time out the connect.
class AudioCharCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo &info, uint16_t subValue) override {
        gAudioSubscribed = (subValue & 0x0001) != 0;  // bit0 = notifications enabled
        Serial.printf("[BLE] audio %ssubscribed (subValue=%u)\n",
                      gAudioSubscribed ? "" : "un", subValue);
    }
};

// Speaker downlink: the relay writes opcode-framed Opus here; we reassemble fragments and feed
// the existing speaker path (Audio.cpp speaker* fns -> spkRing -> audioStreamTask -> I2S).
class SpeakerCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
        const std::string v = c->getValue();
        if (v.empty()) return;
        const uint8_t *p = (const uint8_t *)v.data();
        const size_t n = v.size();
        switch (p[0]) {
            case SPK_OP_START: gSpkAsmLen = 0; speakerBegin(); break;
            case SPK_OP_END:   speakerEnd();   break;
            case SPK_OP_STOP:  gSpkAsmLen = 0; speakerStop(); break;
            case SPK_OP_AUDIO: {
                if (n < 2) break;                      // need opcode + flags
                const bool final = (p[1] & 0x01) != 0;
                const size_t dlen = n - 2;
                if (gSpkAsmLen + dlen > sizeof(gSpkAsm)) {  // overrun: drop this frame
                    gSpkAsmLen = 0;
                    Serial.println("[spk/ble] frame too large, dropped");
                    break;
                }
                memcpy(gSpkAsm + gSpkAsmLen, p + 2, dlen);
                gSpkAsmLen += dlen;
                if (final) {
                    speakerFeedOpus(gSpkAsm, gSpkAsmLen);
                    gSpkAsmLen = 0;
                }
                break;
            }
            default:
                Serial.printf("[spk/ble] unknown opcode 0x%02X\n", p[0]);
                break;
        }
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

    // Audio service: mic audio NOTIFY + codec READ + speaker WRITE (downlink).
    NimBLEService *audioSvc = gServer->createService(OMI_SERVICE_UUID);
    gAudioChar = audioSvc->createCharacteristic(OMI_AUDIO_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    gAudioChar->setCallbacks(new AudioCharCallbacks());
    NimBLECharacteristic *codecChar =
        audioSvc->createCharacteristic(OMI_CODEC_CHAR_UUID, NIMBLE_PROPERTY::READ);
    codecChar->setValue(&CODEC_OPUS, 1);
    // Speaker downlink: WRITE_NR for streaming throughput (relay paces frames), WRITE so a
    // client can also use acked writes. Opcode-framed Opus reassembled in SpeakerCharCallbacks.
    gSpeakerChar = audioSvc->createCharacteristic(
        ELATO_SPEAKER_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    gSpeakerChar->setCallbacks(new SpeakerCharCallbacks());
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
    Serial.printf("[BLE] advertising as \"%s\" (OMI mic 16 kHz + speaker 24 kHz, Opus)\n",
                  deviceName);
}

bool bleIsConnected() { return gConnected; }

void bleNotifyAudioFrame(const uint8_t *opus, size_t len) {
    if (!gConnected || !gAudioSubscribed || !gAudioChar || len == 0) return;
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
