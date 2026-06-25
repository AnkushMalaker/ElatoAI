/**
 * Audio.cpp - Chronicle transport for the Elato device.
 *
 * Chronicle's Wyoming-over-WebSocket, full duplex:
 *   - Mic OUT: continuous 16 kHz / 16-bit mono PCM (INMP441 read as 32-bit + gain),
 *     sent as Wyoming `audio-chunk` (JSON text frame + binary payload) over /ws?codec=pcm.
 *   - Speaker IN: streamed Opus (Elato-style) - small binary Opus packets decoded into a
 *     ring buffer and played gaplessly. Bracketed by `speak-start` / `speak-end` text
 *     control frames. Avoids the 15 KB single-frame WebSocket limit and bounds RAM.
 *   - LED:    inbound `led-control` overrides the status LED briefly.
 *   - Button: sendButtonEvent() emits Wyoming `button-event`.
 *
 * Threading: webSocket.loop() runs in networkTask while holding wsMutex and dispatches
 * webSocketEvent(); handlers called from it must NOT re-take wsMutex. Opus decode happens
 * inline there (fast) and writes PCM into `spkRing`. audioStreamTask is the sole consumer
 * of spkRing and the sole I2S-out writer (SPSC with networkTask).
 */
#include "Audio.h"
#include "LEDHandler.h"
#include "Transport.h"
#include "BLEManager.h"
#include <WiFi.h>
#include <math.h>
#include <opus.h>

// WEBSOCKET
SemaphoreHandle_t wsMutex;
WebSocketsClient webSocket;

// TASK HANDLES
TaskHandle_t speakerTaskHandle = NULL;
TaskHandle_t micTaskHandle = NULL;
TaskHandle_t networkTaskHandle = NULL;

// Compat (used by main.cpp enterSleep); not part of the Chronicle flow.
volatile bool scheduleListeningRestart = false;
unsigned long speakingStartTime = 0;
volatile bool i2sOutputFlushScheduled = false;
volatile bool i2sInputFlushScheduled = false;

// AUDIO SETTINGS
int currentVolume = 80;
const int CHANNELS = 1;          // Mono
const int BITS_PER_SAMPLE = 16;  // 16-bit audio

// Speaker streaming runs at a fixed rate (matches the backend Opus encoder).
static const uint32_t SPK_RATE = 24000;

// Defined in WifiManager.cpp - (re)authenticates and refreshes authTokenGlobal.
bool isDeviceRegistered();

// I2S streams (AudioTools)
static I2SStream i2sOut;                 // speaker - access from audioStreamTask only
static VolumeStream volumeOut(i2sOut);   // volume wrapper over the speaker
static I2SStream i2sInput;               // mic - access from micTask only

// Connection / login state
static volatile bool wsConnected = false;
static volatile bool needRelogin = false;

// ── speaker streaming state ─────────────────────────────────────
// Ring of decoded 24 kHz/16-bit PCM. Producer: networkTask (opus decode in webSocketEvent).
// Consumer: audioStreamTask. BufferRTOS is single-producer/single-consumer safe.
static BufferRTOS<uint8_t> spkRing(32 * 1024, 1024);
static OpusDecoder *opusDec = nullptr;
static volatile bool speakActive = false;  // between speak-start and drain
static volatile bool speakEnded = false;   // speak-end seen; drain then stop
static volatile uint32_t lastSpeakEndMs = 0;  // millis() when playback last stopped

// Mic-duck guard: while we're playing our own audio (and briefly after, to cover the
// speaker/room decay) we stop sending mic frames upstream. This is acoustic-echo
// ducking without true AEC — it keeps the device from transcribing its own TTS or
// re-triggering the wake word on its own output. Trade-off: no barge-in mid-playback.
#ifndef MIC_ECHO_GUARD_MS
#define MIC_ECHO_GUARD_MS 250
#endif
static inline bool micDucked() {
    if (speakActive) return true;
    uint32_t last = lastSpeakEndMs;
    return last != 0 && (millis() - last) < MIC_ECHO_GUARD_MS;
}

unsigned long getSpeakingDuration() {
    if (deviceState == SPEAKING && speakingStartTime > 0) {
        return millis() - speakingStartTime;
    }
    return 0;
}

// ── outbound helpers ────────────────────────────────────────────
// Raw (no mutex) - only call from inside webSocketEvent, where wsMutex is held.
static void sendAudioStartRaw() {
    static const char *m =
        "{\"type\":\"audio-start\",\"data\":{\"rate\":16000,\"width\":2,\"channels\":1,"
        "\"mode\":\"streaming\"},\"payload_length\":0}";
    webSocket.sendTXT((uint8_t *)m, strlen(m));
    Serial.println("[WS] audio-start sent");
}

// Button event - BLE notify (OMI button char) or Wyoming button-event over WS, per transport.
// Called from the touch/button task.
void sendButtonEvent(const char *state) {
    if (transportMode == TRANSPORT_BLE) {
        if (!bleIsConnected()) return;
        uint32_t s = 0;
        if (!strcmp(state, "SINGLE_PRESS")) s = 1;
        else if (!strcmp(state, "DOUBLE_PRESS")) s = 2;
        else if (!strcmp(state, "LONG_PRESS")) s = 3;
        bleNotifyButton(s);
        Serial.printf("[BLE] button-event %s\n", state);
        return;
    }
    if (!wsConnected) return;
    char msg[128];
    int n = snprintf(msg, sizeof(msg),
        "{\"type\":\"button-event\",\"data\":{\"state\":\"%s\"},\"payload_length\":0}", state);
    xSemaphoreTake(wsMutex, portMAX_DELAY);
    webSocket.sendTXT((uint8_t *)msg, n);
    xSemaphoreGive(wsMutex);
    Serial.printf("[WS] button-event %s\n", state);
}

// ── inbound: led-control ────────────────────────────────────────
static void handleLedControl(JsonVariantConst d) {
    // Backend may send rgb as 0..1 floats (with brightness) or 0..255 ints.
    float r = d["r"] | 0.0f, g = d["g"] | 0.0f, b = d["b"] | 0.0f;
    float br = d["brightness"] | 1.0f;
    float dur = d["duration"] | 5.0f;
    float scale = (r <= 1.0f && g <= 1.0f && b <= 1.0f) ? 255.0f : 1.0f;
    setLedOverride((uint8_t)(r * scale * br), (uint8_t)(g * scale * br),
                   (uint8_t)(b * scale * br), (uint32_t)(dur * 1000.0f));
}

static void handleTextMessage(const uint8_t *payload, size_t length) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
        Serial.printf("[WS] bad JSON text: %s\n", err.c_str());
        return;
    }
    const char *mtype = doc["type"] | "";
    if (strcmp(mtype, "speak-start") == 0) {
        // Newest-wins: drop any audio still buffered from a superseded clip so only
        // the newest reply plays (the backend cancels the old stream; this clears its
        // tail). Safe to reset here: this runs in networkTask (the ring's producer),
        // and the consumer (audioStreamTask) polls available() rather than blocking.
        spkRing.reset();
        if (opusDec) opus_decoder_ctl(opusDec, OPUS_RESET_STATE);
        speakEnded = false;
        speakActive = true;
        Serial.println("[spk] speak-start");
    } else if (strcmp(mtype, "speak-end") == 0) {
        speakEnded = true;  // audioStreamTask drains the ring then stops
        Serial.println("[spk] speak-end");
    } else if (strcmp(mtype, "speak-stop") == 0) {
        // Barge-in / hard stop (e.g. a button press): drop any buffered audio and
        // end playback now instead of draining. Same producer-side reset as
        // speak-start; emptying the ring + marking ended makes audioStreamTask
        // flush I2S and power the amp down on its next pass (~40 ms).
        spkRing.reset();
        if (opusDec) opus_decoder_ctl(opusDec, OPUS_RESET_STATE);
        speakEnded = true;
        Serial.println("[spk] speak-stop");
    } else if (strcmp(mtype, "led-control") == 0) {
        handleLedControl(doc["data"]);
    } else if (strcmp(mtype, "pong") == 0) {
        // heartbeat ack - ignore
    } else {
        Serial.printf("[WS] text type=%s (ignored)\n", mtype);
    }
}

// ── inbound: Opus audio packet (binary frame) ───────────────────
// Decode 120 ms Opus packets to 24 kHz PCM and push into the speaker ring.
static int16_t opusPcm[5760];  // max decoded samples per packet (120 ms @ 48 kHz headroom)
static void handleOpusPacket(const uint8_t *pkt, size_t len) {
    if (!opusDec) return;
    int samples = opus_decode(opusDec, pkt, (opus_int32)len, opusPcm,
                              sizeof(opusPcm) / sizeof(int16_t), 0);
    if (samples > 0) {
        speakActive = true;  // first packet starts playback even if speak-start was missed
        spkRing.writeArray((uint8_t *)opusPcm, samples * (int)sizeof(int16_t));
    } else if (samples < 0) {
        Serial.printf("[spk] opus_decode err %d (len=%u)\n", samples, (unsigned)len);
    }
}

// ── BLE speaker downlink entry points ───────────────────────────
// opus_decode is stack-heavy and MUST NOT run on the NimBLE host task: its write callback
// has a small stack, and decoding there overflowed and reset the device the instant the
// first frame arrived. So the BLE write callback (BLEManager.cpp) only enqueues opcode-framed
// messages here; speakerDecodeTask (its own large stack) owns opusDec and the spkRing
// producer side and does the reset/decode/flag work. audioStreamTask drains spkRing -> I2S
// exactly as in WiFi, so mic-ducking and newest-wins playback are identical in BLE mode.
struct SpkMsg {
    uint8_t op;          // 0=start, 1=audio, 2=end, 3=stop
    uint16_t len;        // opus length (op==1 only)
    uint8_t data[512];   // one Opus packet (24 kHz/60 ms VOIP packets are well under this)
};
static QueueHandle_t spkQueue = nullptr;

// Called from the NimBLE host-task write callback — keep it cheap (copy + non-blocking send).
static void spkEnqueue(uint8_t op, const uint8_t *pkt, size_t len) {
    if (!spkQueue) return;  // decode task not up yet (very early boot) -> drop
    SpkMsg m;
    m.op = op;
    m.len = 0;
    if (op == 1) {
        if (len > sizeof(m.data)) len = sizeof(m.data);
        m.len = (uint16_t)len;
        memcpy(m.data, pkt, len);
    }
    xQueueSend(spkQueue, &m, 0);  // drop on overflow rather than stall the BLE host task
}

void speakerBegin()                                  { spkEnqueue(0, nullptr, 0); }
void speakerFeedOpus(const uint8_t *pkt, size_t len) { spkEnqueue(1, pkt, len); }
void speakerEnd()                                    { spkEnqueue(2, nullptr, 0); }
void speakerStop()                                   { spkEnqueue(3, nullptr, 0); }

// Decodes queued Opus -> spkRing on its own stack. Single producer of spkRing (audioStreamTask
// is the single consumer), so the SPSC invariant holds. opusDec is created by audioStreamTask;
// handleOpusPacket / the resets below all null-check it, so a brief startup race is safe.
void speakerDecodeTask(void *parameter) {
    spkQueue = xQueueCreate(16, sizeof(SpkMsg));
    SpkMsg m;
    for (;;) {
        if (xQueueReceive(spkQueue, &m, portMAX_DELAY) != pdTRUE) continue;
        switch (m.op) {
            case 0:  // speak-start: newest-wins reset, then arm playback
                spkRing.reset();
                if (opusDec) opus_decoder_ctl(opusDec, OPUS_RESET_STATE);
                speakEnded = false;
                speakActive = true;
                Serial.println("[spk/ble] speak-start");
                break;
            case 1:  // audio packet
                handleOpusPacket(m.data, m.len);
                break;
            case 2:  // speak-end: drain then stop
                speakEnded = true;
                Serial.println("[spk/ble] speak-end");
                break;
            case 3:  // speak-stop / barge-in: flush now
                spkRing.reset();
                if (opusDec) opus_decoder_ctl(opusDec, OPUS_RESET_STATE);
                speakEnded = true;
                Serial.println("[spk/ble] speak-stop");
                break;
        }
    }
}

// ── websocket event handler (runs inside networkTask, wsMutex held) ──
void webSocketEvent(WStype_t type, const uint8_t *payload, size_t length) {
    switch (type) {
    case WStype_DISCONNECTED:
        Serial.println("[WS] Disconnected");
        wsConnected = false;
        // Don't re-auth/reconnect if we're disconnecting on purpose to sleep.
        if (deviceState != SLEEP) {
            needRelogin = true;
            deviceState = IDLE;
        }
        break;
    case WStype_CONNECTED:
        Serial.printf("[WS] Connected: %s\n", payload);
        wsConnected = true;
        needRelogin = false;  // connected (lib auto-reconnect or manual) - cancel pending relogin
        sendAudioStartRaw();
        deviceState = LISTENING;
        break;
    case WStype_TEXT:
        handleTextMessage(payload, length);
        break;
    case WStype_BIN:
        handleOpusPacket(payload, length);
        break;
    case WStype_ERROR:
        Serial.println("[WS] Error");
        break;
    default:
        break;
    }
}

// ── connect ─────────────────────────────────────────────────────
// Called from connectCb (WifiManager) and on reconnect. Builds the Chronicle
// URL with the freshly-acquired JWT and device name.
void websocketSetup(const String& server_domain, int port, const String& /*path*/) {
    String url = "/ws?codec=pcm&token=" + authTokenGlobal +
                 "&device_name=" + String(DEVICE_NAME);

    xSemaphoreTake(wsMutex, portMAX_DELAY);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(3000);
    webSocket.disableHeartbeat();
    webSocket.begin(server_domain.c_str(), port, url.c_str());
    xSemaphoreGive(wsMutex);

    Serial.printf("[WS] connecting ws://%s:%d/ws?codec=pcm&device_name=%s\n",
                  server_domain.c_str(), port, DEVICE_NAME);
}

void networkTask(void *parameter) {
    uint32_t lastRelogin = 0;
    while (1) {
        // Refresh the JWT and reconnect only while actually disconnected (token may
        // have expired). Never force-reconnect while connected - the WebSocketsClient
        // auto-reconnect handles transient drops with the existing token.
        if (needRelogin && !wsConnected && deviceState != SLEEP &&
            WiFi.status() == WL_CONNECTED) {
            uint32_t now = millis();
            if (now - lastRelogin > 3000) {
                lastRelogin = now;
                needRelogin = false;
                Serial.println("[WS] re-authenticating + reconnecting...");
                if (isDeviceRegistered()) {  // refreshes authTokenGlobal
                    xSemaphoreTake(wsMutex, portMAX_DELAY);
                    webSocket.disconnect();
                    xSemaphoreGive(wsMutex);
                    websocketSetup(ws_server, ws_port, ws_path);
                } else {
                    needRelogin = true;  // retry next cycle
                }
            }
        }

        xSemaphoreTake(wsMutex, portMAX_DELAY);
        webSocket.loop();
        xSemaphoreGive(wsMutex);
        vTaskDelay(1);
    }
}

// ── speaker: drain the ring buffer to I2S (the sole I2S-out writer) ──
// A short boot tone (24 kHz) also routes through the ring to verify the path.
static void enqueueBootTone() {
    const int ms = 200;
    const int n = SPK_RATE * ms / 1000;
    int16_t s;
    for (int i = 0; i < n; i++) {
        s = (int16_t)(6000.0f * sinf(2.0f * 3.14159265f * 440.0f * i / SPK_RATE));
        spkRing.writeArray((uint8_t *)&s, sizeof(s));
    }
    speakActive = true;
    speakEnded = true;  // play it out then idle
    Serial.println("[spk] boot test tone (440 Hz)");
}

void audioStreamTask(void *parameter) {
    pinMode(I2S_SD_OUT, OUTPUT);
    digitalWrite(I2S_SD_OUT, LOW);  // amp off until we play

    auto cfg = i2sOut.defaultConfig(TX_MODE);
    cfg.bits_per_sample = BITS_PER_SAMPLE;
    cfg.sample_rate = SPK_RATE;
    cfg.channels = CHANNELS;
    cfg.pin_bck = I2S_BCK_OUT;
    cfg.pin_ws = I2S_WS_OUT;
    cfg.pin_data = I2S_DATA_OUT;
    cfg.port_no = I2S_PORT_OUT;
    i2sOut.begin(cfg);

    auto vcfg = volumeOut.defaultConfig();
    vcfg.copyFrom(cfg);
    vcfg.allow_boost = true;
    volumeOut.begin(vcfg);
    volumeOut.setVolume(currentVolume / 100.0f);

    int derr = 0;
    opusDec = opus_decoder_create(SPK_RATE, CHANNELS, &derr);
    if (derr != OPUS_OK || !opusDec) {
        Serial.printf("[spk] opus_decoder_create failed: %d\n", derr);
    }

    enqueueBootTone();

    bool amp = false;
    uint32_t emptySince = 0;
    uint8_t buf[1024];

    // Jitter pre-roll: buffer a small cushion of decoded PCM before starting to
    // drain, so brief gaps between inbound Opus packets don't starve the I2S DMA
    // mid-word (which clicks/crackles). The I2S write is blocking, so once we
    // start, playback is paced in real time and the cushion absorbs network
    // jitter. ~150 ms @ 24 kHz / 16-bit mono = 7200 bytes.
    const size_t PREROLL_BYTES = 7200;

    while (1) {
        size_t avail = spkRing.available();
        if (avail > 0) {
            if (!amp) {
                // Wait for the cushion before the first sample. Skip the wait if
                // the whole clip already arrived (speak-end) and it's shorter
                // than the pre-roll, so short replies still play promptly.
                if (avail < PREROLL_BYTES && !speakEnded) {
                    vTaskDelay(pdMS_TO_TICKS(5));
                    continue;
                }
                digitalWrite(I2S_SD_OUT, HIGH);  // amp on
                amp = true;
                if (deviceState != SLEEP) deviceState = SPEAKING;
                speakingStartTime = millis();
            }
            emptySince = 0;
            int n = avail < sizeof(buf) ? (int)avail : (int)sizeof(buf);
            int got = spkRing.readArray(buf, n);
            if (got > 0) volumeOut.write(buf, got);
        } else {
            if (amp) {
                // Ring momentarily empty. Keep the amp on through brief network
                // underruns; only stop once the utterance has ended (speak-end)
                // or the ring has stayed empty long enough.
                if (emptySince == 0) emptySince = millis();
                bool done = speakEnded || (millis() - emptySince > 600);
                if (done && (millis() - emptySince > 40)) {
                    i2sOut.flush();
                    vTaskDelay(pdMS_TO_TICKS(20));
                    digitalWrite(I2S_SD_OUT, LOW);  // amp off (avoid idle hiss)
                    amp = false;
                    speakActive = false;
                    speakEnded = false;
                    lastSpeakEndMs = millis();  // start the mic-duck tail guard
                    emptySince = 0;
                    if (deviceState != SLEEP) deviceState = wsConnected ? LISTENING : IDLE;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(3));
        }
    }
}

// ── mic: stream PCM as Wyoming audio-chunk ──────────────────────
// The INMP441 is a 24-bit I2S mic: read 32-bit slots and downshift to 16-bit
// with gain. (Reading 16-bit directly only keeps the top 16 of 24 bits, leaving
// the quiet signal buried in the low bits -> noisy/quiet capture.)
// MIC_GAIN_SHIFT: 16 = unity. Smaller = louder. 13 ≈ +8x.
#ifndef MIC_GAIN_SHIFT
#define MIC_GAIN_SHIFT 13
#endif

void micTask(void *parameter) {
    auto cfg = i2sInput.defaultConfig(RX_MODE);
    cfg.bits_per_sample = 32;  // INMP441 24-bit data lives in a 32-bit slot
    cfg.sample_rate = MIC_SAMPLE_RATE;  // 16000
    cfg.channels = CHANNELS;
    cfg.i2s_format = I2S_LEFT_JUSTIFIED_FORMAT;
    cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.pin_bck = I2S_SCK;
    cfg.pin_ws = I2S_WS;
    cfg.pin_data = I2S_SD;
    cfg.port_no = I2S_PORT_IN;
    i2sInput.begin(cfg);

    // Transport is fixed for this boot (reboot-swap), so branch once here:
    //   BLE : encode 60 ms (960-sample) Opus frames, one packet per BLE notification.
    //   WiFi: stream raw 16-bit PCM as Wyoming audio-chunk (original behaviour).
    const bool ble = (transportMode == TRANSPORT_BLE);
    const int FRAMES = ble ? 960 : 512;  // 60 ms Opus frame vs 32 ms PCM chunk @ 16 kHz
    static int32_t in32[960];
    static int16_t out16[960];
    static uint8_t opusBuf[256];
    char header[160];

    OpusEncoder *micEnc = nullptr;
    if (ble) {
        int err = 0;
        micEnc = opus_encoder_create(16000, 1, OPUS_APPLICATION_VOIP, &err);
        if (!micEnc || err != OPUS_OK) {
            Serial.printf("[BLE] opus encoder init failed: %d\n", err);
            vTaskDelete(NULL);
            return;
        }
        // ~16 kbps keeps a 60 ms packet near ~120 bytes, so it fits one BLE notification
        // even at the small ATT MTU iOS negotiates (payload ~182, plus our 3-byte header).
        opus_encoder_ctl(micEnc, OPUS_SET_BITRATE(16000));
        opus_encoder_ctl(micEnc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    }

    while (1) {
        bool connected = ble ? bleIsConnected() : wsConnected;
        if (!connected) {
            vTaskDelay(10);
            continue;
        }

        // I2S reads can be short; accumulate exactly FRAMES samples (Opus needs a full frame).
        int got = 0;
        while (got < FRAMES) {
            size_t nbytes = i2sInput.readBytes((uint8_t *)(in32 + got),
                                               (FRAMES - got) * sizeof(int32_t));
            int ns = nbytes / sizeof(int32_t);
            if (ns <= 0) { vTaskDelay(1); continue; }
            got += ns;
        }

        // Duck our own playback: keep draining the I2S DMA (above) but don't send the
        // mic upstream while we're speaking, so the backend never hears our own output.
        if (micDucked()) {
            vTaskDelay(1);
            continue;
        }

        for (int i = 0; i < FRAMES; i++) {
            int32_t v = in32[i] >> MIC_GAIN_SHIFT;
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            out16[i] = (int16_t)v;
        }

        if (ble) {
            int n = opus_encode(micEnc, out16, FRAMES, opusBuf, sizeof(opusBuf));
            if (n > 0) bleNotifyAudioFrame(opusBuf, (size_t)n);
            else if (n < 0) Serial.printf("[BLE] opus_encode err %d\n", n);
        } else {
            size_t outBytes = (size_t)FRAMES * sizeof(int16_t);
            int hlen = snprintf(header, sizeof(header),
                "{\"type\":\"audio-chunk\",\"data\":{\"rate\":16000,\"width\":2,\"channels\":1},"
                "\"payload_length\":%u}", (unsigned)outBytes);
            xSemaphoreTake(wsMutex, portMAX_DELAY);
            bool ok = webSocket.sendTXT((uint8_t *)header, hlen);
            if (ok) webSocket.sendBIN((uint8_t *)out16, outBytes);
            xSemaphoreGive(wsMutex);
        }

        vTaskDelay(1);
    }
}
