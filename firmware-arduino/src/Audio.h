#ifndef AUDIO_H
#define AUDIO_H

#include "AudioTools.h"
#include "Config.h"

extern SemaphoreHandle_t wsMutex;
extern WebSocketsClient webSocket;

extern TaskHandle_t speakerTaskHandle;
extern TaskHandle_t micTaskHandle;
extern TaskHandle_t networkTaskHandle;

// Kept for compatibility with main.cpp sleep handling (unused in CHRONICLE flow).
extern volatile bool scheduleListeningRestart;
extern unsigned long speakingStartTime;

extern int currentVolume;
extern const int CHANNELS;        // Mono
extern const int BITS_PER_SAMPLE; // 16-bit audio

extern volatile bool i2sOutputFlushScheduled;
extern volatile bool i2sInputFlushScheduled;

// WEBSOCKET
void webSocketEvent(WStype_t type, const uint8_t *payload, size_t length);
void websocketSetup(const String& server_domain, int port, const String& path);
void networkTask(void *parameter);

// AUDIO
unsigned long getSpeakingDuration();
void audioStreamTask(void *parameter);  // speaker (plays inbound play-audio WAV)
void micTask(void *parameter);          // mic (streams PCM via Wyoming audio-chunk)

// Send a Wyoming button-event ("SINGLE_PRESS" / "DOUBLE_PRESS" / "LONG_PRESS").
void sendButtonEvent(const char *state);

#endif
