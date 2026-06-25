#include "FactoryReset.h"
#include "LEDHandler.h"
#include "OTA.h"
#include "WifiManager.h"
#include "Transport.h"
#include "BLEManager.h"
#include <driver/touch_sensor.h>
#include <esp_sleep.h>

#define TOUCH_THRESHOLD 28000
#define REQUIRED_RELEASE_CHECKS                                                \
  100 // how many consecutive times we need "below threshold" to confirm release
#define TOUCH_DEBOUNCE_DELAY 500 // milliseconds

AsyncWebServer webServer(80);
WIFIMANAGER WifiManager;
esp_err_t getErr = ESP_OK;
TaskHandle_t touchTaskHandle = NULL;

// Main Thread -> onButtonLongPressUpEventCb -> enterSleep()
// Main Thread -> onButtonDoubleClickCb -> enterSleep()
// Touch Task -> touchTask -> enterSleep()
// Main Thread -> loop() (inactivity timeout) -> enterSleep()
void enterSleep() {
  Serial.println("Going to sleep...");

  // First, change device state to prevent any new data processing
  deviceState = SLEEP;
  scheduleListeningRestart = false;
  i2sOutputFlushScheduled = true;
  i2sInputFlushScheduled = true;
  vTaskDelay(10); // let all tasks accept state

  xSemaphoreTake(wsMutex, portMAX_DELAY);

  // Stop audio tasks first
  i2s_stop(I2S_PORT_IN);
  i2s_stop(I2S_PORT_OUT);

  // Properly disconnect WebSocket and wait for it to complete
  if (webSocket.isConnected()) {
    webSocket.disconnect();
    // Give some time for the disconnect to process
  }
  xSemaphoreGive(wsMutex);
  delay(100);

  // Stop all tasks that might be using I2S or other peripherals
  i2s_driver_uninstall(I2S_PORT_IN);
  i2s_driver_uninstall(I2S_PORT_OUT);

  // Flush any remaining serial output
  Serial.flush();

#ifdef TOUCH_MODE
  // Stop the touch task from polling the pad while we reconfigure the touch
  // FSM for sleep wakeup. enterSleep() runs on the main loop task, but
  // touchTask keeps calling touchRead() every 20ms on its own; a concurrent
  // read racing touchSleepWakeUpEnable() leaves the wakeup misconfigured so
  // the device never wakes on touch. Suspend it and let any in-flight read
  // finish before we touch the peripheral.
  if (touchTaskHandle != NULL) {
    vTaskSuspend(touchTaskHandle);
    delay(50);
  }

  // Wait for the finger to come off the pad before arming wakeup.
  while (touchRead(TOUCH_PAD_NUM2) > TOUCH_THRESHOLD) {
    delay(50);
  }
  delay(500);

  // Arm touch wakeup. CRUCIAL: the sleep wake threshold is a DELTA above the
  // sleep-channel benchmark (the auto-tracked untouched baseline), NOT an
  // absolute touchRead() value. A finger only adds a few thousand counts above
  // benchmark -- and far fewer on battery: with USB unplugged the board ground
  // floats, so the finger couples to ground much more weakly than when USB ties
  // the board to mains earth. The old absolute-style threshold (~25k) was only
  // ever crossable with USB's earth reference, which is exactly why wake worked
  // plugged in but never on battery. Set a small delta relative to the measured
  // sleep benchmark so a floating-ground touch still crosses it.
  touchSleepWakeUpEnable(TOUCH_PAD_NUM2, 1500); // full sleep-channel init
  delay(20);                                    // let the benchmark settle
  uint32_t sleepBenchmark = 0;
  touch_pad_sleep_channel_read_benchmark(TOUCH_PAD_NUM2, &sleepBenchmark);
  uint32_t wakeDelta = sleepBenchmark / 10; // ~10% of benchmark
  if (wakeDelta < 1500) {
    wakeDelta = 1500; // floor: stay above noise
  }
  touch_pad_sleep_set_threshold(TOUCH_PAD_NUM2, wakeDelta);
  Serial.printf("Touch sleep wakeup armed: benchmark=%u, wakeDelta=%u\n",
                sleepBenchmark, wakeDelta);
  Serial.flush();
#endif

  esp_deep_sleep_start();
  delay(1000);
}

void processSleepRequest() {
  if (sleepRequested) {
    sleepRequested = false;
    enterSleep(); // Just call it directly - no state checking needed
  }
}

void printOutESP32Error(esp_err_t err) {
  switch (err) {
  case ESP_OK:
    Serial.println("ESP_OK no errors");
    break;
  case ESP_ERR_INVALID_ARG:
    Serial.println("ESP_ERR_INVALID_ARG if the selected GPIO is not an RTC "
                   "GPIO, or the mode is invalid");
    break;
  case ESP_ERR_INVALID_STATE:
    Serial.println("ESP_ERR_INVALID_STATE if wakeup triggers conflict or "
                   "wireless not stopped");
    break;
  default:
    Serial.printf("Unknown error code: %d\n", err);
    break;
  }
}

// Button (non-touch) callbacks -> Chronicle button-events. Long hold -> sleep.
static void onButtonSingleClickCb(void *button_handle, void *usr_data) {
  sendButtonEvent("SINGLE_PRESS");
}

static void onButtonDoubleClickCb(void *button_handle, void *usr_data) {
  sendButtonEvent("DOUBLE_PRESS");
}

static void onButtonLongPressUpEventCb(void *button_handle, void *usr_data) {
  Serial.println("Button long press -> sleep");
  delay(10);
  sleepRequested = true;
}

void getAuthTokenFromNVS() {
  preferences.begin("auth", false);
  authTokenGlobal = preferences.getString("auth_token", "");
  preferences.end();
}

void setupWiFi() {
#ifdef CHRONICLE_MODE
  // Bake in the device's network so it joins without the captive portal.
  // (apList is empty on each boot, so this stays a single NVS entry.)
  WifiManager.addWifi(WIFI_SSID, WIFI_PASS);
#endif
  WifiManager.startBackgroundTask(
      "ELATO-DEVICE"); // Run the background task to take care of our Wifi
  WifiManager.fallbackToSoftAp(
      true); // Run a SoftAP if no known AP can be reached
  WifiManager.attachWebServer(&webServer); // Attach our API to the Webserver
  WifiManager.attachUI();                  // Attach the UI to the Webserver

  // Run the Webserver and add your webpages to it
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/wifi");
  });
  webServer.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
  });
  webServer.begin();
}

// Touch -> Chronicle gestures:
//   short tap                 -> SINGLE_PRESS
//   two quick taps (<400ms)   -> DOUBLE_PRESS
//   long hold (>1.5s)         -> sleep
//   tap, then press-and-hold  -> swap transport (WiFi<->BLE) + reboot   [double-tap-hold]
//
// The swap gesture is the SECOND press of a double being held; the sleep gesture is a
// lone press being held. We distinguish them by whether a first tap is still pending, so
// holding the second press triggers a swap instead of sleep.
void touchTask(void *parameter) {
  touch_pad_init();
  touch_pad_config(TOUCH_PAD_NUM2);

  bool touched = false;
  bool gestureHandled = false;  // this press already fired sleep/swap; ignore further holds
  bool isSecondPress = false;   // current press is the 2nd of a potential double
  bool tapPending = false;      // a first short tap is awaiting its partner
  unsigned long pressStart = 0;
  unsigned long lastRelease = 0;
  unsigned long tapFirstTime = 0;
  const unsigned long LONG_PRESS_MS = 1500;  // lone hold -> sleep
  const unsigned long SWAP_HOLD_MS = 700;    // 2nd-press hold -> transport swap
  const unsigned long DOUBLE_GAP_MS = 400;
  const unsigned long TAP_MAX_MS = 800;
  const unsigned long DEBOUNCE_MS = 50;

  while (1) {
    uint32_t touchValue = touchRead(TOUCH_PAD_NUM2);
    bool isTouched = (touchValue > TOUCH_THRESHOLD);
    unsigned long now = millis();

    // Press edge
    if (isTouched && !touched && (now - lastRelease > DEBOUNCE_MS)) {
      touched = true;
      pressStart = now;
      gestureHandled = false;
      isSecondPress = (tapPending && (now - tapFirstTime <= DOUBLE_GAP_MS));
    }

    // Hold handling: 2nd-press hold -> swap; lone-press hold -> sleep
    if (touched && isTouched && !gestureHandled) {
      unsigned long held = now - pressStart;
      if (isSecondPress && held >= SWAP_HOLD_MS) {
        gestureHandled = true;
        tapPending = false;
        requestTransportSwap();  // persists new mode + reboots (does not return)
      } else if (!isSecondPress && held >= LONG_PRESS_MS) {
        gestureHandled = true;
        sleepRequested = true;
      }
    }

    // Release edge
    if (!isTouched && touched) {
      touched = false;
      lastRelease = now;
      unsigned long dur = now - pressStart;
      if (gestureHandled) {
        isSecondPress = false;  // already acted
      } else if (isSecondPress) {
        // 2nd press released before the swap-hold threshold -> a normal double tap
        tapPending = false;
        isSecondPress = false;
        sendButtonEvent("DOUBLE_PRESS");
      } else if (dur < TAP_MAX_MS) {
        // first short tap -> wait for a partner press
        tapPending = true;
        tapFirstTime = now;
      }
    }

    // Resolve a lone single tap once the double-tap window passes
    if (tapPending && !touched && (now - tapFirstTime > DOUBLE_GAP_MS)) {
      tapPending = false;
      sendButtonEvent("SINGLE_PRESS");
    }

    vTaskDelay(20);
  }
  vTaskDelete(NULL);
}

void setupDeviceMetadata() {
  // factoryResetDevice();
  // resetAuth();

  deviceState = IDLE;

  getAuthTokenFromNVS();
  getOTAStatusFromNVS();

  if (otaState == OTA_IN_PROGRESS || otaState == OTA_COMPLETE) {
    deviceState = OTA;
  }
  if (factory_reset_status) {
    deviceState = FACTORY_RESET;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Report why we booted so touch wakeup from deep sleep is verifiable.
  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  if (wakeupCause == ESP_SLEEP_WAKEUP_TOUCHPAD) {
    Serial.println("Woke up from touchpad deep sleep.");
  } else {
    Serial.printf("Normal startup (wake cause %d).\n", (int)wakeupCause);
  }

  // SETUP
  setupDeviceMetadata();
  wsMutex = xSemaphoreCreateMutex();

// INTERRUPT
#ifdef TOUCH_MODE
  xTaskCreate(touchTask, "Touch Task", 4096, NULL, configMAX_PRIORITIES - 2,
              &touchTaskHandle);
#else
  getErr = esp_sleep_enable_ext0_wakeup(BUTTON_PIN, LOW);
  printOutESP32Error(getErr);
  Button *btn = new Button(BUTTON_PIN, false);
  btn->attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn->attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn->attachLongPressUpEventCb(&onButtonLongPressUpEventCb, NULL);
#endif

  // Choose transport for this boot (reboot-swap; default WiFi). See Transport.h.
  // Double-tap-hold the touch pad (touchTask above) to swap and reboot into the other one.
  // MUST run before micTask is created: micTask is higher priority than setup() and
  // captures transportMode once at start, so resolving it late makes the mic boot in the
  // wrong (default WiFi) mode and never notify over BLE.
  transportMode = loadTransportMode();
  Serial.printf("[BOOT] transport = %s\n",
                transportMode == TRANSPORT_BLE ? "BLE" : "WiFi");

  // LED + mic run under every transport. micTask needs a big stack in BLE mode because the
  // Opus encoder runs there: opus_encode is very stack-hungry (8 KB overflowed and reset the
  // device the instant a central connected and encoding began), so size for the worst case.
  xTaskCreatePinnedToCore(ledTask, "LED Task", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(micTask, "Microphone Task", 24576, NULL, 4, NULL, 1);

  if (transportMode == TRANSPORT_BLE) {
    // OMI-compatible BLE peripheral: mic (Opus) + buttons. No WiFi / WebSocket / speaker
    // downlink in this mode (OMI BLE clients don't push audio back to the device).
    setLedOverride(0, 0, 255, 1500);  // blue = BLE
    bleSetup("Elato");
  } else {
    // WiFi + Wyoming WebSocket, full duplex (includes speaker downlink).
    setLedOverride(0, 255, 0, 1500);  // green = WiFi
    xTaskCreatePinnedToCore(audioStreamTask, "Speaker Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(networkTask, "Websocket Task", 8192, NULL,
                            configMAX_PRIORITIES - 1, &networkTaskHandle, 0);
    setupWiFi();
  }
}

void loop() {
  processSleepRequest();
  if (otaState == OTA_IN_PROGRESS) {
    loopOTA();
  }
}