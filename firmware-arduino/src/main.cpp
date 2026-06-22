#include "FactoryReset.h"
#include "LEDHandler.h"
#include "OTA.h"
#include "WifiManager.h"
#include <driver/touch_sensor.h>

#define TOUCH_THRESHOLD 28000
#define REQUIRED_RELEASE_CHECKS                                                \
  100 // how many consecutive times we need "below threshold" to confirm release
#define TOUCH_DEBOUNCE_DELAY 500 // milliseconds

AsyncWebServer webServer(80);
WIFIMANAGER WifiManager;
esp_err_t getErr = ESP_OK;

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
  touch_pad_intr_disable(TOUCH_PAD_INTR_MASK_ALL);
  while (touchRead(TOUCH_PAD_NUM2) > TOUCH_THRESHOLD) {
    delay(50);
  }
  delay(500);
  touchSleepWakeUpEnable(TOUCH_PAD_NUM2, TOUCH_THRESHOLD);
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

// Touch -> Chronicle button-events:
//   short tap          -> SINGLE_PRESS
//   two taps (<400ms)  -> DOUBLE_PRESS
//   long hold (>1.5s)  -> sleep
void touchTask(void *parameter) {
  touch_pad_init();
  touch_pad_config(TOUCH_PAD_NUM2);

  bool touched = false;
  bool longHandled = false;
  bool tapPending = false;
  unsigned long pressStart = 0;
  unsigned long lastRelease = 0;
  unsigned long tapFirstTime = 0;
  const unsigned long LONG_PRESS_MS = 1500;
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
      longHandled = false;
    }

    // Long hold -> sleep
    if (touched && isTouched && !longHandled &&
        (now - pressStart >= LONG_PRESS_MS)) {
      longHandled = true;
      sleepRequested = true;
    }

    // Release edge
    if (!isTouched && touched) {
      touched = false;
      lastRelease = now;
      unsigned long dur = now - pressStart;
      if (!longHandled && dur < TAP_MAX_MS) {
        if (tapPending && (now - tapFirstTime <= DOUBLE_GAP_MS)) {
          tapPending = false;
          sendButtonEvent("DOUBLE_PRESS");
        } else {
          tapPending = true;
          tapFirstTime = now;
        }
      }
    }

    // Resolve a single tap once the double-tap window passes
    if (tapPending && (now - tapFirstTime > DOUBLE_GAP_MS)) {
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

  // SETUP
  setupDeviceMetadata();
  wsMutex = xSemaphoreCreateMutex();

// INTERRUPT
#ifdef TOUCH_MODE
  xTaskCreate(touchTask, "Touch Task", 4096, NULL, configMAX_PRIORITIES - 2,
              NULL);
#else
  getErr = esp_sleep_enable_ext0_wakeup(BUTTON_PIN, LOW);
  printOutESP32Error(getErr);
  Button *btn = new Button(BUTTON_PIN, false);
  btn->attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn->attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn->attachLongPressUpEventCb(&onButtonLongPressUpEventCb, NULL);
#endif

  // Pin audio tasks to Core 1 (application core)
  xTaskCreatePinnedToCore(ledTask,    // Function
                          "LED Task", // Name
                          4096,       // Stack size
                          NULL,       // Parameters
                          5,          // Priority
                          NULL,       // Handle
                          1           // Core 1 (application core)
  );

  xTaskCreatePinnedToCore(audioStreamTask, // Function
                          "Speaker Task",  // Name
                          4096,            // Stack size
                          NULL,            // Parameters
                          3,               // Priority
                          NULL,            // Handle
                          1                // Core 1 (application core)
  );

  xTaskCreatePinnedToCore(micTask,           // Function
                          "Microphone Task", // Name
                          4096,              // Stack size
                          NULL,              // Parameters
                          4,                 // Priority
                          NULL,              // Handle
                          1                  // Core 1 (application core)
  );

  // Pin network task to Core 0 (protocol core)
  xTaskCreatePinnedToCore(networkTask,              // Function
                          "Websocket Task",         // Name
                          8192,                     // Stack size
                          NULL,                     // Parameters
                          configMAX_PRIORITIES - 1, // Highest priority
                          &networkTaskHandle,       // Handle
                          0                         // Core 0 (protocol core)
  );

  // WIFI
  setupWiFi();
}

void loop() {
  processSleepRequest();
  if (otaState == OTA_IN_PROGRESS) {
    loopOTA();
  }
}