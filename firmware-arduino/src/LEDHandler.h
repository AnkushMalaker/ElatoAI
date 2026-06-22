#ifndef LEDHANDLER_H
#define LEDHANDLER_H

#include "Config.h"

void setLEDColor(uint8_t r, uint8_t g, uint8_t b);
// Show an RGB colour for durationMs, overriding the status colour (led-control).
void setLedOverride(uint8_t r, uint8_t g, uint8_t b, uint32_t durationMs);
void turnOffLED();
void turnOnLED();
void setupRGBLED();
void turnOnBlueLED();
void turnOnRedLEDFlash();
void ledTask(void *parameter);

#endif