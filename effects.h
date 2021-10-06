#ifndef _EFFECTS_H
#define _EFFECTS_H

#include <Adafruit_NeoPixel.h>

void colorWipe(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed, uint32_t color);
void theaterChase(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed, uint32_t color);
void rainbow(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed);
void theaterChaseRainbow(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed);
void cylonBounce(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed, uint32_t color);

#endif
