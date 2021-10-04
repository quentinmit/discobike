#ifndef _EFFECTS_H
#define _EFFECTS_H

#include <Adafruit_NeoPixel.h>

void colorWipe(Adafruit_NeoPixel &strip, uint32_t frame, uint32_t color);
void theaterChase(Adafruit_NeoPixel &strip, uint32_t frame, uint32_t color);
void rainbow(Adafruit_NeoPixel &strip, uint32_t frame);
void theaterChaseRainbow(Adafruit_NeoPixel &strip, uint32_t frame);

#endif
