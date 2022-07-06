#include "effects.h"

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed, uint32_t color) {
  frame *= speed;
  // default speed is 1 frame per pixel
  frame /= 256;
  frame %= (2*strip.numPixels());
  strip.clear();
  for(int i=max(0, (int)frame-(int)strip.numPixels()); i<=min(frame, strip.numPixels()); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed, uint32_t color) {
  frame *= speed;
  // default speed is 4 frames per pixel
  frame /= 1024;
  int b = frame % 3;
  strip.clear();         //   Set all pixels in RAM to 0 (off)
  // 'c' counts up from 'b' to end of strip in steps of 3...
  for(int c=b; c<strip.numPixels(); c += 3) {
    strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed) {
  frame *= speed;
  // default speed is 128 frames (4.2 seconds) per loop
  frame /= 128;
  long firstPixelHue = (256 * frame) % 65536;
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    // Offset pixel hue by an amount to make one full revolution of the
    // color wheel (range of 65536) along the length of the strip
    // (strip.numPixels() steps):
    int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
    // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
    // optionally add saturation and value (brightness) (each 0 to 255).
    // Here we're using just the single-argument hue variant. The result
    // is passed through strip.gamma32() to provide 'truer' colors
    // before assigning to each pixel:
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed) {
  frame *= speed;
  frame /= 256;
  int firstPixelHue = (65536 / 90) * (frame % 90);     // One cycle of color wheel over 90 frames / 3 seconds
  frame >>= 2; // move at 4 frames per pixel
  int b = frame % 3; //        'b' counts from 0 to 2...
  strip.clear();         //   Set all pixels in RAM to 0 (off)
  // 'c' counts up from 'b' to end of strip in increments of 3...
  for(int c=b; c<strip.numPixels(); c += 3) {
    // hue of pixel 'c' is offset by an amount to make one full
    // revolution of the color wheel (range 65536) along the length
    // of the strip (strip.numPixels() steps):
    int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
    uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
    strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
  }
}

uint32_t colorDiv(uint32_t color, uint8_t div) {
  uint8_t *y = (uint8_t *)&color;
  for (int i = 0; i < 4; i++) {
    y[i] = y[i] / div;
  }
  return color;
}

void cylonBounce(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed, uint32_t color) {
  int EyeSize = 4;
  int ReturnDelay = 5;

  int steps = strip.numPixels()-EyeSize-2;
  uint32_t frames = 2 * (steps + ReturnDelay);

  frame *= speed;
  frame /= 256;

  frame %= frames;

  int i = min(frame, steps);
  if (frame > steps + ReturnDelay) {
    i = max(0, steps - ((int)frame - (steps + ReturnDelay)));
  }
  strip.clear();
  strip.setPixelColor(i, colorDiv(color, 10));
  for(int j = 1; j <= EyeSize; j++) {
    strip.setPixelColor(i+j, color);
  }
  strip.setPixelColor(i+EyeSize+1, colorDiv(color, 10));
}

uint32_t heatColor(byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);

  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252

  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    return Adafruit_NeoPixel::Color(255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    return Adafruit_NeoPixel::Color(255, heatramp, 0);
  } else {                               // coolest
    return Adafruit_NeoPixel::Color(heatramp, 0, 0);
  }
}

void fire(Adafruit_NeoPixel &strip, uint32_t frame, int16_t speed) {
  int Cooling = 55;
  int Sparking = 120;

  static uint32_t last_frame;
  bool upside_down = (speed < 0);
  frame *= speed;
  if ((last_frame - frame) < 256 && (last_frame < frame)) {
    // Skip frames to control speed
    return;
  }
  last_frame = frame;

  static byte* heat;
  if (heat == NULL) {
    heat = (byte*)rtos_malloc(strip.numPixels());
  }
  int cooldown;

  // Step 1.  Cool down every cell a little
  for( int i = 0; i < strip.numPixels(); i++) {
    cooldown = random(0, ((Cooling * 10) / strip.numPixels()) + 2);

    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= strip.numPixels() - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < strip.numPixels(); j++) {
    strip.setPixelColor(upside_down ? (strip.numPixels()-j-1) : j, heatColor(heat[j]) );
  }
}
