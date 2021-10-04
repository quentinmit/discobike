#include "effects.h"

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(Adafruit_NeoPixel &strip, uint32_t frame, uint32_t color) {
  frame %= (2*strip.numPixels());
  strip.clear();
  for(int i=max(0, frame-strip.numPixels()); i<=min(frame, strip.numPixels()); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(Adafruit_NeoPixel &strip, uint32_t frame, uint32_t color) {
  frame >>= 1; // 2 frames per move
  int b = frame % 3;
  strip.clear();         //   Set all pixels in RAM to 0 (off)
  // 'c' counts up from 'b' to end of strip in steps of 3...
  for(int c=b; c<strip.numPixels(); c += 3) {
    strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(Adafruit_NeoPixel &strip, uint32_t frame) {
  long firstPixelHue = (256 * frame) % (5*65536);
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
void theaterChaseRainbow(Adafruit_NeoPixel &strip, uint32_t frame) {
  int firstPixelHue = (65536 / 90) * (frame % 90);     // One cycle of color wheel over 90 frames
  frame >>= 1; // 2 frames per move
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
