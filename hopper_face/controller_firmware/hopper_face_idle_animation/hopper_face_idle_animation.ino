#include <Adafruit_NeoPixel.h>

#define PIN            4
#define NUMPIXELS      40
#define PIXELS_BIGGER 24
#define PIXELS_SMALLER 16

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 35;

void setup() {
  pixels.begin();
  all_white();
}

void all_white(){
  for(int i=0;i<NUMPIXELS;i++){
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.
  }
  pixels.show(); 
}

void run_color(uint32_t color){
  for(int i=0;i<PIXELS_BIGGER;i++){
    pixels.setPixelColor(i, color);
    pixels.show(); 
    delay(delayval);
  }

  for(int i=PIXELS_BIGGER + PIXELS_SMALLER - 1;i>=PIXELS_BIGGER;i--){
    pixels.setPixelColor(i, color); 
    pixels.show(); 
    delay(delayval);
  }
}

void loop() {
  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.

  run_color(pixels.Color(0, 20, 0));
  run_color(pixels.Color(20, 0, 0));
  run_color(pixels.Color(0, 0, 20));
  run_color(pixels.Color(20, 0, 20));
  run_color(pixels.Color(0, 20, 20));
  run_color(pixels.Color(20, 20, 0));
  run_color(pixels.Color(20, 20, 20));
  run_color(pixels.Color(0, 60, 20));
  run_color(pixels.Color(20, 60, 60));
}
