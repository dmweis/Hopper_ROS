#include <Adafruit_NeoPixel.h>


#define NEOPIXEL_PIN 4

#define NUMBER_OF_PIXELS 40
#define PIXELS_ON_BIGGER_RING 24
#define PIXELS_ON_SMALLER_RING 16

#define RESET_VALUE 121
#define RESET_COUNTER_MAX 240

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMBER_OF_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

int pinToUpdate = 0;
int resetValueCounter = 0;

void setup() {
  pixels.begin();
  Serial.begin(115200);
  all_off();
}

void set_all_pixels(uint32_t color){
  for(int i=0;i<NUMBER_OF_PIXELS;i++){
    pixels.setPixelColor(i, color);
  }
  pixels.show(); 
}

void all_off(){
  set_all_pixels(pixels.Color(0,0,0));
}

void loop() {
  if (Serial.available() >= 3){
    byte red = Serial.read();
    byte green = Serial.read();
    byte blue = Serial.read();
    uint32_t newColor = pixels.Color(red, green, blue);
    pixels.setPixelColor(pinToUpdate, newColor);
    pinToUpdate++;
    if (pinToUpdate >= NUMBER_OF_PIXELS){
      pixels.show();
      pinToUpdate = 0;
    }

    // reset logic
    if (red == RESET_VALUE && green == RESET_VALUE && blue == RESET_VALUE){
      resetValueCounter+=3;
      if (resetValueCounter >= RESET_COUNTER_MAX){
        all_off();
        delay(500);
        while(Serial.available() >= 1){
          Serial.read();
        }
        pinToUpdate = 0;
        resetValueCounter = 0;
      }
    } else {
      resetValueCounter = 0;
    }
  }
}
