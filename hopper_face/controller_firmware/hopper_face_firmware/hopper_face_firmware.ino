#include <Adafruit_NeoPixel.h>
#include <PacketSerial.h>

#define NEOPIXEL_PIN 4

#define NUMBER_OF_PIXELS 40
#define PIXELS_ON_BIGGER_RING 24
#define PIXELS_ON_SMALLER_RING 16

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMBER_OF_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
PacketSerial_<COBS, 0, 500> packetSerial;

int pinToUpdate = 0;

void setup() {
  pixels.begin();
  packetSerial.begin(115200);
  packetSerial.setPacketHandler(&onPacketReceived);
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
  packetSerial.update();
}

void onPacketReceived(const uint8_t* buffer, size_t size){
  if (size != NUMBER_OF_PIXELS * 3){
    return;
  } 
  // Make a temporary buffer.
  uint8_t tempBuffer[size];

  // Copy the packet into our temporary buffer.
  memcpy(tempBuffer, buffer, size);

  for (int i = 0; i < size; i+=3){
    byte red = tempBuffer[i];
    byte green = tempBuffer[i+1];;
    byte blue = tempBuffer[i+2];;
    uint32_t newColor = pixels.Color(red, green, blue);
    pixels.setPixelColor(pinToUpdate, newColor);
    pinToUpdate++;
  }
  pixels.show();
  pinToUpdate = 0;
}
