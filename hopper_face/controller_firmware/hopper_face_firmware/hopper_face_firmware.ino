#include <Adafruit_NeoPixel.h>
#include <PacketSerial.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

#define NEOPIXEL_PIN D5

#define NUMBER_OF_PIXELS 40
#define PIXELS_ON_BIGGER_RING 24
#define PIXELS_ON_SMALLER_RING 16

Adafruit_BNO055 bno = Adafruit_BNO055();

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMBER_OF_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
PacketSerial_<COBS, 0, 500> packetSerial;

int pinToUpdate = 0;
long lastImuUpdate = 0;

void setup() {
  pixels.begin();
  packetSerial.begin(115200);
  packetSerial.setPacketHandler(&onPacketReceived);
  all_off();
  bno.begin();
  bno.setExtCrystalUse(true);
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
  if (lastImuUpdate + BNO055_SAMPLERATE_DELAY_MS < millis()){
    lastImuUpdate = millis();
    // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.print(quat.z(), 4);
  Serial.print(",");
  Serial.print(quat.w(), 4);
  Serial.print(", ");

  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Serial.print(linear.x());
  Serial.print(",");
  Serial.print(linear.y());
  Serial.print(",");
  Serial.print(linear.z());
  Serial.println(", ");

  imu::Vector<3> rot = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print(rot.x());
  Serial.print(",");
  Serial.print(rot.y());
  Serial.print(",");
  Serial.print(rot.z());
  Serial.println(", ");

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(system, DEC);
  Serial.print(",");
  Serial.print(gyro, DEC);
  Serial.print(",");
  Serial.print(accel, DEC);
  Serial.print(",");
  Serial.println(mag, DEC);
  }
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
