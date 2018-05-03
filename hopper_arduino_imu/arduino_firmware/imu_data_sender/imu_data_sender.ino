#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();


void setup() {
  Serial.begin(115200);
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void loop() {
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);

  Serial.print("{\"accel\":{\"x\":");
  Serial.print(accel.acceleration.x);
  Serial.print(",\"y\":");
  Serial.print(accel.acceleration.y);
  Serial.print(",\"z\":");
  Serial.print(accel.acceleration.z);
  Serial.print("},\"mag\":{\"x\":");
  Serial.print(mag.magnetic.x);
  Serial.print(",\"y\":");
  Serial.print(mag.magnetic.y);
  Serial.print(",\"z\":");
  Serial.print(mag.magnetic.z);
  Serial.print("},\"gyro\":{\"x\":");
  Serial.print(gyro.gyro.x);
  Serial.print(",\"y\":");
  Serial.print(gyro.gyro.y);
  Serial.print(",\"z\":");
  Serial.print(gyro.gyro.z);
  Serial.print("},\"temp\":");
  Serial.print(temp.temperature);
  Serial.println("}");
  delay(20);
}
