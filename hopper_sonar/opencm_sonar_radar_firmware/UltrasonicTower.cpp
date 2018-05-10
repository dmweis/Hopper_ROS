#include "wirish.h"
#include "UltrasonicTower.h"

UltrasonicTower::UltrasonicTower(byte leftTrigPin, byte leftEchoPin, byte centerTrigPin, byte centerEchoPin, byte rightTrigPin, byte rightEchoPin)
{
  pinMode(leftTrigPin, OUTPUT);
  pinMode(centerTrigPin, OUTPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(centerEchoPin, INPUT);
  pinMode(rightEchoPin, INPUT);
  digitalWrite(leftTrigPin, LOW);
  digitalWrite(centerTrigPin, LOW);
  digitalWrite(rightTrigPin, LOW);

  attachInterrupt(leftEchoPin,left_timer_change,CHANGE);
  attachInterrupt(centerEchoPin,center_timer_change,CHANGE);
  attachInterrupt(rightEchoPin,right_timer_change,CHANGE);
  _leftTrigPin = leftTrigPin;
  _leftEchoPin = leftEchoPin;
  _centerTrigPin = centerTrigPin;
  _centerEchoPin = centerEchoPin;
  _rightTrigPin = rightTrigPin;
  _rightEchoPin = rightEchoPin;
}

int UltrasonicTower::readLeftDistance()
{
  leftDistance = 0;
  digitalWrite(_leftTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_leftTrigPin, LOW);
  delayMicroseconds(200 * US_ROUNDTRIP_CM);
  return leftDistance;
}

int UltrasonicTower::readCenterDistance()
{
  centerDistance = 0;
  digitalWrite(_centerTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_centerTrigPin, LOW);
  delayMicroseconds(200 * US_ROUNDTRIP_CM);
  return centerDistance;
}

int UltrasonicTower::readRightDistance()
{
  rightDistance = 0;
  digitalWrite(_rightTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_rightTrigPin, LOW);
  delayMicroseconds(200 * US_ROUNDTRIP_CM);
  return rightDistance;
}

void UltrasonicTower::left_timer_change()
{
  leftEchoTime = micros() - leftStartTime;
  leftStartTime = micros();  
  leftDistance = timeDistanceConvert(leftEchoTime, US_ROUNDTRIP_CM);
}
void UltrasonicTower::center_timer_change()
{
  centerEchoTime = micros() - centerStartTime;
  centerStartTime = micros();  
  centerDistance = timeDistanceConvert(centerEchoTime, US_ROUNDTRIP_CM);
}
void UltrasonicTower::right_timer_change()
{
  rightEchoTime = micros() - rightStartTime;
  rightStartTime = micros();  
  rightDistance = timeDistanceConvert(rightEchoTime, US_ROUNDTRIP_CM);
}