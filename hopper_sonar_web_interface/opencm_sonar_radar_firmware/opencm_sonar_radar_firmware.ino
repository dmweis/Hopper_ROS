#define leftSensorTrigPin 7
#define leftSensorEchoPin 6

#define centerSensorTrigPin 5
#define centerSensorEchoPin 4

#define rightSensorTrigPin 3
#define rightSensorEchoPin 2

#define maximumSensorRange 200
#define minimumSensorRange 0

#define DXL_BUS_Serial3 1
#define ID_NUM 1

#include "UltrasonicTower.h"

UltrasonicTower sensorTower(leftSensorTrigPin, leftSensorEchoPin, centerSensorTrigPin, centerSensorEchoPin, rightSensorTrigPin, rightSensorEchoPin);

Dynamixel Dxl(DXL_BUS_Serial3);

void setup()
{
  Serial3.begin(9600);
  Dxl.begin(3);
  Dxl.jointMode(ID_NUM);
  Dxl.goalSpeed(ID_NUM, 80);
  moveServoTo(0);
}

void loop()
{
  moveServoTo(60);
  while (Dxl.isMoving(ID_NUM))
  {
    measureAllSensors("CW");
  }
  moveServoTo(-60);
  while (Dxl.isMoving(ID_NUM))
  {
    measureAllSensors("CCW");
  }
}

int measureAllSensors(char* direction)
{
  int leftDistance = sensorTower.readLeftDistance();
  int centerDistance = sensorTower.readCenterDistance();
  int rightDistance = sensorTower.readRightDistance();
  int currentServoAngle = getServoPosition();
  sendSensorData(direction, currentServoAngle, leftDistance, centerDistance, rightDistance);
}

void sendSensorData(char* direction, int motorAngle, int leftDistance, int centerDistance, int rightDistance)
{
  Serial3.print("{\"direction\":\"");
  Serial3.print(direction);
  Serial3.print("\", \"angle\": ");
  if (motorAngle < 0)
  {
    motorAngle = 360 + motorAngle;
  }
  Serial3.print(motorAngle);
  if (leftDistance >= maximumSensorRange || leftDistance <= minimumSensorRange)
  {
    Serial3.print(", \"leftSensor\":null");
  }
  else
  {
    Serial3.print(", \"leftSensor\":");
    Serial3.print(leftDistance);
  }
  if (centerDistance >= maximumSensorRange || centerDistance <= minimumSensorRange)
  {
    Serial3.print(", \"centerSensor\":null");
  }
  else
  {
    Serial3.print(", \"centerSensor\":");
    Serial3.print(centerDistance);
  }
  if (rightDistance >= maximumSensorRange || rightDistance <= minimumSensorRange)
  {
    Serial3.print(", \"rightSensor\":null");
  }
  else
  {
    Serial3.print(", \"rightSensor\":");
    Serial3.print(rightDistance);
  }
  Serial3.println("}");
}

int getServoPosition()
{
  int position = Dxl.getPosition(ID_NUM);
  return map(position, 1023, 0, -150, 150);
}

void moveServoTo(int angle)
{
  Dxl.goalPosition(ID_NUM, map(angle, -150, 150, 1023, 0));
}
