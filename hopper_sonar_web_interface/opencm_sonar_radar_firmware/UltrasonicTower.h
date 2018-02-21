#define US_ROUNDTRIP_CM 57
#define timeDistanceConvert(echoTime, conversionFactor) (max((echoTime + conversionFactor / 2) / conversionFactor, (echoTime ? 1 : 0)))

#ifndef UltrasonicTower_h
#define UltrasonicTower_h

#include <wirish.h>

volatile static long leftEchoTime;
volatile static long leftStartTime;
volatile static int leftDistance;

volatile static long centerEchoTime;
volatile static long centerStartTime;
volatile static int centerDistance;

volatile static long rightEchoTime;
volatile static long rightStartTime;
volatile static int rightDistance;


class UltrasonicTower
{
    public:
        UltrasonicTower(byte leftTrigPin, byte leftEchoPin, byte centerTrigPin, byte centerEchoPin, byte rightTrigPin, byte rightEchoPin);
        int readLeftDistance();
        int readCenterDistance();
        int readRightDistance();
    private:
        static void left_timer_change();
        static void center_timer_change();
        static void right_timer_change();
        byte _leftTrigPin;
        byte _leftEchoPin;
        byte _centerTrigPin;
        byte _centerEchoPin;
        byte _rightTrigPin;
        byte _rightEchoPin;
       
};

#endif