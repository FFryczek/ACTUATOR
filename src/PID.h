#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class PID {
  private:
    float desiredPos;
    float proportionalK;
    float integralK;
    float differentialK;
    int pwmFreq;
    float pwmCap;

    float integralError;
    float differentialError;
    float prevError;
    float error;

  public:
    PID(float desired_position, float Kp, float Ki, float Kd);

    void controlLoop(float sensorValue, bool printer, uint8_t pwmPin,Servo esc);
    void setDesiredPosition(float pos);
    float angleDifference(float desiredPos, float currentPos);
    bool isEqual(float desiredPos, float currentPos, float approximation);
};

#endif