#include "PID.h"
#include <Servo.h>

PID::PID(float desired_position, float Kp, float Ki, float Kd) {
    desiredPos = desired_position;
    proportionalK = Kp;
    integralK = Ki;
    differentialK = Kd;
    pwmFreq = 350;
    pwmCap= 0.05;  //CAPING PWM 

    integralError = 0.0;
    differentialError = 0.0;
    prevError = 0.0;
    error = 0.0;
}

void PID::controlLoop(float sensorValue, bool printer, uint8_t pwmPin, Servo esc) {
    
    
    // --P--
    error = desiredPos - sensorValue;
    if (error > 180.0) error -= 360.0;          //adjust the error to fit within 1 rotation
    else if (error < -180.0) error += 360.0;
    float proportionalOutput = proportionalK * error;

    // --I--
    integralError += error;
    integralError = constrain(integralError, -500.0, 500.0); // Anti-windup
    float integralOutput = integralK * integralError;

    // --D--
    differentialError = error - prevError;
    float differentialOutput = differentialK * differentialError;
    prevError = error;

    // Output Calculation
    float output = proportionalOutput + integralOutput + differentialOutput;

    //Cap PWM
    int pwmMax = 255 * pwmCap;
    float outputCapped = output * pwmCap;

    // Constrain PWM value
    int pwmValue = constrain(abs(outputCapped), 0, pwmMax);
    int throttle = map(pwmValue,0,255,1120,2000);
    esc.writeMicroseconds(throttle);

    // Print debug info
    if (printer) {
        Serial.print("Setpoint: "); Serial.println(desiredPos);
        Serial.print("Total Output: "); Serial.println(output);
        Serial.print("Proportional Output: "); Serial.println(proportionalOutput);
        Serial.print("Integral Output: "); Serial.println(integralOutput);
        Serial.print("Differential Output: "); Serial.println(differentialOutput);
        Serial.print("Position: "); Serial.println(sensorValue);
        Serial.print("PWM Throttle: "); Serial.println(throttle);
        Serial.println();
    }
    
}


void PID::setDesiredPosition(float pos) {
    // Constrain position 
    if (pos < 1.0) pos = 1.0;
    if (pos > 360.0) pos = 360.0;
    desiredPos = pos;
}

float PID::angleDifference(float desiredPos, float currentPos){
    float diff = desiredPos-currentPos;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return fabs(diff);
}

bool PID::isEqual(float desiredPos, float currentPos, float approximation){
    return this->angleDifference(desiredPos,currentPos) <= approximation;
}