#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_TMAG5273_Arduino_Library.h>
#include "PID.h"

Servo esc;
const int escPIN = 9;
TMAG5273 mySensor;
float dp = 0;
float Kp = 5;
float Kd = 0.3;
float Ki = 0.1;
PID myPID(dp,Kp,Ki,Kd); //ADJUST GAINS IF NEEDED

const uint8_t pwmPin = 9;
float desiredPos = 0;
float currentPos = 0;
bool stopper = 0;

void setup() {
  //Arming sequence
  esc.attach(escPIN); ;
  esc.writeMicroseconds(1000);
  delay(3000);

  Serial.begin(115200);
  while (!Serial);

  Wire.begin();

  if (!mySensor.begin(0x22, Wire)) {
    Serial.println("TMAG5273 not detected.");
    while (1);
  }

  mySensor.setAngleEn(true);
  Serial.println("TMAG5273 initialized for angle measurement.");

  Serial.println("Please enter desired angle (1-360) with respect to current position and press Enter:");

  while (Serial.available() == 0) {
    // Wait here until user input arrives
    delay(10);
  }
  float currentPos = mySensor.getAngleResult(); // Reads angle in degrees (0° to 360°)
  desiredPos = currentPos + Serial.parseFloat();

    if (desiredPos > 360.0) desiredPos -= 360.0;          //adjust the desired position to fit within 1 rotation

  myPID.setDesiredPosition(desiredPos);
}

void loop() {
    
   // while(stopper == 0){
    float currentPos = mySensor.getAngleResult(); // Reads angle in degrees (0° to 360°)
    Serial.println("VALUE:");
    Serial.print(currentPos);

    if(!myPID.isEqual(desiredPos,currentPos,2.0)){
    myPID.controlLoop(currentPos, true, pwmPin,esc);
    delay(100);
    }else{
        stopper = 1;
    }
    //}
}
