#include "CytronMotorDriver.h"

long left_speed;
long right_speed;

// define encoder pins
const int encoder0PinA = 18; // interrupt pin 5 on MEGA
const int encoder0PinB = 19; // interrupt pin 4 on MEGA

const int encoder1PinA = 20; // interrupt pin 3 on MEGA
const int encoder1PinB = 21; // interrupt pin 2 on MEGA

// encoder value change motor turn angles
const float ratio = 2*PI/32800.0;
// 360. -> 1 turn (in deg)
// 2*PI -> 1 turn (in rad)
// 3280 -> Gear and Encoder Ratio

// declare and initialize global encoder position variables
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 1, 0); // PWM 1 = Pin 1, DIR 1 = Pin 0.
CytronMD motor2(PWM_DIR, 5, 4); // PWM 2 = Pin 5, DIR 2 = Pin 4.

float target = 1.0;

// // Define parameters for Dijkstra and Dynamic Window Approach
// float dist_threshold = 0.25; // threshold distance to goal
// float angle_threshold = 0.1; // threshold orientation to goal

// float wheelDiameter = 0.05; // meter
// float wheelRadius = wheelDiameter/2.0;
// float interWheelDistance = 0.14; // meter
// //scannerPoseWrtBob = bob_getScannerPose(connection);

// // controller parameters
// float Krho = 0.5;
// float Kalpha = 1.5;
// float Kbeta = -0.6;
// bool backwardAllowed = false;
// bool useConstantSpeed = true;
// float constantSpeed = 0.4;

int state = 0;
int state3 = 0;

// unsigned long timePrv = 0;

// float wheelRPosPrv = 0.0;
// float wheelLPosPrv = 0.0;

void doEncoder0A() {
  encoder0Pos += (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))?1:-1;
}

void doEncoder0B() {
  encoder0Pos -= (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))?1:-1;
}

void doEncoder1A() {
  encoder1Pos += (digitalRead(encoder1PinA) == digitalRead(encoder1PinB))?1:-1;
}

void doEncoder1B() {
  encoder1Pos -= (digitalRead(encoder1PinA) == digitalRead(encoder1PinB))?1:-1;
}

void setup() 
{ 
  
  Serial.begin(115200);
  
  // begining of encoder code
  
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);

  // encoder pin on interrupt #5 (pin 18)
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);

  // encoder pin on interrupt #4 (pin 19)
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);
  
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  // encoder pin on interrupt #3 (pin 20)
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);

  // encoder pin on interrupt #2 (pin 21)
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);

  // end of encoder code
  
} 

void loop() 
{ 
  // unsigned long timeNow = millis();
  // unsigned long timeDlt = timeNow - timePrv;
  // float         timeStp = timeDlt*0.001;

  float wheelLPos = -(float)encoder0Pos*ratio;
  float wheelRPos = (float)encoder1Pos*ratio;

  // float wheelLVel = (wheelLPos-wheelLPosPrv)/timeStp;
  // float wheelRVel = (wheelRPos-wheelRPosPrv)/timeStp;

  if ((wheelLPos+wheelRPos)/2.0 < target)
    state = 0;
  else
    state = 1;

  if (state == 0)
  {
    left_speed = 50;
    right_speed = 50;
  }
  else
  {
    left_speed = 0;
    right_speed = 0;
  }
  
  motor1.setSpeed(-right_speed);
  motor2.setSpeed(left_speed);

  Serial.print("wheelLPos: ");
  Serial.print(wheelLPos, 6);
  Serial.print(", ");
  Serial.print("wheelRPos: ");
  Serial.print(wheelRPos, 6);
  Serial.print(", ");
  Serial.print("state: ");
  Serial.print(state);
  Serial.println();
}
