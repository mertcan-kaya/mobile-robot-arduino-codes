#include "CytronMotorDriver.h"
#include <math.h>

long left_speed;
long right_speed;

// define encoder pins
const int encoder0PinA = 18; // interrupt pin 5 on MEGA
const int encoder0PinB = 19; // interrupt pin 4 on MEGA

const int encoder1PinA = 20; // interrupt pin 3 on MEGA
const int encoder1PinB = 21; // interrupt pin 2 on MEGA

unsigned long timeNow = 0;
unsigned long timeDlt = 0;
float         timeStp = 0.0;
unsigned long timePrv = 0;

// encoder value change motor turn angles
const float ratio = 2*PI/3280.0;
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

// Define parameters for Dijkstra and Dynamic Window Approach
// float dist_threshold = 0.25; // threshold distance to goal
float dist_threshold = 0.05; // threshold distance to goal
float angle_threshold = 0.1; // threshold orientation to goal

float wheelDiameter = 0.05; // meter
float wheelRadius = wheelDiameter/2.0;
float interWheelDistance = 0.14; // meter
//scannerPoseWrtBob = bob_getScannerPose(connection);

// controller parameters
float Krho = 7;
float Kalpha = 5;
float Kbeta = -0.6;
// float Kalpha = 0.0;
// float Kbeta = -0.00;
bool backwardAllowed = false;
bool useConstantSpeed = false;
float constantSpeed = 4;
float constantSpeed2 = 20;

int state = 0;
int state3 = 0;

float wheelLPos = 0.0;
float wheelRPos = 0.0;

float wheelLVel = 0.0;
float wheelRVel = 0.0;

float wheelRPosPrv = 0.0;
float wheelLPosPrv = 0.0;

float x = 0.0;
float y = 0.0;
float theta = 0.0;

float xR_dot = 0.0;
float theta_dot = 0.0;

float xR = 0.0; // delete later

float x_dot = 0.0;
float y_dot = 0.0;

float xg = 1.0;
float yg = 0.0;
float thetag = 0.0;

// float xg = 1.0;
// float yg = 0.0;
// float thetag = PI/2;

float rho = 0.0;
float lambda = 0.0;
float alpha = 0.0;

float beta = 0.0;

float vu = 0.0;
float omega = 0.0;

float Phia = 0.0;
float Phib = 0.0;

float RightWheelVelocity = 0.0;
float LeftWheelVelocity = 0.0;

float dtheta = 0.0;

bool EndCond = false;    

float normalizeAngle(float angle) {  
  // normalizeAngle   set angle to the range [-pi,pi)
  return fmod((angle+PI),(2*PI)) - PI;
}

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
  
  timePrv = millis();
} 

void loop() 
{ 
  timeNow = millis();
  timeDlt = timeNow - timePrv;
  if (timeDlt > 0)
    timeStp = (float)timeDlt*0.001;
  else
    timeStp = 0.008;
  timePrv = timeNow;

  wheelLPos = -(float)encoder0Pos*ratio;
  wheelRPos = (float)encoder1Pos*ratio;

  wheelLVel = (wheelLPos-wheelLPosPrv)/timeStp;
  wheelRVel = (wheelRPos-wheelRPosPrv)/timeStp;

  wheelLPosPrv = wheelLPos;
  wheelRPosPrv = wheelRPos;

  if (EndCond == false) {
    xR_dot = wheelRadius*(wheelRVel+wheelLVel)/2;
    theta_dot = wheelRadius*(wheelLVel-wheelRVel)/(2*interWheelDistance);

    xR = xR + xR_dot*timeStp;
    theta = theta + theta_dot*timeStp;

    x_dot = cos(theta)*xR_dot;
    y_dot = sin(theta)*xR_dot;

    x = x + x_dot*timeStp;
    y = y + y_dot*timeStp;

    // run control step
    //[ vu, omega ] = calculateControlOutput([x, y, theta], [xg, yg, thetag], parameters);

    rho = sqrt(pow((xg-x),2)+pow((yg-y),2));        // pythagoras theorem, sqrt(dx^2 + dy^2)
    // lambda = atan2(yg-y, xg-x);           // angle of the vector pointing from the robot to the goal in the inertial frame
    // alpha = normalizeAngle(lambda-theta); // angle of the vector pointing from the robot to the goal in the robot frame

    // beta = normalizeAngle(thetag-lambda);
    
    lambda = normalizeAngle(atan2(yg-y, xg-x));           // angle of the vector pointing from the robot to the goal in the inertial frame
    alpha = lambda-theta; // angle of the vector pointing from the robot to the goal in the robot frame
    beta = thetag-lambda;

    // if (backwardAllowed == true) {
    //     // If obstacle in "front" => go forward
    //     if (abs(alpha) > PI/2) {
    //         alpha = normalizeAngle(lambda-theta-PI);
    //         beta = thetag-lambda-PI;
    //         Krho = -Krho;
    //     }
    // }

    vu = Krho*rho; // [m/s]
    omega = Kalpha*alpha + Kbeta*beta; // [rad/s]

    // if (useConstantSpeed == true) {
    //     float absVel = abs(vu);
    //     if (absVel > 1e-6) {
    //         vu = constantSpeed*vu/absVel;
    //         omega = constantSpeed*omega/absVel;
    //     }
    //     else if (abs(omega) > 1e-6) {
    //         vu = constantSpeed*vu/abs(omega);
    //         omega = constantSpeed*omega/abs(omega);
    //     }
    // }

    if (useConstantSpeed == true) {
        float absVel = abs(vu);
        if (absVel > 10) {
            vu = constantSpeed*vu/absVel;
        }
        if (abs(omega) > 0.1) {
            omega = constantSpeed2*omega/abs(omega);
        }
    }

  //  float halfWheelbase = interWheelDistance/2;

  //   float a = wheelRadius/2;

  // M = [a                a;
  //      a*(1/halfWheelbase) a*(-1/halfWheelbase)];

  //   detA = halfWheelbase/(-2*a*a);

  // Minv = [1/(2*a) halfWheelbase/(2*a);
  //         1/(2*a) -halfWheelbase/(2*a)];

  // Minv = [1/wheelRadius halfWheelbase/wheelRadius;
  //         1/wheelRadius -halfWheelbase/wheelRadius];

  // PhiPrime = Minv * [vu; omega];
  // LeftWheelVelocity = PhiPrime(2);
  // RightWheelVelocity = PhiPrime(1);

    Phia = vu/wheelRadius;
    Phib = omega*interWheelDistance/wheelDiameter;

    RightWheelVelocity = (Phia + Phib);
    LeftWheelVelocity = (Phia - Phib);

    // RightWheelVelocity = Phia*0.97;
    // LeftWheelVelocity = Phia;

    if (RightWheelVelocity < -254)
      RightWheelVelocity = -254;
    if (RightWheelVelocity > 254)
      RightWheelVelocity = 254;

    if (LeftWheelVelocity < -254)
      LeftWheelVelocity = -254;
    if (LeftWheelVelocity > 254)
      LeftWheelVelocity = 254;

    // End condition
    dtheta = abs(normalizeAngle(theta-thetag));

    EndCond = (rho < dist_threshold && dtheta < angle_threshold) || rho > 5;
    
    // EndCond = (xg-xR < dist_threshold && dtheta < angle_threshold);   
  }
  else {
    RightWheelVelocity = 0.0;
    LeftWheelVelocity = 0.0;
  }

  motor1.setSpeed(-(int)RightWheelVelocity);
  motor2.setSpeed((int)LeftWheelVelocity);

  Serial.print("xR: ");
  Serial.print(xR, 6);
  Serial.print(", ");
  Serial.print("x: ");
  Serial.print(x, 6);
  // Serial.println();
  Serial.print(", ");
  Serial.print("yg: ");
  Serial.print(yg, 6);
  Serial.print(", ");
  Serial.print("y: ");
  Serial.print(y, 6);
  // // Serial.println();
  Serial.print(", ");
  // Serial.print("thetag: ");
  // Serial.print(thetag, 6);
  // Serial.print(", ");
  Serial.print("theta: ");
  Serial.print(theta, 6);
  Serial.print(", ");
  Serial.print("dtheta: ");
  Serial.print(dtheta, 6);
  Serial.print(", ");
  
  Serial.print("vu: ");
  Serial.print(vu, 6);
  Serial.print(", ");
  Serial.print("omega: ");
  Serial.print(omega, 6);
  Serial.print(", ");

  Serial.print("wheelLPos: ");
  Serial.print(wheelLPos, 6);
  Serial.print(", ");
  Serial.print("wheelRPos: ");
  Serial.print(wheelRPos, 6);
  Serial.print(", ");
  Serial.print("wheelLVel: ");
  Serial.print(wheelLVel, 6);
  Serial.print(", ");
  Serial.print("wheelRVel: ");
  Serial.print(wheelRVel, 6);
  Serial.print(", ");
  Serial.print("wLvel: ");
  Serial.print(LeftWheelVelocity, 6);
  Serial.print(", ");
  Serial.print("wRvel: ");
  Serial.print(RightWheelVelocity, 6);
  
  // Serial.print(", ");
  // Serial.print("lambda: ");
  // Serial.print(lambda, 6);
  // Serial.print(", ");
  // Serial.print("alpha: ");
  // Serial.print(alpha, 6);
  // Serial.print(", ");
  // Serial.print("beta: ");
  // Serial.print(beta, 6);

  Serial.println();
  
  // Serial.print("dtheta: ");
  // Serial.print(dtheta, 6);
  // Serial.println();
  // Serial.print("timeStp: ");
  // Serial.print(timeStp, 6);
  // Serial.print(", ");
  // Serial.print("vu: ");
  // Serial.print(vu, 6);
  // Serial.print(", ");
  // Serial.print("omega: ");
  // Serial.print(omega, 6);
  // Serial.println();
  // Serial.print("rho: ");
  // Serial.print(rho, 6);
  // Serial.print(", ");
  // Serial.print("lambda: ");
  // Serial.print(lambda, 6);
  // Serial.println();
  // Serial.print("alpha: ");
  // Serial.print(alpha, 6);
  // Serial.print(", ");
  // Serial.print("beta: ");
  // Serial.print(beta, 6);
  // Serial.println();
  // Serial.println();

}
