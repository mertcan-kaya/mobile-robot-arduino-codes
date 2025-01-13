
/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo 
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * This code is beerware; if you see me (or any other SparkFun employee) at the
 * local, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ***************************************************************/

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Pose2D.h>
#include "CytronMotorDriver.h"
#include <math.h>
#include <Servo.h>

// Wheelbase Motion code

ros::NodeHandle nh;
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

// Define parameters for Dijkstra and Dynamic Window Approach
// float dist_threshold = 0.25; // threshold distance to goal
float dist_threshold = 0.01; // threshold distance to goal
float angle_threshold = 0.01; // threshold orientation to goal

float wheelDiameter = 0.05; // meter
float wheelRadius = wheelDiameter/2.0;
float interWheelDistance = 0.14; // meter
//scannerPoseWrtBob = bob_getScannerPose(connection);

// controller parameters
float Krho = 0.7;
float Kalpha = 2.0;
float Kbeta = -0.6;
// float Kalpha = 0.0;
// float Kbeta = -0.00;
bool backwardAllowed = false;
bool useConstantSpeed = false;
float constantSpeed = 4;
float constantSpeed2 = 20;

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

// float xg = 1.0;
// float yg = 0.5;
// float thetag = -PI/4;

float xg = 1.0;
float yg = 0.0;
float thetag = PI/2;

float rho = 0.0;
float lambda = 0.0;
float alpha = 0.0;

float beta_mot = 0.0;

float vu = 0.0;
float omega = 0.0;

float Phia = 0.0;
float Phib = 0.0;

float RightWheelVelocity = 0.0;
float LeftWheelVelocity = 0.0;

float dtheta = 0.0;

bool EndCond = false;
bool StartCond = false;

int left_pwm = 0;
int right_pwm = 0;

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

// Servo Code

int neck_len = 75;
int neck_yaw = 90;
int neck_pitch = 10;

void neck_len_cb(const std_msgs::UInt16& cmd_msg){
  neck_len = cmd_msg.data;
}

void neck_yaw_cb(const std_msgs::UInt16& cmd_msg){
  neck_yaw = cmd_msg.data;
}

void neck_pitch_cb(const std_msgs::UInt16& cmd_msg){
  neck_pitch = cmd_msg.data;
}

void pose2d_cb(const geometry_msgs::Pose2D& cmd_msg){
  xg = cmd_msg.x;
  yg = cmd_msg.y;
  thetag = cmd_msg.theta;
}
 
ros::Subscriber<std_msgs::UInt16> sub1("neck_len", neck_len_cb);
ros::Subscriber<std_msgs::UInt16> sub2("neck_yaw", neck_yaw_cb);
ros::Subscriber<std_msgs::UInt16> sub3("neck_pitch", neck_pitch_cb);
ros::Subscriber<geometry_msgs::Pose2D> sub4("travel_pose", pose2d_cb);

class Sweeper
{
  Servo servo;              // the servo
  int pos;              // current servo position 
  int increment;        // increment to move for each interval
  int updateInterval;      // interval between updates
  unsigned long lastUpdate; // last update of position

public: 
  Sweeper(int interval)
  {
    updateInterval = interval;
    increment = 1;
  }
  
  void Attach(int pin)
  {
    servo.attach(pin);
  }
  
  void Detach()
  {
    servo.detach();
  }
  
  void GoTo(int posDes)
  {
    if((millis() - lastUpdate) > updateInterval)  // time to update
    {
      lastUpdate = millis();
      if (pos < posDes)
        pos += increment;
      if (pos > posDes)
        pos -= increment;
      servo.write(pos);
      //Serial.println(pos);
    }
  }
  
  void MoveCheck(int posDes)
  {
    if(pos-posDes < 1 && pos-posDes > -1)
    {
      digitalWrite(LED_BUILTIN, LOW);  // Update the actual LED
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);   // Update the actual LED
    }
  }
  
  void Update()
  {
    if((millis() - lastUpdate) > updateInterval)  // time to update
    {
      lastUpdate = millis();
      pos += increment;
      servo.write(pos);
      if ((pos >= 1) || (pos <= 0)) // end of sweep
      {
        // reverse direction
        increment = -increment;
      }
    }
  }
};

Sweeper sweeper1(15);
Sweeper sweeper2(15);
Sweeper sweeper3(15);


void setup()
{
  nh.initNode();
  //nh.advertise(imu_pub);

// Wheelbase motion
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
  timeStp = 0.008;

  // Servo

  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  
  sweeper1.Attach(15);
  sweeper2.Attach(14);
  sweeper3.Attach(13);

  sweeper1.GoTo(80);
  sweeper2.GoTo(80);
  sweeper3.GoTo(10);
  
} 

void loop()
{

  // Servo

  sweeper1.GoTo(neck_len);
  sweeper2.GoTo(neck_yaw);
  sweeper3.GoTo(neck_pitch);
  
  sweeper1.MoveCheck(neck_len);
  sweeper2.MoveCheck(neck_yaw);
  sweeper3.MoveCheck(neck_pitch);

// Wheelbase
  if (StartCond == true) {
    if (EndCond == false) {

      // run control step
      rho = sqrt(pow((xg-x),2)+pow((yg-y),2));        // pythagoras theorem, sqrt(dx^2 + dy^2)
      lambda = atan2(yg-y, xg-x);           // angle of the vector pointing from the robot to the goal in the inertial frame
      alpha = normalizeAngle(lambda-theta); // angle of the vector pointing from the robot to the goal in the robot frame
      beta_mot = normalizeAngle(thetag-lambda);
      
      // lambda = normalizeAngle(atan2(yg-y, xg-x));           // angle of the vector pointing from the robot to the goal in the inertial frame
      // alpha = lambda-theta; // angle of the vector pointing from the robot to the goal in the robot frame
      // beta = thetag-lambda;

      if (backwardAllowed == true) {
          // If obstacle in "front" => go forward
          if (fabs(alpha) > PI/2) {
              alpha = normalizeAngle(lambda-theta-PI);
              beta_mot = normalizeAngle(thetag-lambda-PI);
              Krho = -Krho;
          }
      }

      vu = Krho*rho; // [m/s]
      if (rho > 1e-6) {
        omega = Kalpha*alpha + Kbeta*beta_mot; // [rad/s]
      } else {
        omega = 6*Kalpha*normalizeAngle(thetag-theta); // [rad/s]
      }

      if (useConstantSpeed == true) {
          float absVel = fabs(vu);
          if (absVel > 10) {
              vu = constantSpeed*vu/absVel;
          }
          if (fabs(omega) > 0.1) {
              omega = constantSpeed2*omega/fabs(omega);
          }
      }

      Phia = vu/wheelRadius;
      Phib = omega*interWheelDistance/wheelDiameter;

      RightWheelVelocity = 0.25*(Phia + Phib);
      LeftWheelVelocity = 0.25*(Phia - Phib);

      // End condition
      dtheta = fabs(normalizeAngle(theta-thetag));
      EndCond = (rho < dist_threshold && dtheta < angle_threshold) || rho > 5;
      
      xR_dot = wheelRadius*(RightWheelVelocity+LeftWheelVelocity)/2;
      theta_dot = wheelRadius*(RightWheelVelocity-LeftWheelVelocity)/(2*interWheelDistance);

      // xR = xR + xR_dot*timeStp;
      theta = theta + theta_dot*timeStp;

      x_dot = cos(theta)*xR_dot;
      y_dot = sin(theta)*xR_dot;

      x = x + x_dot*timeStp;
      y = y + y_dot*timeStp;

    }
    else {
      RightWheelVelocity = 0.0;
      LeftWheelVelocity = 0.0;
    }
  } else {
    LeftWheelVelocity = 20.0;
    RightWheelVelocity = 20.0;
  }

  left_pwm = (int)(60*LeftWheelVelocity);
  right_pwm = -(int)(60*RightWheelVelocity);

  if (right_pwm < -254)
    right_pwm = -254;
  if (right_pwm > 254)
    right_pwm = 254;

  if (left_pwm < -254)
    left_pwm = -254;
  if (left_pwm > 254)
    left_pwm = 254;

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

  if (fabs(wheelLVel)+fabs(wheelRVel) > 1e-6)
    StartCond = true;

  wheelLPosPrv = wheelLPos;
  wheelRPosPrv = wheelRPos;

  motor1.setSpeed(right_pwm);
  motor2.setSpeed(left_pwm);


  

}
