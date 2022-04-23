#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
//
ros::NodeHandle nh;

#define STEP1_DIR LOW
#define STEP2_DIR LOW
#define STEP3_DIR HIGH
#define STEP4_DIR HIGH

const int dirPin1 = 8;
const int stepPin1 = 9;
const int dirPin2 = 4;
const int stepPin2 = 5;
const int dirPin3 = 2;
const int stepPin3 = 3;
const int dirPin4 = 7;
const int stepPin4 = 6;


const int stepsPerRevolution = 200;  
const int microsteps = 16;
const double STEPS = stepsPerRevolution * microsteps;
const double step_angle = (2*3.14) / STEPS;
//need to change as per need
double a1 = 7.928;
double a2 = 10.0;
double plength = 40.48;


volatile double stepper1_height = 0.0;
volatile double stepper2_height = 0.0;
volatile double stepper3_height = 0.0;
volatile double stepper4_height = 0.0;
volatile double x = 0.0;
volatile double y = 0.0;
volatile double z = 0.0;
volatile double vel = 0.0;
volatile double e = 1;


void calculate_height(double angle_x, double angle_y) {
  
 
    double h = 12.5;
    stepper1_height = h + (plength/2) * sin(angle_x);
    stepper2_height = h + (plength/2) * sin(angle_y);
    stepper3_height = h - (plength/2) * sin(angle_x);
    stepper4_height = h - (plength/2) * sin(angle_y);
}

float invkin(double y){
    double q2 = acos((pow(y,2)+ pow(8.98,2)-pow(a1,2)-pow(a2,2))/(2*a1*a2));
    double stepper_angle = atan(y/8.98) - (atan(a2*sin(q2)/(a1+a2*cos(q2))));
    return stepper_angle;
}

void messageCb(const geometry_msgs::Twist& msg)
{
  x = msg.linear.x;
  y = msg.linear.y;
  z = msg.linear.z;
  vel = msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("ballTracker", &messageCb);

geometry_msgs::Twist stepperData;
ros::Publisher pub("steps", &stepperData);

double previous_thetax = 0.0;
double previous_thetay = 0.0;

void setup(){

  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT); 
 
}

void loop(){

  
  double thetax = asin((x*0.098)/(e *(1+e)*vel*vel))/2;
  double thetay = asin((y*0.098)/(e *(1+e)*vel*vel))/2;

  Serial.println(thetax);
  Serial.println(thetay);
  thetax = thetax - previous_thetax;
  thetay = thetay - previous_thetay;

  thetax = (thetax*3.14)/180;
  thetax = (thetay*3.14)/180;
  calculate_height(thetax,thetay);

  
  double stepper1_angle = invkin(stepper1_height)+ 0.08726;
  double stepper2_angle = invkin(stepper2_height)+ 0.08726;
  double stepper3_angle = invkin(stepper3_height)+  0.08726;
  double stepper4_angle = invkin(stepper4_height)+ 0.08726;


  int step1 = stepper1_angle/step_angle;
  int step2 = stepper2_angle/step_angle;
  int step3 = stepper3_angle/step_angle;
  int step4 = stepper4_angle/step_angle;

  Serial.println(step1) ;
  delay(100);
  Serial.println(step2) ;

  if(step1 < 0){
      digitalWrite(dirPin1, !STEP1_DIR);
      step1 = - step1;
  }
  else{
    digitalWrite(dirPin1, STEP1_DIR);
  }

  if(step2 < 0){
      digitalWrite(dirPin2, !STEP2_DIR);
      step2 = - step2;
  }
  else{
    digitalWrite(dirPin2, STEP2_DIR);
  }
  if(step3 < 0){
      digitalWrite(dirPin3, !STEP3_DIR);
      step3 = - step3;
  }
  else{
    digitalWrite(dirPin3, STEP3_DIR);
  }
  if(step4 < 0){
      digitalWrite(dirPin4, !STEP4_DIR);
      step4 = - step4;
  }
  else{
    digitalWrite(dirPin4, STEP4_DIR);
  }
  
  for(int x = 0; x <= step1; x++)
  {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin1, LOW);
  }

  // Spin motor slowly
  for(int x = 0; x <= step2; x++)
  {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPin2, LOW);
  }

  // Spin motor slowly
  for(int x = 0; x <= step3; x++)
  {
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPin3, LOW);
  }
  // Spin motor slowly
  for(int x = 0; x <= step4; x++)
  {
    digitalWrite(stepPin4, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPin4, LOW);
  } 
  previous_thetax = thetax;
  previous_thetay = thetay;  
      
  stepperData.linear.x = x;
  stepperData.linear.y = y;
  stepperData.linear.z = z;
  stepperData.angular.z = vel;
  
  pub.publish( &stepperData );
  nh.spinOnce();
  delay(200);


}
