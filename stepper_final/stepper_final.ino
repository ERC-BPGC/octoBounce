#include <math.h>
#define motorInterfaceType 1

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
float STEPS = stepsPerRevolution * microsteps;
float step_angle = (2*3.14) / STEPS;
//need to change as per need
double a1 = 7.928;
double a2 = 10.0;
double plength = 40.48;


volatile double stepper1_height = 0.0;
volatile double stepper2_height = 0.0;
volatile double stepper3_height = 0.0;
volatile double stepper4_height = 0.0;

void calculate_height(double angle_x, double angle_y) {
  
 
    double h = 12.5;
    stepper1_height = h + (plength/2) * sin(angle_x);
    stepper2_height = h - (plength/2) * sin(angle_x);
    stepper3_height = h + (plength/2) * sin(angle_y);
    stepper4_height = h - (plength/2) * sin(angle_y);
}

float invkin(double y){
    double q2 = acos((pow(y,2)+ pow(8.98,2)-pow(a1,2)-pow(a2,2))/(2*a1*a2));
    double stepper_angle = atan(y/8.98) - (atan(a2*sin(q2)/(a1+a2*cos(q2))));
    Serial.println(stepper_angle);
    delay(100);
    return abs(stepper_angle);
}

int stepCount = 0;

void setup(){
  Serial.begin(9600);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT);
    // Set motor direction clockwise
  digitalWrite(dirPin1, LOW);
    // Set motor direction clockwise
  digitalWrite(dirPin2, LOW);
  digitalWrite(dirPin3, HIGH);
  digitalWrite(dirPin4, HIGH);



    double angle_x = (7*3.14)/180;
    double angle_y = (7*3.14)/180;
    calculate_height(angle_x, angle_y);

    Serial.println(angle_x);
    Serial.println(angle_y);
    
    double stepper1_angle = invkin(stepper1_height)+ 0.08726;
    double stepper2_angle = invkin(stepper2_height)+ 0.08726;
    double stepper3_angle = invkin(stepper3_height)+  0.08726;
    double stepper4_angle = invkin(stepper4_height)+ 0.08726;


    int step1 = stepper1_angle/step_angle;
    int step3 = stepper2_angle/step_angle;
    int step2 = stepper3_angle/step_angle;
    int step4 = stepper4_angle/step_angle;



    
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
  
}

void loop(){
      
  
  }
