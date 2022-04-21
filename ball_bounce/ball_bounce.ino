#include <Stepper.h>
#include <math.h>

const int stepsPerRevolution = 200;  
const int microsteps = 32;
   = stepsPerRevolution * microsteps;
step_angle = 360 / STEPS;
//need to change as per need
a1 = 1;
a2 = 1;
plength = 1;

Stepper stepper1(STEPS, 1, 2, 3, 4);
Stepper stepper2(STEPS, 5, 6, 7, 8);
Stepper stepper3(STEPS, 9, 10, 11, 12);
Stepper stepper4(STEPS, 8, 9, 10, 11);


float calculate_height(float angle_x, float angle_y, float h, float* stepper_height) {
  float stepper1_height = h + (plength/2) * sin(angle_x);
  float stepper2_height = h + (plength/2) * sin(angle_y);
  float stepper3_height = h - (plength/2) * sin(angle_x);
  float stepper4_height = h - (plength/2) * sin(angle_y);

  stepper_height[4] = {stepper1_height, stepper2_height, stepper3_height, stepper4_height};
  return stepper_height;
}

float invkin(float y){
  float q2 = acos((pow(y,2)-pow(a1,2)-pow(a2,2))/(2*a1*a2));
  float stepper_angle = 90 - (atan(a2*sin(q2)/(a1+a2*cos(q2))));
  return stepper_angle;
}

int stepCount = 0;         

void setup() {
  Serial.begin(9600);
  stepper1.setSpeed(100);
  stepper2.setSpeed(100);
  stepper3.setSpeed(100);
  stepper4.setSpeed(100);
}

void loop() {
  float angle_x = 10;
  float angle_y = 10;
  float h = 5;
  float stepper_height[4] = calculate_height(angle_x, angle_y, h);
  float stepper1_angle = invkin(stepper_height[0]);
  float stepper2_angle = invkin(stepper_height[1]);
  float stepper3_angle = invkin(stepper_height[2]);
  float stepper4_angle = invkin(stepper_height[3]);

  int step1 = stepper1_angle/step_angle;
  int step2 = stepper2_angle/step_angle;
  int step3 = stepper3_angle/step_angle;
  int step4 = stepper4_angle/step_angle;

  stepper1.step(step1);
  stepper2.step(step2);
  stepper3.step(step3);
  stepper4.step(step4);

}