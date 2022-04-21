#include <AccelStepper.h>
#include <math.h>
#define motorInterfaceType 1


const int stepsPerRevolution = 200;  
const int microsteps = 32;
int STEPS = stepsPerRevolution * microsteps;
float step_angle = 360 / STEPS;
//need to change as per need
int a1 = 1;
int a2 = 1;
float plength = 1;

AccelStepper stepperx1 = AccelStepper(motorInterfaceType, 9,8);
AccelStepper stepperx2 = AccelStepper(motorInterfaceType, 5,4);
AccelStepper steppery2 = AccelStepper(motorInterfaceType, 3,2);
AccelStepper steppery1 = AccelStepper(motorInterfaceType, 6,7);

volatile float stepper1_height = 0.0;
volatile float stepper2_height = 0.0;
volatile float stepper3_height = 0.0;
volatile float stepper4_height = 0.0;

void calculate_height(float angle_x, float angle_y, float h) {
 
    stepper1_height = h + (plength/2) * sin(angle_x);
    stepper2_height = h + (plength/2) * sin(angle_y);
    stepper3_height = h - (plength/2) * sin(angle_x);
    stepper4_height = h - (plength/2) * sin(angle_y);
}

float invkin(float y){
    float q2 = acos((pow(y,2)-pow(a1,2)-pow(a2,2))/(2*a1*a2));
    float stepper_angle = 90 - (atan(a2*sin(q2)/(a1+a2*cos(q2))));
    return stepper_angle;
}

int stepCount = 0;

void setup(){
    stepperx1.setMaxSpeed(1000);
    stepperx1.setAcceleration(500);

    steppery1.setMaxSpeed(1000);
    steppery1.setAcceleration(500);

    stepperx2.setMaxSpeed(1000);
    stepperx2.setAcceleration(500);

    steppery2.setMaxSpeed(1000);
    steppery2.setAcceleration(500);

}

void loop(){
    float angle_x = 10;
    float angle_y = 10;
    float h = 5;
    calculate_height(angle_x, angle_y, h);
    float stepper1_angle = invkin(stepper1_height);
    float stepper2_angle = invkin(stepper2_height);
    float stepper3_angle = invkin(stepper3_height);
    float stepper4_angle = invkin(stepper4_height);

    int step1 = stepper1_angle/step_angle;
    int step2 = stepper2_angle/step_angle;
    int step3 = stepper3_angle/step_angle;
    int step4 = stepper4_angle/step_angle;

    stepperx1.moveTo(step1);
    stepperx2.moveTo(step2);
    steppery1.moveTo(step3);
    steppery2.moveTo(step4);

    stepperx1.runToPosition();
    steppery1.runToPosition();
    stepperx2.runToPosition();
    steppery2.runToPosition();

}
