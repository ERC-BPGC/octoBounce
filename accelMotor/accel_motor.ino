#include <AccelStepper.h>
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepperx1 = AccelStepper(motorInterfaceType, 9,8);
AccelStepper stepperx2 = AccelStepper(motorInterfaceType, 5,4);
AccelStepper steppery2 = AccelStepper(motorInterfaceType, 3,2);
AccelStepper steppery1 = AccelStepper(motorInterfaceType, 6,7);

void setup() {
    stepperx1.setMaxSpeed(1000);
    stepperx1.setAcceleration(500);

    steppery1.setMaxSpeed(1000);
    steppery1.setAcceleration(500);

    stepperx2.setMaxSpeed(1000);
    stepperx2.setAcceleration(500);

    steppery2.setMaxSpeed(1000);
    steppery2.setAcceleration(500);
}

void loop() {
stepperx1.moveTo(-100);

steppery1.moveTo(100);

stepperx2.moveTo(100);

steppery2.moveTo(-100);


stepperx1.runToPosition();
steppery1.runToPosition();
stepperx2.runToPosition();
steppery2.runToPosition();
}