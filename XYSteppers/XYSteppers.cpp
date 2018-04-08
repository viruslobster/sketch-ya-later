#include "Arduino.h"
#include "XYSteppers.h"

// Note: max_speed is in steps per second
XYSteppers::XYSteppers (int max_speed, int motor_1_pin_1, int motor_1_pin_2, int motor_1_pin_3, int motor_1_pin_4, int motor_2_pin_1, int motor_2_pin_2, int motor_2_pin_3, int motor_2_pin_4) {
  xStepper = new DumbStepper(motor_1_pin_1, motor_1_pin_2, motor_1_pin_3, motor_1_pin_4);
  yStepper = new DumbStepper(motor_2_pin_1, motor_2_pin_2, motor_2_pin_3, motor_2_pin_4);
  this->max_speed = max_speed;
  minDelay = 1000L * 1000L / max_speed;
  xForward = true;
  yForward = true;
  xLocation = 0;
  yLocation = 0;
}

void XYSteppers::moveToPoint (int x, int y) {
  if (x > 0 && x < MAX_X && y > 0 && y < MAX_Y) {
    int xFixWiggle = (x > xLocation && !xForward) ? X_WIGGLE :
                     (x < xLocation && xForward) ? -X_WIGGLE : 0;
    int yFixWiggle = (y > yLocation && !yForward) ? Y_WIGGLE :
                     (y < yLocation && yForward) ? -Y_WIGGLE : 0;
    stepMotors(xFixWiggle, yFixWiggle);

    xForward = (x < xLocation) ? false : (x > xLocation) ? true : xForward;
    yForward = (y < yLocation) ? false : (y > yLocation) ? true : yForward;

    stepMotors(x - xLocation, y - yLocation);
    xLocation = x;
    yLocation = y;
  }
}

void XYSteppers::stepMotors (int x, int y) {
  if (x != 0 || y != 0) {
    unsigned long startTime = micros();

    int maxSteps = max(abs(x), abs(y));
    unsigned long minScaledDelay = (unsigned long) (minDelay * (sqrt(1.0L*x*x+1.0L*y*y)/maxSteps));
    int xSteps = 0;
    int ySteps = 0;

    unsigned long xStepDelay = x == 0 ? -1 : maxSteps * minScaledDelay / abs(x);
    unsigned long yStepDelay = y == 0 ? -1 : maxSteps * minScaledDelay / abs(y);

    unsigned long lastXTime = 0;
    unsigned long lastYTime = 0;

    unsigned long now;

    while (xSteps < abs(x) || ySteps < abs(y))  {
      now = micros();
      if (x != 0 && now - lastXTime >= xStepDelay) {
        if (x > 0) {
          xStepper->stepBackward();
        }
        else {
          xStepper->stepForward();
        }
        xSteps++;
        lastXTime = now;
      }
      if (y != 0 && now - lastYTime >= yStepDelay) {
        if (y > 0) {
          yStepper->stepForward();
        }
        else {
          yStepper->stepBackward();
        }
        ySteps++;
        lastYTime = now;
      }
    }
  }
}

void XYSteppers::idleMotors() {
  xStepper->idleMotor();
  yStepper->idleMotor();
}
