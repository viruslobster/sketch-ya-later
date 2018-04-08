#ifndef XYSteppers_h
#define XYSteppers_h

#include "DumbStepper.h"
#define MAX_X 3160
#define MAX_Y 2107
#define X_WIGGLE 30
#define Y_WIGGLE 30

// library interface description
class XYSteppers {
  public:
    // constructors:
    XYSteppers (int max_speed, int motor_1_pin_1, int motor_1_pin_2,
                int motor_1_pin_3, int motor_1_pin_4, int motor_2_pin_1,
                int motor_2_pin_2, int motor_2_pin_3, int motor_2_pin_4);

    // mover method:
    void moveToPoint (int x, int y);
    void idleMotors ();

  private:
    void stepMotors (int x, int y);

    DumbStepper* xStepper;
    DumbStepper* yStepper;

    int max_speed;
    unsigned long minDelay;
    bool xForward;
    bool yForward;
    int xLocation;
    int yLocation;
};

#endif
