#include "Arduino.h"
#include "DumbStepper.h"

DumbStepper::DumbStepper(int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4) {
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;
  this->motor_pin_3 = motor_pin_3;
  this->motor_pin_4 = motor_pin_4;
  state = 0;
}


void DumbStepper::stepForward() {
  int temp = (state + 1) % 4;
  stepMotor(temp < 0 ? temp + 4 : temp);
}

void DumbStepper::stepBackward() {
  int temp = (state - 1) % 4;
  stepMotor(temp < 0 ? temp + 4 : temp);
}

/*
 * Moves the motor forward or backwards.
 */
void DumbStepper::stepMotor(int nextState)
{
  switch (nextState) {
    case 0:  // 1010
      analogWrite(motor_pin_1, 255);
      analogWrite(motor_pin_2, 0);
      analogWrite(motor_pin_3, 255);
      analogWrite(motor_pin_4, 0);
    break;
    case 1:  // 0110
      analogWrite(motor_pin_1, 0);
      analogWrite(motor_pin_2, 255);
      analogWrite(motor_pin_3, 255);
      analogWrite(motor_pin_4, 0);
    break;
    case 2:  //0101
      analogWrite(motor_pin_1, 0);
      analogWrite(motor_pin_2, 255);
      analogWrite(motor_pin_3, 0);
      analogWrite(motor_pin_4, 255);
    break;
    case 3:  //1001
      analogWrite(motor_pin_1, 255);
      analogWrite(motor_pin_2, 0);
      analogWrite(motor_pin_3, 0);
      analogWrite(motor_pin_4, 255);
    break;
  }

  state = nextState;
}

/*
 * Moves the motor forward or backwards.
 */
void DumbStepper::idleMotor()
{
  analogWrite(motor_pin_1, 127);
  analogWrite(motor_pin_2, 127);
  analogWrite(motor_pin_3, 127);
  analogWrite(motor_pin_4, 127);
}
