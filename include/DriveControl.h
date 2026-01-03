#ifndef DRIVE_CONTROL_h
#define DRIVE_CONTROL_h

void driverControl();

void forward(int leftMotorSpeed, int rightMotorSpeed);
void backward(int leftMotorSpeed, int rightMotorSpeed);
void steerLeft(int leftMotorSpeed, int rightMotorSpeed);
void steerRight(int leftMotorSpeed, int rightMotorSpeed);
void stop();

enum DriverCommand {
  STOP = 0,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  FORWARD_LEFT,
  FORWARD_RIGHT,
  BACKWARD_LEFT,
  BACKWARD_RIGHT
};

#endif