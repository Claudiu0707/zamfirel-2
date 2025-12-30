#ifndef DRIVE_CONTROL_h
#define DRIVE_CONTROL_h

void driverControl();

void forward(int leftMotorSpeed, int rightMotorSpeed);
void reverse(int leftMotorSpeed, int rightMotorSpeed);
void steerLeft(int leftMotorSpeed, int rightMotorSpeed);
void steerRight(int leftMotorSpeed, int rightMotorSpeed);
void stop();
#endif