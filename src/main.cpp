#include <Arduino.h>
#include <SoftwareSerial.h>
#include "CarBrain.h"
#include <L298N.h>

SoftwareSerial bluetoothDevice(11, 12);

// L298N Bridges
#define ENA 9
#define IN1 A0
#define IN2 A1
#define ENB 10
#define IN3 A2
#define IN4 A3
#define HVCC A5

// Motors
L298N motorRight(ENA, IN1, IN2);
L298N motorLeft(ENB, IN3, IN4);

// IR Module 
#define S5 4
#define S4 5
#define S3 6
#define S2 7
#define S1 8

CarMode carMode;

Instruction instruction;

int sensorLM, sensorL, sensorC, sensorR, sensorRM;
const float sensorLMW = -1.5, sensorLW = -0.75, sensorCW = 0, sensorRW = 0.75, sensorRMW = 1.5;
float sensorSum, sensorAvg;

float error = 0, previousError = 0;
float Kp = 18, Ki = 0.00, Kd = 10.00;
float lineFollowerBaseSpeed = 140;
float lineFollowerLeftBaseSpeed = lineFollowerBaseSpeed;
float lineFollowerRightBaseSpeed = 0.85 * lineFollowerBaseSpeed;
float I = 0;

// Keep track of which parameter is calibrated
int parameterToCalibrateIndex = 0;  
const int maxParametersToCalibrate = 5;

// Tune these 4 values to achieve aproximative equal motor speeds :)))
// My soldering was pretty bad or the L298N is incredibly bad
// either way, these values need to be tuned
const int LEFT_FWD  = 255;
const int RIGHT_FWD = 190;

const int LEFT_BWD  = 170;
const int RIGHT_BWD = 170;


void setup() {
  Serial.begin(115200);
  bluetoothDevice.begin(9600);

  digitalWrite(HVCC, HIGH);

  // IR Module
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // carMode
  carMode = SETUP_MODE;
  parameterToCalibrateIndex = 0;

  instruction.processed = false;
}

void loop() {

  if (bluetoothDevice.available() >= 2) readInstruction();
  else instruction = {0, 0, false};
  
  if (shouldRunControlLoop())
    processInstruction();
}

// Instruction must be processed either if instruction was not yet processed, 
// or it is in line follower mode which needs continuous processing
bool shouldRunControlLoop() {
  return !instruction.processed || carMode == LINE_FOLLOWER_MODE;
}

void readInstruction() {
  instruction.type = bluetoothDevice.read();
  instruction.value = bluetoothDevice.read() - '0';
  
  Serial.println(instruction.type);
  Serial.println(instruction.value);
  
  instruction.processed = false;
}

void processInstruction() {
  handleModeChange();
  runCurrentMode();
  
  instruction.processed = true;
}

void handleModeChange() {
  // Check if operation mode was changed   
  if (instruction.type == 'S') {
    switch (instruction.value) {
      case SETUP_MODE:
        carMode = SETUP_MODE;
        Serial.println("Setup Mode SELECTED");
        break;
      case DRIVER_MODE:
        carMode = DRIVER_MODE;
        Serial.println("Driver Mode SELECTED");
        break;
      case LINE_FOLLOWER_MODE:
        carMode = LINE_FOLLOWER_MODE;
        Serial.println("Line Follower Mode SELECTED");
        break;
    }
    stop();
  }
}

void runCurrentMode() {
  switch (carMode) {
    case SETUP_MODE:
      setupControl();
      break;
    case DRIVER_MODE:
      driverControl();
      break;
    case LINE_FOLLOWER_MODE:
      lineFollowerControl();
      break;
  }
}

void driverControl() {
  if (instruction.type == 'D') {
    switch (instruction.value) {
      case STOP:
        stop();
        break;
      case FORWARD:
        forward(LEFT_FWD, RIGHT_FWD);
        break;
      case BACKWARD:
        backward(LEFT_BWD, RIGHT_BWD);
        break;
      case LEFT:
        steerLeft(LEFT_FWD, RIGHT_FWD);
        break;
      case RIGHT:
        steerRight(LEFT_FWD, RIGHT_FWD);
        break;
      case FORWARD_LEFT:
        forward(LEFT_FWD / 2, RIGHT_FWD);
        break;
      case FORWARD_RIGHT:
        forward(LEFT_FWD, RIGHT_FWD / 2);
        break;
      case BACKWARD_LEFT:
        backward(LEFT_BWD / 2, RIGHT_BWD);
        break;
      case BACKWARD_RIGHT:
        backward(LEFT_BWD, RIGHT_BWD / 2);
        break;
    }
  }
}

void lineFollowerControl() {
  readIRData();
  pid();
}

void setupControl(){
  // 'W' command = Wait for specified number of bytes(chars) 
  // Paramteres calibration loops trough: Kp, Ki, Kd, lineFollowerBaseSpeed 
  if (instruction.type == 'W') { 
    if (parameterToCalibrateIndex == maxParametersToCalibrate) parameterToCalibrateIndex = 0;

    calibrateLineFollowerData(parameterToCalibrateIndex, instruction.value);
    parameterToCalibrateIndex++;
  }
}

void calibrateLineFollowerData(int component, int length) {
  // Small delay to allow data to arrive
  delay(5);

  String data = "";
  for (int i = 0; i < length && bluetoothDevice.available(); i++) {
      char c = bluetoothDevice.read();
      data += c; 
  }
  data += '\n';
  
  switch (component) {
    case 0:
      Kp = convertStringToFloat(length, data);
      break;
    case 1:
      Ki = convertStringToFloat(length, data);
      break;
    case 2:
      Kd = convertStringToFloat(length, data);
      break;
    case 3:
      lineFollowerLeftBaseSpeed = convertStringToFloat(length, data);
      break;
    case 4: 
      lineFollowerRightBaseSpeed = convertStringToFloat(length, data);
      break;
    default:
      return;
  }
}

float convertStringToFloat(unsigned int length, String data) {
    // Parameters calibrated must be valid floats
    if (length <= 0 || data.length() == 0) return 0.0f;

    if (data.length() > length) data = data.substring(0, length);

    return data.toFloat();
}

void forward(int leftMotorSpeed, int rightMotorSpeed) {
  motorLeft.setSpeed(leftMotorSpeed);
  motorRight.setSpeed(rightMotorSpeed);

  motorLeft.forward();
  motorRight.forward();
}
void backward(int leftMotorSpeed, int rightMotorSpeed) {
  motorLeft.setSpeed (leftMotorSpeed);
  motorRight.setSpeed (rightMotorSpeed);

  motorLeft.backward();
  motorRight.backward();
}

void steerLeft(int leftMotorSpeed, int rightMotorSpeed) {
  motorLeft.setSpeed(leftMotorSpeed);
  motorRight.setSpeed(rightMotorSpeed);

  motorLeft.backward();
  motorRight.forward();
}
void steerRight(int leftMotorSpeed, int rightMotorSpeed) {
  motorLeft.setSpeed(leftMotorSpeed);
  motorRight.setSpeed(rightMotorSpeed);

  motorLeft.forward();
  motorRight.backward();
}

void stop() {
  motorLeft.stop();
  motorRight.stop();
}

void pid() {
  sensorSum = 0;
  sensorAvg = 0;

  sensorAvg =  sensorLM * sensorLMW + 
                sensorL  * sensorLW  +
                sensorC  * sensorCW  +
                sensorR  * sensorRW  +
                sensorRM * sensorRMW;
  sensorSum = sensorLM + sensorL + sensorC + sensorR + sensorRM;

  if (sensorSum != 0) error = sensorAvg / sensorSum;

  float P = error;
  I += error;
  float D = error - previousError;

  previousError = error;

  float correction = Kp * P + Ki * I + Kd * D;
  int leftSpeed = constrain(lineFollowerBaseSpeed + correction, 0, 255);
  int rightSpeed = constrain(lineFollowerBaseSpeed - correction, 0, 255);

  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.print("║ P: ");
  Serial.print(P, 3);
  Serial.print("\t│ I: ");
  Serial.print(I, 3);
  Serial.print("\t│ D: ");
  Serial.print(D, 3);
  Serial.println("\t║");
  Serial.println("╟────────────────────────────────────────────────────────╢");
  Serial.print("║ Error: ");
  Serial.print(error, 3);
  Serial.print("\t│ Correction: ");
  Serial.print(correction, 2);
  Serial.println("\t\t║");
  Serial.println("╟────────────────────────────────────────────────────────╢");
  Serial.print("║ Left: ");
  Serial.print(leftSpeed);
  Serial.print("\t│ Right: ");
  Serial.print(rightSpeed);
  Serial.println("\t\t\t║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
  // delay(1000);
  // printIRData();
  forward(leftSpeed, rightSpeed);
}

void readIRData() {
  // The sensor module I used was a digital one
  sensorLM = digitalRead(S1);
  sensorL  = digitalRead(S2);
  sensorC  = digitalRead(S3);
  sensorR  = digitalRead(S4);
  sensorRM = digitalRead(S5);

  convertIRData();
}

void convertIRData() {
  // The IR sensors send '0' on line detection
  // For my ease, i perform a negation on all values 
  sensorLM = !sensorLM;
  sensorL  = !sensorL;
  sensorC  = !sensorC;
  sensorR  = !sensorR;
  sensorRM = !sensorRM;
}

void printIRData(){
  Serial.println();
  Serial.print("S1 = ");
  Serial.print(sensorLM);
  
  Serial.print(" | S2 = ");
  Serial.print(sensorL);

  Serial.print(" | S3 = ");
  Serial.print(sensorC);
  
  Serial.print(" | S4 = ");
  Serial.print(sensorR);
  
  Serial.print(" | S5 = ");
  Serial.println(sensorRM);
  delay(3000);
}