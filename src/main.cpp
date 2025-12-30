#include <Arduino.h>
#include <SoftwareSerial.h>
#include "CarBrain.h"

SoftwareSerial SerialBT(11, 12);

// H-Bridge
#define ENA 9
#define IN1 A0
#define IN2 A1
#define ENB 10
#define IN3 A2
#define IN4 A3
#define HVCC A5

// IR Module 
#define S5 4
#define S4 5
#define S3 6
#define S2 7
#define S1 8

int carMode;          // 0 - STAND BY | 1 - DRIVER MODE | 2 - LINE FOLLOWER
int instruction[2];
bool instructionProcessed;
int bytesRead = 0;

int sensorLM, sensorL, sensorC, sensorR, sensorRM;
int sensorLMW = -2, sensorLW = -1, sensorCW = 0, sensorRW = 1, sensorRMW = 2;
int sensorSum, sensorAvg;
float error = 0, previousError = 0;
float Kp = 50, Ki = 0.10, Kd = 15.00;
float I = 0;
float baseSpeed = 90;

int motorSpeed=255;
int halfMotorSpeed = 60;

unsigned long instructionExecutionTime = 5000; // 500 ms
unsigned long previousTime, currentTime;

void setup() {
  Serial.begin(115200);
  SerialBT.begin(9600);

  // H-Bridge
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(HVCC, HIGH);

  // IR Module
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // carMode
  carMode = 2;
  currentTime = previousTime = 0;
  instructionProcessed = false;
}

void loop() {
  currentTime = millis();


  if (SerialBT.available() >= 2) {
      readInstruction();
  }
  else {
      instruction[0] = 0;
      instruction[1] = 0;
  }

  if(currentTime - previousTime > instructionExecutionTime) {
      previousTime = currentTime;
      stop();
  } else {
      if (!instructionProcessed)
        processInstruction();
  }
  readIRData();
  convertIRData();
  pidCalculation();
}

void readInstruction() {
  bytesRead++;
  instruction[0] = SerialBT.read();
  instruction[0] = instruction[0];
  bytesRead++;
  instruction[1] = SerialBT.read();
  instruction[1] = instruction[1] - '0';
  bytesRead = 0;
  // Serial.println(instruction[0]);
  // Serial.println(instruction[1]);
  instructionProcessed = false;
}

void processInstruction() {
  if (carMode == 1) {
    driverControl();
  } else if (carMode == 2) {
    lineFollowerControl();
  }
  
  instructionProcessed = true;
}

void driverControl() {
  if (instruction[0] == 'D') {
      switch (instruction[1]) {
        case 0:
          stop();
          break;
        case 1:
          forward(motorSpeed, motorSpeed);
          break;
        case 2:
          reverse(motorSpeed, motorSpeed);
          break;
        case 3:
          steerLeft(motorSpeed, motorSpeed);
          break;
        case 4: 
          steerRight(motorSpeed, motorSpeed);
          break;
        case 5:
          forward(motorSpeed, halfMotorSpeed);
          break;
        case 6:
          forward(halfMotorSpeed, motorSpeed);
          break;
        case 7:
          reverse(motorSpeed, halfMotorSpeed);
          break;
        case 8:
          reverse(halfMotorSpeed, motorSpeed);
          break;
      }
    }
}
void lineFollowerControl() {
  
}



void forward(int leftMotorSpeed, int rightMotorSpeed) {
  analogWrite(ENA, leftMotorSpeed);
  analogWrite(ENB, rightMotorSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void reverse(int leftMotorSpeed, int rightMotorSpeed) {
  analogWrite(ENA, leftMotorSpeed);
  analogWrite(ENB, rightMotorSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void steerLeft(int leftMotorSpeed, int rightMotorSpeed) {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void steerRight(int leftMotorSpeed, int rightMotorSpeed) {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void pidCalculation() {
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
  int leftSpeed = constrain(baseSpeed - correction, 0, 255);
  int rightSpeed = constrain(baseSpeed + correction, 0, 255);

  {
    Serial.print("leftSpeed: ");
    Serial.print(leftSpeed);
    Serial.print("  | rightSpeed: ");
    Serial.print(rightSpeed);
    Serial.print("  |  error: ");
    Serial.println(error);
  }
  
  forward(leftSpeed, rightSpeed);
  // delay(300);
}

void readIRData() {
  sensorLM = digitalRead(S1);
  sensorL  = digitalRead(S2);
  sensorC  = digitalRead(S3);
  sensorR  = digitalRead(S4);
  sensorRM = digitalRead(S5);
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
  Serial.print(sensorRM);
  delay(3000);
}
void convertIRData() {
  sensorLM = 1 - sensorLM * 1;
  sensorL  = 1 - sensorL * 1;
  sensorC  = 1 - sensorC * 1;
  sensorR  = 1 - sensorR * 1;
  sensorRM = 1 - sensorRM * 1;
}
