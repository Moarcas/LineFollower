#include <QTRSensors.h>

const int motorLeftForwardPin = 7;   // Pinul pentru mers înainte la motorul stâng
const int motorLeftReversePin = 6;   // Pinul pentru mers înapoi la motorul stâng
const int motorRightForwardPin = 5;  // Pinul pentru mers înainte la motorul drept
const int motorRightReversePin = 4;  // Pinul pentru mers înapoi la motorul drept
const int motorLeftSpeedPin = 11;    // Pinul pentru controlul vitezei motorului stang
const int motorRightSpeedPin = 10;   // Pinul pentru controlul vitezei motorului drept

// increase kp’s value and see what happens
float kp = 5.5;
float kd = 3.3;
int p = 0;
int d = 0;
int error = 0; 
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 220;

QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };

void setup() {
  // pinMode setup
  pinMode(motorLeftForwardPin, OUTPUT);
  pinMode(motorLeftReversePin, OUTPUT);
  pinMode(motorRightForwardPin, OUTPUT);
  pinMode(motorRightReversePin, OUTPUT);
  pinMode(motorLeftSpeedPin, OUTPUT);
  pinMode(motorRightSpeedPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);
  delay(500);
  
  // calibrate sensor
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode
  calibrateSensor();
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
}

void loop() {
  int correctionSpeed = getPID();
  int motorLeftSpeed = baseSpeed;
  int motorRightSpeed = baseSpeed;
  // a bit counter intuitive because of the signs
  // basically in the first if, you substract the error from m1Speed (you add the negative)
  // in the 2nd if you add the error to m2Speed (you substract the negative)
  // it's just the way the values of the sensors and/or motors lined up
  if (error < 0) {
    motorLeftSpeed += correctionSpeed;
    //motorRightSpeed -= correctionSpeed / 10;
    //Serial.println(correctionSpeed);
  } else if (error > 0) {
    motorRightSpeed -= correctionSpeed;
    //motorLeftSpeed += correctionSpeed / 10; 
    //Serial.println(correctionSpeed);
  } 
  // make sure it doesn't go past limits. You can use -255 instead of 0 if calibrated programmedproperly.
  // making sure we don't go out of bounds
  // maybe the lower bound should be negative, instead of 0? This of what happens when making asteep turn
  motorLeftSpeed = constrain(motorLeftSpeed, -100, maxSpeed);
  motorRightSpeed = constrain(motorRightSpeed, -100, maxSpeed);
  setMotorSpeed(motorLeftSpeed, motorRightSpeed);
}

// calculate PID value based on error, kp, kd, ki, p, i and d.
float getPID() {
  error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);
  for (int i = 0; i < 6; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  p = error;
  d = error - lastError;
  lastError = error;
  return kp * p + kd * d;
}

// each arguments takes values between -255 and 255. The negative values represent the motor speedin reverse.
void setMotorSpeed(int motorRightSpeed, int motorLeftSpeed) {
  if (motorLeftSpeed == 0) {
    digitalWrite(motorLeftForwardPin, LOW);
    digitalWrite(motorLeftReversePin, LOW);
  } else {
    if (motorLeftSpeed > 0) {
      digitalWrite(motorLeftForwardPin, HIGH);
      digitalWrite(motorLeftReversePin, LOW);
    }
    if (motorLeftSpeed < 0) {
      digitalWrite(motorLeftForwardPin, LOW);
      digitalWrite(motorLeftReversePin, HIGH);
      motorLeftSpeed = -motorLeftSpeed;
    }
  }
   if (motorRightSpeed == 0) {
    digitalWrite(motorRightForwardPin, LOW);
    digitalWrite(motorRightReversePin, LOW); 
  } else {
    if (motorRightSpeed > 0) { 
      digitalWrite(motorRightForwardPin, HIGH);
      digitalWrite(motorRightReversePin, LOW);
    }
    if (motorRightSpeed < 0) {
      digitalWrite(motorRightForwardPin, LOW);
      digitalWrite(motorRightReversePin, HIGH);
      motorRightSpeed = -motorRightSpeed;
    }
  }
  analogWrite(motorLeftSpeedPin, motorLeftSpeed);
  analogWrite(motorRightSpeedPin, motorRightSpeed);
}

void swap(int &a, int &b) {
  int c = a;
  a = b;
  b = c;
}

void calibrateSensor() {
  int numberTurns = 5;
  long long int lastTurn = millis();
  const int turnTime = 1000;
  int motorRightSpeed = 100;
  int motorLeftSpeed = 100;
  while (numberTurns) {
    //qtr.readLineBlack(sensorValues);
    //if (sensorValues[0] < 20) 
    qtr.calibrate();
    if (millis() - lastTurn > turnTime) {
      //swap(motorLeftSpeed, motorRightSpeed);
      motorRightSpeed = -motorRightSpeed;
      motorLeftSpeed = -motorLeftSpeed;
      //setMotorSpeed(motorLeftSpeed, motorRightSpeed);
      numberTurns--;
      lastTurn = millis();
    }
  }
}