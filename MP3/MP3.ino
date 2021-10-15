/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <PID_v1.h>

// Define constants
#define BAUD_RATE 115200      // make sure Serial Monitor is set to "No line ending"

#define LEFT_MOTOR_PIN 3      // M3
#define RIGHT_MOTOR_PIN 2     // M2
#define LEFT_SENSOR_PIN A0    // Analog pin 0
#define RIGHT_SENSOR_PIN A1   // Analog pin 1

#define INITIAL_SPEED 50   // initial speed of wheels at setup()

#define THRESHOLD 350       // threshold of tape vs. floor

#define FREQ 1000         // data collection frequency (ms)

// Initialize variables for serial reading
String command = "";    // for incoming serial data
double incomingData;    // placeholder, will refactor/rewrite code later

// Initialize coefficients for tuning
double kp = 0;
double ki = 0;
double kd = 0;

// Track times
unsigned long currentTime, previousTime;
double elapsedTime;
double currentError, previousError;

// Track error
double error, cumeError, rateError;

// Global variables to store data
int leftSensorValue, rightSensorValue, leftMotorValue, rightMotorValue;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *leftMotor = AFMS.getMotor(LEFT_MOTOR_PIN);   // attach left motor
Adafruit_DCMotor *rightMotor = AFMS.getMotor(RIGHT_MOTOR_PIN);  // attach right motor

void setup() {
  Serial.begin(BAUD_RATE);           // set up Serial library
  Serial.setTimeout(1);

  // set up sensors - technically not necessary, since digital pins default to input
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  previousTime = millis();
  previousError = 0;

  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(INITIAL_SPEED);
  rightMotor->setSpeed(INITIAL_SPEED);

  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  
  // turn on motors
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void loop() {
  // Dynamically tune PID with the Serial Monitor
  // Receive data from serial monitor in the form "letterValue" where letter = {p, i, d}
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == '\r') {
      parseData(command);
      command = "";
    } else {
      command += ch;
    }
  }

  leftSensorValue = analogRead(LEFT_SENSOR_PIN);
  rightSensorValue = analogRead(RIGHT_SENSOR_PIN);
//  printSensors(leftSensorValue, rightSensorValue);

  leftMotorValue = analogRead(LEFT_MOTOR_PIN);
  rightMotorValue = analogRead(RIGHT_MOTOR_PIN);

  serialDisplay(leftSensorValue, rightSensorValue, leftMotorValue, rightMotorValue);
  delay(FREQ);

  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

// Compute PID
double computePID(double input) {
  currentTime = millis();
  elapsedTime = double(currentTime - previousTime);

  currentError = THRESHOLD - input;                 // compute error
  cumeError += error * elapsedTime;                 // compute integral
  rateError = (error - previousError) / elapsedTime;    // compute derivative

  double diff = kp * error + ki * cumeError + kd * rateError;   // compute PID output

  previousError = currentError;
  previousTime = currentTime;

  return diff;        // return PID output
}

// Helper function for receiveData
void parseData(String command) {
  String s = command.substring(0, 1);      // slice out the first value of the command
  incomingData = command.substring(1).toDouble();    // slice out the numerical data
  
  if (s == "p") {
    kp = incomingData;
    Serial.print("kp set to ");
    Serial.println(incomingData);
  } else if (s == "i") {
    ki = incomingData;
    Serial.print("ki set to ");
    Serial.println(incomingData);
  } else if (s == "d") {
    kd = incomingData;
    Serial.print("kd set to ");
    Serial.println(incomingData);
  } else if (s == "x") {
    Serial.println("Braking!");
    brake();
  } else if (s == "q") {
    Serial.println("Resetting!");
    kp = 0;
    ki = 0;
    kd = 0;
  } else {
    Serial.print("Unexpected command: ");
    Serial.println(command);
  }
}

// Brake the vehicle
void brake() {
  // Set the speed to 0
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);

  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  
  // turn on motors
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

// Analyze sensor values
void printSensors(int left, int right) {
  Serial.print("left sensor: ");
  Serial.print(left);
  Serial.print(" right sensor: ");
  Serial.println(right);
  
  if (left > THRESHOLD) {
    Serial.print(" left on tape ");
  } else {
    Serial.print(" left on floor ");
  }
  if (right > THRESHOLD) {
    Serial.println(" right on tape ");
  } else {
    Serial.println(" right on floor ");
  }
}

// Print values to serial monitor
void serialDisplay(int leftSensorValue, int rightSensorValue, int leftMotorValue, int rightMotorValue) {
  Serial.print(leftSensorValue);
  Serial.print(",");
  Serial.print(rightSensorValue);
  Serial.print(",");
  Serial.print(leftMotorValue);
  Serial.print(",");
  Serial.println(rightMotorValue);
}
