#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Define constants
#define LEFT_MOTOR_PIN 3      // Set the left motor to pin M3
#define RIGHT_MOTOR_PIN 2     // Set the right motor to pin M2
#define LEFT_SENSOR_PIN A1    // Set left sensor to analog pin 1
#define RIGHT_SENSOR_PIN A0   // Set right sensor to analog pin 0


// Initialize variables for serial reading
String command = "";    // for incoming serial data
double incomingData;    

// Initialize coefficients for tuning
double kp = 30; // Coefficient proportional gain
double ki = 0.002; // Coefficient for integral gain
double kd = 3000; // Coefficient for derivative gain.

// Track times
uint32_t currentTime, previousTime, elapsedTime;

const uint16_t INITIAL_SPEED = 50; // Set the intial speed of the motors
const uint16_t THRESHOLD = 200; // Define the threshold for floor/tape
const uint32_t BAUD_RATE = 115200;

// Track error
int sensor[2] = {0, 0}; // Intialize values for left and right sensors.
                        //    errors can be -1, 0, or 1 depending on which side
                        //    of the tape the robot is on.
                        //    sensor[0] is left wheel. sensor[1] is right wheel.
int error, previousError; // Track current and previous errors
double PID = 0.0; // Initialize PID for control loop 
double cumeError, rateError; 



// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *leftMotor = AFMS.getMotor(LEFT_MOTOR_PIN);   // attach left motor
Adafruit_DCMotor *rightMotor = AFMS.getMotor(RIGHT_MOTOR_PIN);  // attach right motor

void setup() {
  Serial.begin(BAUD_RATE);           
  Serial.setTimeout(1);

  // set up sensors - technically not necessary, since digital pins default to input
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  previousTime = millis(); // Get the current value of the millis timer
  previousError = 0;

  delay(10000); // Delay so that the robot does not start moving right away
  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(INITIAL_SPEED-3); // Subtract by 3 do to unequal motor speeds
  rightMotor->setSpeed(INITIAL_SPEED);

  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  

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

  // Define sensor values as 1 if they are greater than the threshold
  //    meaning that the sensor is reading the tape. Define sensor values
  //    as 0 if they are less than the threshold meaning that the sensor is
  //    is reading the ground.
  sensor[0] = analogRead(LEFT_SENSOR_PIN) > THRESHOLD; 
  sensor[1] = analogRead(RIGHT_SENSOR_PIN) > THRESHOLD;

  if ((sensor[0] == 1) && (sensor[1] == 1)){ // If robot is on tape, error = 0.
    error = 0;
  } 
  if ((sensor[0] == 1) && (sensor[1] == 0)){ // If robot right of tape, error = -1.
    error = -1;
  }
  if ((sensor[0] == 0) && (sensor[1] == 1)){ // If robot left of tape, error = 1.
    error = 1;
  }

  PID = computePID(error);

  // If robot is to the right of the tape, decrease left motor speed and increase
  // right motor speed. If robot is to the left of the tape, increase left motor speed
  // and decrease right motor speed.
  leftMotor->setSpeed(INITIAL_SPEED -3 + PID); 
  rightMotor->setSpeed(INITIAL_SPEED - PID);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);

  
}

// Compute PID
double computePID(double error) {
  currentTime = millis(); // Get the current value of the millis timer
  elapsedTime = double(currentTime - previousTime);

  cumeError += error*elapsedTime;                 // compute integral
  rateError = (error - previousError)/elapsedTime;    // compute derivative

  double diff = kp * error + ki * cumeError + kd * rateError;   // compute PID output

  previousError = error;
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
