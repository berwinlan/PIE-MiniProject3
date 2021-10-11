/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->  http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>

// Define constants
#define BAUD_RATE 115200      // make sure Serial Monitor is set to "No line ending"

#define LEFT_MOTOR_PIN 3      // M3
#define RIGHT_MOTOR_PIN 2     // M2
#define LEFT_SENSOR_PIN A0    // Analog pin 0
#define RIGHT_SENSOR_PIN A1   // Analog pin 1

#define INITIAL_SPEED 50   // initial speed of wheels at setup()

// Initialize variables
int incomingData;    // for incoming serial data

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *leftMotor = AFMS.getMotor(LEFT_MOTOR_PIN);   // attach left motor
Adafruit_DCMotor *rightMotor = AFMS.getMotor(RIGHT_MOTOR_PIN);  // attach right motor

void setup() {
  Serial.begin(BAUD_RATE);           // set up Serial library

  // set up sensors - technically not necessary, since digital pins default to input
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(INITIAL_SPEED);
  rightMotor->setSpeed(INITIAL_SPEED);

  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  
  // turn on motors
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void loop() {

  receiveData();

  int leftSensorValue = min(min(analogRead(LEFT_SENSOR_PIN), analogRead(LEFT_SENSOR_PIN)), analogRead(LEFT_SENSOR_PIN));
  int rightSensorValue = min(min(analogRead(RIGHT_SENSOR_PIN), analogRead(RIGHT_SENSOR_PIN)), analogRead(RIGHT_SENSOR_PIN));

//  Serial.print("left sensor: ");
//  Serial.print(leftSensorValue);
//  Serial.print(", right sensor: ");
//  Serial.println(rightSensorValue);

  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
}

// Receive data in the form "letterNumber" where letter = {p, i, d}
void receiveData() {
  if (Serial.available() >= 2) {   // can change to Serial.available() >= {known input length}
    char letter = Serial.read();
    switch (letter) {
      case 'p': incomingData = Serial.parseInt();
        Serial.print("case: ");
        Serial.print(letter);
        Serial.print(", data: ");
        Serial.println(incomingData);
        
      case 'i': incomingData = Serial.parseInt();
        Serial.print("case: ");
        Serial.print(letter);
        Serial.print(", data: ");
        Serial.println(incomingData);
        
      case 'd': incomingData = Serial.parseInt();
        Serial.print("case: ");
        Serial.print(letter);
        Serial.print(", data: ");
        Serial.println(incomingData);

      default: 
        Serial.print("Unexpected command: ");
        Serial.println(letter);
    }
  }
}
