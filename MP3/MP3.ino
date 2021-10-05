/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->  http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>

#define BAUD_RATE 115200

#define LEFT_MOTOR_PIN 3    // M1
#define RIGHT_MOTOR_PIN 2   // M2
#define LEFT_SENSOR_PIN A0    // Analog pin 0
#define RIGHT_SENSOR_PIN A1   // Analog pin 1

#define INITIAL_SPEED 150   // initial speed of wheels at setup()

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

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
  rightMotor->run(FORWARD);
  
  // turn on motors
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void loop() {
  int leftSensorValue = min(min(analogRead(LEFT_SENSOR_PIN), analogRead(LEFT_SENSOR_PIN)), analogRead(LEFT_SENSOR_PIN));
  int rightSensorValue = min(min(analogRead(RIGHT_SENSOR_PIN), analogRead(RIGHT_SENSOR_PIN)), analogRead(RIGHT_SENSOR_PIN));

  Serial.print("left sensor: ");
  Serial.print(leftSensorValue);
  Serial.print(", right sensor: ");
  Serial.println(rightSensorValue);

//  uint8_t i;
//
//  Serial.print("tick");
//
//  myMotor->run(FORWARD);
//  for (i=0; i<255; i++) {
//    myMotor->setSpeed(i);
//    delay(10);
//  }
//  for (i=255; i!=0; i--) {
//    myMotor->setSpeed(i);
//    delay(10);
//  }
//
//  Serial.print("tock");
//
//  myMotor->run(BACKWARD);
//  for (i=0; i<255; i++) {
//    myMotor->setSpeed(i);
//    delay(10);
//  }
//  for (i=255; i!=0; i--) {
//    myMotor->setSpeed(i);
//    delay(10);
//  }
//
//  Serial.print("tech");
//  myMotor->run(RELEASE);
//  delay(1000);
}
