#include <Adafruit_MotorShield.h>

#define BAUD_RATE 115200

#define LEFT_MOTOR_PIN 3    // M1
#define RIGHT_MOTOR_PIN 2   // M2
#define LEFT_SENSOR_PIN     // TODO
#define RIGHT_SENSOR_PIN    // TODO

#define INITIAL_SPEED 150   // initial speed of wheels at setup()

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *leftMotor = AFMS.getMotor(LEFT_MOTOR_PIN);   // attach left motor
Adafruit_DCMotor *rightMotor = AFMS.getMotor(RIGHT_MOTOR_PIN);  // attach right motor

enum states {     // enumerate states
    STRAIGHT,
    LEFT,
    RIGHT,
    STOP
};

states current_state;   // initialize variable to store current state

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);     // set up Serial library

  // set up sensors - technically not necessary, since digital pins default to input
//  pinMode(LEFT_SENSOR_PIN, INPUT);
//  pinMode(RIGHT_SENSOR_PIN, INPUT);

  // code below from DCMotorTest example code
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
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

  current_state = STRAIGHT;
}

void loop() {
  // put your main code here, to run repeatedly:
//  int leftSensorValue = min(min(analogRead(LEFT_SENSOR_PIN), analogRead(LEFT_SENSOR_PIN)), analogRead(LEFT_SENSOR_PIN));
//  int rightSensorValue = min(min(analogRead(RIGHT_SENSOR_PIN), analogRead(RIGHT_SENSOR_PIN)), analogRead(RIGHT_SENSOR_PIN));
  int leftSensorValue = analogRead(LEFT_SENSOR_PIN);
  int rightSensorValue = analogRead(RIGHT_SENSOR_PIN);
  Serial.print("left sensor: ");
  Serial.print(leftSensorValue);
  Serial.print(", right sensor: ");
  Serial.println(rightSensorValue);
//  switch (current_state) {
//    case STRAIGHT:
//
//    case LEFT:
//
//    case RIGHT:
//
//    case STOP:
//    
  }
}
