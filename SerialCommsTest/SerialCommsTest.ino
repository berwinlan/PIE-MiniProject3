/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->  http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>

#define BAUD_RATE 115200
// make sure Serial Monitor is set to "No line ending"

#define LEFT_MOTOR_PIN 3    // M3
#define RIGHT_MOTOR_PIN 2   // M2
#define LEFT_SENSOR_PIN A0    // Analog pin 0
#define RIGHT_SENSOR_PIN A1   // Analog pin 1

#define INITIAL_SPEED 50   // initial speed of wheels at setup()

int incomingData;    // for incoming serial data
bool newData;

void setup() {
  Serial.begin(BAUD_RATE);           // set up Serial library

}

void loop() {

  receiveData();
//  showNewData();
}

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
    
    newData = true;
  }
}

void showNewData() {
  if (newData == true) {
    
    newData = false;
  }

// send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte (0-255):
    incomingData = Serial.parseFloat();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingData, 5);
  }
}
