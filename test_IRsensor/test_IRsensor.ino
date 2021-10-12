


int sensorInput1 = A0;
int sensorInput2 = A1;    
int sensorValue1 = 0;  // variable to store the value coming from the sensor
int sensorValue2 = 0;

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(115200);
  // have delay after starting serial
  delay(200);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

}

void loop() {
  // read the value from the sensor:
  sensorValue1 = analogRead(sensorInput1);
  sensorValue2 = analogRead(sensorInput2);
 // Serial.print("First Sensor: ");
  Serial.println(sensorValue1);
 // Serial.print("Second Sensor: ");
 // Serial.println(sensorValue2);
  // stop the program for <sensorValue> milliseconds:
  delay(20);


}
