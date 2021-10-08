#define BAUDRATE 115200

int x;

void setup() {
  Serial.begin(BAUDRATE);
  Serial.setTimeout(1);
}

void loop() {
  while (!Serial.available());
    x = Serial.readString().toInt();
    Serial.print(x + 1);
}
