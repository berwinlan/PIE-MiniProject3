#define BAUDRATE 115200

int x;
String command = "";

void setup() {
  Serial.begin(BAUDRATE);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == '\r') {
      Serial.println(command);
      command = "";
    } else {
      command += ch;
    }
  }
}
