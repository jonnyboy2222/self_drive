#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 6); // RX=10, TX=11

void setup() {
  mySerial.begin(9600);     // ESP32-CAM 통신용
  Serial.begin(9600);
}

void loop() {
  String data = "Hello from Arduino!";
  mySerial.println(data);
  delay(1000);

  if (mySerial.available() > 0)
  {
    String receivedData = mySerial.readStringUntil('\n');
    Serial.print(receivedData);
  }
}
