#include <SoftwareSerial.h>
#include <ArduinoJson.h>

unsigned long prev;
unsigned long current;
SoftwareSerial espSerial(2, 3); // RX, TX

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  espSerial.begin(9600);
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
  if (current - prev > 5000)
  { 
    StaticJsonDocument<200> doc;

    doc["sensor"] = "gps";
    doc["time"] = 1351824120;
    doc["data"][0] = 48.7;
    doc["data"][1] = 2.3;

    // Send JSON to ESP32 via SoftwareSerial pins
    serializeJson(doc, espSerial);
    espSerial.println();
    prev = current;
  }

  if (espSerial.available() > 0)
  {
    String sensorData = espSerial.readStringUntil('\n');
    Serial.println(sensorData);
    Serial.println("Done!");
  }
}
