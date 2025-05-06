#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// Define pins for SoftwareSerial
const byte ESP_RX_PIN = 2;
const byte ESP_TX_PIN = 3;

// Define baud rates
const long SERIAL_BAUD_RATE = 9600;
const long ESP_SERIAL_BAUD_RATE = 9600;

unsigned long prevMillis = 0; // Renamed for clarity, standard practice
const unsigned long interval = 5000; // Interval for sending data

// Flag to alternate data types
bool sendGpsDataNext = true;

// Sample RFID tag
const char* SAMPLE_RFID_TAG = "123ABC456";

SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN); // RX, TX

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  espSerial.begin(ESP_SERIAL_BAUD_RATE);
  Serial.println("Arduino setup complete. Waiting for ESP...");
  delay(2000); // Allow time for ESP to initialize if needed
}

void loop() {
  unsigned long currentMillis = millis(); // Get current time

  // Send data to ESP32 every 'interval' milliseconds
  if (currentMillis - prevMillis >= interval) {
    prevMillis = currentMillis; // Save the last time data was sent

    StaticJsonDocument<200> doc; // JSON document buffer

    if (sendGpsDataNext) {
      // Populate GPS JSON document
      doc["sensor"] = "gps";
      doc["time"] = 1351824120; // Example timestamp
      JsonArray dataArray = doc.createNestedArray("data");
      dataArray.add(48.7); // Example latitude
      dataArray.add(2.3);  // Example longitude
      Serial.print("Sending GPS JSON to ESP: ");
    } else {
      // Populate RFID JSON document
      doc["rfid_tag"] = SAMPLE_RFID_TAG;
      Serial.print("Sending RFID JSON to ESP: ");
    }

    // Print to local monitor for debugging
    serializeJsonPretty(doc, Serial);
    Serial.println();

    // Send JSON to ESP32 via SoftwareSerial
    if (serializeJson(doc, espSerial) == 0) {
      Serial.println(F("Failed to write JSON to espSerial"));
    } else {
      espSerial.println(); // Send newline as a delimiter
      Serial.println(F("JSON sent to ESP."));
    }

    // Toggle the flag for the next iteration
    sendGpsDataNext = !sendGpsDataNext;
  }

  // Check for incoming data from ESP32
  if (espSerial.available() > 0) {
    String receivedData = espSerial.readStringUntil('\n');
    receivedData.trim(); // Remove any leading/trailing whitespace

    Serial.print("Received from ESP: [");
    Serial.print(receivedData);
    Serial.println("]");
  }
}
