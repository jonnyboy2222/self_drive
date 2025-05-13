#include <SPI.h>
#include "RFIDManager.h"
#include "AlcoholManager.h"
#include "AuthManager.h"
#include "AmbientLightManager.h"
#include "ObstacleManager.h"
#include "TempManager.h"
#include "ShockManager.h"
#define BAUD_RATE 9600

// RFID (SPI: 50=MISO, 51=MOSI, 52=SCK)
#define RFID_SS_PIN     53   // SDA
#define RFID_RST_PIN    49

// Alcohol Sensor
#define ALCOHOL_SENSOR_PIN  A8
#define SWITCH_PIN          48

// Ambient Light
#define LIGHT_SENSOR_PIN    A9
#define LED_PIN             47

// Temperature
#define TEMP_SENSOR_PIN     A10

// Ultrasonic & Buzzer
#define TRIG_PIN            46
#define ECHO_PIN            45
#define BUZZER_PIN          44

//shock
#define SHOCK_SENSOR_PIN 43
// Manager Instances
RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN);
AlcoholManager alcoholManager(ALCOHOL_SENSOR_PIN, SWITCH_PIN);
AuthManager authManager(rfidManager, alcoholManager);

AmbientLightManager ambientLightManager(LIGHT_SENSOR_PIN, LED_PIN);
ObstacleManager obstacleManager(TRIG_PIN, ECHO_PIN, BUZZER_PIN);
TempManager tempManager(TEMP_SENSOR_PIN);
ShockManager shockManager(SHOCK_SENSOR_PIN);

unsigned long lastSendTime =0;
unsigned long currentTime;
unsigned long SendInterval = 1000;
void setup() {
  Serial.begin(BAUD_RATE);
  SPI.begin();

  authManager.begin();
  ambientLightManager.begin();
  obstacleManager.begin();
  tempManager.begin();
  shockManager.begin();
}

void loop() {
  currentTime = millis();
  authManager.update();
  ambientLightManager.update();
  obstacleManager.update();
  tempManager.update();
  shockManager.update();
  
  if ( currentTime - lastSendTime >= SendInterval)
  {
    lastSendTime = currentTime;
    Serial.println(ambientLightManager.getLightState());
    Serial.println(obstacleManager.getAvgDistance());
    Serial.println(tempManager.getCurrentTemperature());
    Serial.println(shockManager.getLatestAverageShock());
  }
  // 수신 처리
  static int idx = 0;
  static char recv_buffer[4];

  while (Serial.available()) {
    char byte = Serial.read();

    if (idx == 0 && (uint8_t)byte != 0xAA) continue;

    recv_buffer[idx++] = byte;

    if (idx == 4) {
      if (recv_buffer[1] == 'V' && recv_buffer[2] == 'F') {
        if ((uint8_t)recv_buffer[3] == 1) {
          authManager.handleResponse(true);
        } else if ((uint8_t)recv_buffer[3] == 0) {
          authManager.handleResponse(false);
        }
      }
      idx = 0;
    }
  }
}
