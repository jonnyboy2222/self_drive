#include <SPI.h>
#include <MFRC522.h>
#include "RFIDManager.h"
#include "AlcoholManager.h"
#include "AuthManager.h"
#include "AmbientLightManager.h"
#include "ObstacleManager.h"
#include "TempManager.h"

#define BAUD_RATE 9600

// RFID
#define RFID_RST_PIN 9
#define RFID_SS_PIN 10

// Alcohol
#define ALCOHOL_SENSOR_PIN A2
#define SWITCH_PIN 8

// Ambient Light
#define LIGHT_SENSOR_PIN A3
#define LED_PIN 12

// Obstacle (Ultrasonic + Buzzer)
#define TRIG_PIN 28
#define ECHO_PIN 29
#define BUZZER_PIN 11

// Temperature
#define TEMP_SENSOR_PIN A1

// Manager Instances
RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN);
AlcoholManager alcoholManager(ALCOHOL_SENSOR_PIN, SWITCH_PIN);
AuthManager authManager(rfidManager, alcoholManager);

AmbientLightManager ambientLightManager(LIGHT_SENSOR_PIN, LED_PIN);
ObstacleManager obstacleManager(TRIG_PIN, ECHO_PIN, BUZZER_PIN);
TempManager tempManager(TEMP_SENSOR_PIN);

void setup() {
  Serial.begin(BAUD_RATE);
  SPI.begin();

  authManager.begin();
  ambientLightManager.begin();
  obstacleManager.begin();
  tempManager.begin();
}

void loop() {
  authManager.update();
  ambientLightManager.update();
  obstacleManager.update();
  tempManager.update();

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
