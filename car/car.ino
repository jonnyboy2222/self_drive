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
#define RFID_SS_PIN         53   // SDA
#define RFID_RST_PIN        49

// Alcohol Sensor
#define ALCOHOL_SENSOR_PIN  A8
#define SWITCH_PIN          48

// Ambient Light
#define LIGHT_SENSOR_PIN    A9
#define LED_PIN             47

// Temperature
#define TEMP_SENSOR_PIN     A0

// Ultrasonic & Buzzer
#define TRIG_PIN            46
#define ECHO_PIN            45
#define BUZZER_PIN          44

//shock
#define SHOCK_SENSOR_PIN    43
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

static int idx = 0;
static char recv_buffer[4];

static int idx2 = 0;
static char recv_buffer2[3];


const byte PACKET_HEADER = 0xAA;

const char CMD_STORE_DATA[] = "DB";
const int DB_PACKET_SIZE = 1 + 2 + 4 + sizeof(float) * 2; // 1(header) + 2(command) + 4(uid) + 8(floats)

const char CMD_LIGHT_STATE[] = "LS";
const int LS_PACKET_SIZE = 1 + 2 + 1; // 1(header) + 2(command) + 1(boolean)

const char CMD_ULTRA_SONIC[] = "UR";
const int UR_PACKET_SIZE = 1 + 2 + 4; // 1(header) + 2(command) + 4(float)

bool lightState;
float ultrasonic;

float shockVal = 0;
float tempVal = 0;

bool previousLightState = false;  // 이전 상태 저장


void setup() {
  Serial.begin(BAUD_RATE);
  SPI.begin();

  authManager.begin();
  ambientLightManager.begin();
  obstacleManager.begin();
  tempManager.begin();
  shockManager.begin();
  rfidManager.begin();
}

void loop() {
  currentTime = millis();
  authManager.update();
  ambientLightManager.update();
  obstacleManager.update();
  tempManager.update();
  shockManager.update();
  rfidManager.update();
  
  if ( currentTime - lastSendTime >= SendInterval)
  {
    lastSendTime = currentTime;

    char send_buffer_db[DB_PACKET_SIZE];

    shockVal = shockManager.getLatestAverageShock();
    tempVal = tempManager.getCurrentTemperature();

    send_buffer_db[0] = PACKET_HEADER;
    memcpy(send_buffer_db + 1, CMD_STORE_DATA, 2);
    memcpy(send_buffer_db + 3, rfidManager.getUIDBytes(), 4);
    memcpy(send_buffer_db + 7, &shockVal, sizeof(float));
    memcpy(send_buffer_db + 11, &tempVal, sizeof(float));

    Serial.write((const uint8_t*)send_buffer_db, DB_PACKET_SIZE);
  }
  // 수신 처리

  lightState = ambientLightManager.getLightState();
  if (lightState != previousLightState) {
    previousLightState = lightState;  // 상태 업데이트

    char send_buffer_ls[LS_PACKET_SIZE];
    send_buffer_ls[0] = PACKET_HEADER;
    memcpy(send_buffer_ls + 1, CMD_LIGHT_STATE, 2);
    memcpy(send_buffer_ls + 3, &lightState, 1);

    Serial.write((const uint8_t*)send_buffer_ls, LS_PACKET_SIZE);
  }

  // ultrasonic = obstacleManager.getAvgDistance();
  // if ()


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
      } else if (recv_buffer[1] == 'M' && recv_buffer[2] == 'B') {
        obstacleManager.setReversing(true);
        ultrasonic = obstacleManager.getAvgDistance();
        char send_buffer_ur[UR_PACKET_SIZE];
        send_buffer_ur[0] = PACKET_HEADER;
        memcpy(send_buffer_ur + 1, CMD_ULTRA_SONIC, 2);
        memcpy(send_buffer_ur + 3, &ultrasonic, 4);
        Serial.write((const uint8_t*)send_buffer_ur, UR_PACKET_SIZE);
      } else if (recv_buffer[1] == 'S' && recv_buffer[2] == 'T') {
        obstacleManager.setReversing(false);
      }
      idx = 0;
    }
  }
}
