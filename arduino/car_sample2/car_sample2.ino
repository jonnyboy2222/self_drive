#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

#include "AlcoholManager.h"
#include "ObstacleManger.h"
#include "ShockManger.h"
#include "TempManager.h"
#include "AmbientLightManager.h"
#include "RFIDManager.h"


#define BAUD_RATE 9600

// === 핀 정의 ===

// 센서

#define SHOCK_SENSOR_PIN 26

#define ULTRASONIC_TRIG  28
#define ULTRASONIC_ECHO  29
#define MAX_DISTANCE_CM  150
#define MIN_DISTANCE_CM  10

#define TEMP_SENSOR_PIN  A0

#define ALCOHOL_SENSOR_PIN A1
#define SWITCH_PIN 36

#define LIGHT_SENSOR_PIN A2

// 출력

#define BUZZER_PIN 11

#define HEADLIGHT_LED_PIN 13
#define LIGHT_THRESHOLD  300

// RFID
#define RFID_RST_PIN 44
#define RFID_SS_PIN 10
#define RFID_DEBOUNCE_TIME 1000

// === Alcohol Sensor Manager ===
AlcoholManager alcoholM;


// === Obstacle Alert Manager (Ultrasonic + Buzzer) ===
ObstacleManager obstacleM;

// === Shock Sensor Manager ===
ShockManager shockM(SHOCK_SENSOR_PIN);

// === Temperature Manager ===
TemperatureManager tempM;

// === Ambient Light Manager ===
AmbientLightManager ambientM;

// === RFID Manager ===
RRFIDManager rfidM(RFID_SS_PIN, RFID_RST_PIN);

enum DriveState { WAIT_FOR_AUTH, MEASURING, ACCESS_GRANTED, ACCESS_DENIED };

// === System Manager ===
class SystemManager 
{
  private:
    LCDManager &lcd;
    AlcoholManager &alcohol;
    DriveManager &drive;
    RFIDManager &rfid;
    ESPManager &esp;
    BluetoothManager &bt;
    ObstacleManager &obstacle;
    ShockManager &shock;
    TempManager &temp;
    AmbientLightManager &ambient;

    DriveState currentState = WAIT_FOR_AUTH;
  public:
    SystemManager(LCDManager &l, AlcoholManager &a, DriveManager &d, RFIDManager &r,
                  ESPManager &e, BluetoothManager &b, ObstacleManager &o, ShockManager &s,
                  TempManager &t, AmbientLightManager &am)
      : lcd(l), alcohol(a), drive(d), rfid(r), esp(e), bt(b), obstacle(o), shock(s), temp(t), ambient(am) {}

    void handleResponse(const String &cmd)
    {
      if (cmd == "PASS" && currentState == WAIT_FOR_AUTH)
      {
        Serial.println("MEASURING");
        lcd.printLine(0, "MEASURING");
        currentState = MEASURING;
      }
      else if (cmd == "FAIL")
      {
        Serial.println("ACCESS DENIED");
        lcd.printLine(0, "ACCESS DENIED");
        drive.stopMotors();
        currentState = ACCESS_DENIED;
      }
    }

    void update()
    {
      if (currentState == MEASURING)
      {
        if (!alcohol.isMeasuring())
        {
          if (alcohol.isSwitchPressed())
          {
            lcd.printLine(0, "Measuring...");
            Serial.println("Measuring Start");
            alcohol.startMeasuring();
          }
        }
        else
        {
          bool isSafe = alcohol.update(); // true면 정상

          // 측정이 끝났을 경우
          if (!alcohol.isMeasuring())
          {
            if (isSafe)
            {
              lcd.printLine(0, "ACCESS GRANTED");
              Serial.println("ACCESS GRANTED");
              currentState = ACCESS_GRANTED;
            }
            else
            {
              lcd.printLine(0, "ACCESS DENIED");
              Serial.println("ACCESS DENIED");
              drive.stopMotors();
              currentState = ACCESS_DENIED;
            }
          }
        }
      }
      else if (currentState == ACCESS_GRANTED)
      {
        obstacle.update();
        shock.update();
        temp.update();
        ambient.update();
      }
    }

    void handleDriveCommand(const String &cmd) 
    {
      if (cmd.startsWith("X")) 
      {
        int angle = cmd.substring(1).toInt();
        steering.write(angle);
      } 
      else if (cmd.length() > 0) 
      {
        char action = cmd.charAt(0);
        switch (action) 
        {
          case 'F': 
            drive.moveForward(); 
            obstacle.setReversing(false);
            break;
          case 'B': 
            drive.moveBackward(); 
            obstacle.setReversing(true);
            break;
          case 'S': 
            drive.stopMotors(); 
            obstacle.setReversing(false);
            break;
        }
      }
    }
};

// === 인스턴스 생성 ===
// AlcoholManager alcoholManager;
// RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN, espSerial);
// ObstacleManager obstacleManager;
// ShockManager shockManager(SHOCK_SENSOR_PIN);
// TempManager tempManager;
// AmbientLightManager ambientLightManager;

SystemManager systemManager(lcdManager, alcoholManager, driveManager, rfidManager,
                            espManager, bluetoothManager, obstacleManager,
                            shockManager, tempManager, ambientLightManager);

// For periodic sensor data bundle sending
static unsigned long lastSensorBundleSendTime = 0;
const unsigned long sensorBundleInterval = 1000; // Send every 1 second

void setup() 
{
  Serial.begin(BAUD_RATE);
  SPI.begin();
  rfidManager.begin();
  obstacleM.begin();
  shockM.begin();
  tempM.begin();
  ambientM.begin();
}


void loop() 
{
  rfidM.update();
  shockM.update(); // Handles shock detection and calculates average internally
  tempM.update(); // Handles temperature reading internally
  float temp = tempM.getTemp();
  float overtemp = tempM.getoverTemp();
  ambientM.update(); // Handles light sensor and headlights
  
  // Handle incoming data from ESP32 (RFID verification results, YOLO commands)
  String serData = Serial.read();
  if (espData != "") {
    Serial.print("Received from Serial: "); Serial.println(serData);
    if (serData == "PASS" || serData == "FAIL") {
      systemManager.handleEspResponse(serData);
    } else {
      Serial.println("Unknown command from Serial: " + serData);
    }
  }

  // Check for communication (manual control commands)
  if (Serial.available()) {
    String Cmd = Serial.read();
    Serial.println(Cmd);
    
  }

  // Periodically send combined sensor data
  unsigned long currentTime = millis();
  if (currentTime - lastSensorBundleSendTime >= sensorBundleInterval) {
    lastSensorBundleSendTime = currentTime;

    String currentRfidUid = rfidManager.getActiveUID();
    float currentShock = shockManager.getLatestAverageShock();
    float currentTemp = tempManager.getCurrentTemperature();

    StaticJsonDocument<200> sensorDoc;
    sensorDoc["purpose"] = "sensor_db"; 
    sensorDoc["rfid_uid"] = currentRfidUid.isEmpty() ? nullptr : currentRfidUid.c_str(); // Send null if UID is empty, else send UID
    sensorDoc["shock"] = currentShock;
    sensorDoc["temperature"] = currentTemp;

    Serial.print("Sending sensor bundle to ESP: ");
    serializeJson(sensorDoc, Serial); // Debug print
    Serial.println();
  }

  systemManager.update();
}
