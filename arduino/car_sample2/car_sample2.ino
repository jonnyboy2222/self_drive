#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

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
class AlcoholManager
{
  private:
    const int THRESHOLD = 200;
    const unsigned long MEASUREDURATION = 5000;
    const unsigned long SAMPLEINTERVAL = 100;

    bool measuring = false;
    unsigned long startTime = 0;
    unsigned long lastSampleTime = 0;
    unsigned long sum = 0;
    int count = 0;

  public:
    void begin()
    {
      pinMode(ALCOHOL_SENSOR_PIN, INPUT);
      pinMode(SWITCH_PIN, INPUT_PULLUP);
    }

    bool isSwitchPressed()
    {
      static bool lastSwitch = HIGH;
      bool current = digitalRead(SWITCH_PIN);
      bool pressed = (lastSwitch == HIGH && current == LOW);
      lastSwitch = current;
      return pressed;
    }

    void startMeasuring()
    {
      measuring = true;
      startTime = millis();
      lastSampleTime = 0;
      sum = 0;
      count = 0;
    }

    bool isMeasuring()
    {
      return measuring;
    }

    bool update()
    {
      if (!measuring) return false;

      unsigned long now = millis();
      if (now - lastSampleTime >= SAMPLEINTERVAL)
      {
        lastSampleTime = now;
        int val = analogRead(ALCOHOL_SENSOR_PIN);
        sum += val;
        count++;
      }

      if (now - startTime >= MEASUREDURATION)
      {
        measuring = false;
        float avg = (float)sum / count;
        Serial.print("Average: ");
        Serial.println(avg);
        if (avg < THRESHOLD)
        {
          return true;
        }
        else
        {
          return false;
        }
        
      }

      return false; // 아직 측정 중
    }
};

// === Obstacle Alert Manager (Ultrasonic + Buzzer) ===
class ObstacleManager
{
  private:

    //후진 상태 체크 관련 변수
    bool isbacking = false;

    //초음파 관련 변수
    const static int SAMPLESIZE = 10;                // num==10
    float samples[SAMPLESIZE] = {0};         // 10개의 칸을 가진 float 배열(samples)이 되고, 모두 0으로 시작!
    int sampleindex = 0;
    float total_distance = 0;
    float avg_distance = 0;                  // 평균
    unsigned long pre_measuretime = 0;       //이전 거리측정 시간
    unsigned long checktime_interval = 15;

    //수동부저 관련 변수
    unsigned long prebeeptime = 0;
    bool buzzerstate = false;
    int beepfreq = 0;
    int beepinterval = 0;

  public:

    void begin()
    {
      pinMode(ULTRASONIC_TRIG, OUTPUT);
      pinMode(ULTRASONIC_ECHO, INPUT);
      pinMode(BUZZER_PIN, OUTPUT);
    }

    ///////////////////////////////////////////////////////////////////////////////후진 중? 체크 함수////////////////////////////////////////////////////////////
    // void isBackState()
    // {
    //   current_buttonstate = digitalRead(BUTTON);
    //   if (pre_buttonstate == LOW && current_buttonstate == HIGH)
    //   {
    //     isbacking = !isbacking;
    //   }
    // }
    void setReversing(bool state)
    {
      isbacking = state;
    }

/////////////////////////////////////////////////////////////////////////////초음파 평균값거리 측정함수////////////////////////////////////////////////////////
    void avgDistance()
    {
      unsigned long nowtime = millis();

      if (nowtime - pre_measuretime >= checktime_interval)
      {
        //센서 이용해서 거리 구하기
        digitalWrite(ULTRASONIC_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG, LOW);

        unsigned long duration = pulseIn(ECHO, HIGH, 20000);      //부딪혔다가 돌아오는 시간(20ms초과하면 0반환)
        float distance = duration * 0.034 / 2;                    //cm로 환산

        if (distance>0 && distance<340)                    //이상값 제한(pulseIn(ECHO, HIGH, 20000)로 측정가능한게 340cm까지)
        {
          total_distance -= samples[sampleindex];
          samples[sampleindex] = distance;
          total_distance += samples[sampleindex];                      // total_distance 구하기
          sampleindex = (sampleindex+1) % SAMPLESIZE ;                 // samples 안에 distance 10개 채우는 과정
        }
        pre_measuretime = nowtime;
    
        // avg_distance 구하기
        if (sampleindex == 0)
        {
          avg_distance = total_distance / SAMPLESIZE;
          Serial.print("AVG_DISTANCE: ");
          Serial.println(avg_distance);
        }
      }     
    }
////////////////////////////////////////////////beepfreq 결정함수///////////////////////////////////////////////////////////////
    void setBeepfreqByDistance()
    {
      if (avg_distance <= 10) 
      {
        beepfreq = 2000; 
        beepinterval = 30;
      } 
      else if (avg_distance <= 20) 
      {
        beepfreq = 1500; 
        beepinterval = 50;
      } 
      else if (avg_distance <= 40) 
      {
        beepfreq = 1000; 
        beepinterval = 100;
      } 
      else if (avg_distance <= 70) 
      {
        beepfreq = 700;  
        beepinterval = 200;
      } 
      else if (avg_distance <= 100) 
      {
        beepfreq = 400;  
        beepinterval = 250;
      } 
      else 
      {
        beepfreq = 0;    
        beepinterval = 0;
        noTone(BUZZER_PIN);
      }
    }

////////////////////////////////////////////////////////////////////수동부저 제어 함수///////////////////////////////////////////////////////////////
    void controlBuzzer()
    {
      unsigned long nowtime = millis();
      if (beepfreq>0 && nowtime - prebeeptime >= beepinterval)
      {
        prebeeptime = nowtime;
        if (buzzerstate)
        {
          noTone(BUZZER_PIN);
          buzzerstate = false;
        }
        else
        {
          tone(BUZZER_PIN, beepfreq);
          buzzerstate = true;
        }
      }
    }

///////////////////////////////////////////////////////////////
    void update()
    {
      if(isbacking)
      {
        avgDistance();
        setBeepfreqByDistance();
        controlBuzzer();
      }

      else 
      {
        noTone(BUZZER_PIN);
      }
    }
};

// === Shock Sensor Manager ===
class ShockManager 
{
  private:
    uint8_t pin;
    unsigned long lastWindowTime;
    const unsigned long windowDuration = 200; // 0.2초
    int buffer[5];
    int bufferIndex;
    int countInWindow;
    bool lastSensorState;
    float latest_average_shock = 0.0;

  public:
    ShockManager(uint8_t sensorPin) : pin(sensorPin), lastWindowTime(0), bufferIndex(0), countInWindow(0), lastSensorState(LOW) {
      for (int i = 0; i < 5; i++) buffer[i] = 0;
    }

    void begin() 
    {
      pinMode(pin, INPUT);
      lastWindowTime = millis();
    }

    void update() 
    {
      bool currentSensorState = digitalRead(pin);

      // 상승 에지 감지 (LOW -> HIGH)
      if (currentSensorState == HIGH && lastSensorState == LOW) 
      {
        countInWindow++;
      }
      lastSensorState = currentSensorState;

      unsigned long currentTime = millis();
      if (currentTime - lastWindowTime >= windowDuration) 
      {
        buffer[bufferIndex] = countInWindow;
        bufferIndex = (bufferIndex + 1) % 5;

        // 1초 주기로 평균 계산 및 전송
        if (bufferIndex == 0) 
        {
          int sum = 0;
          for (int i = 0; i < 5; i++) 
          {
            sum += buffer[i];
          }
          latest_average_shock = sum / 5.0; // Update the internal average
          // Serial.print("ShockManager: Updated average to "); 
          // Serial.println(latest_average_shock); // Debug
        }

        countInWindow = 0;
        lastWindowTime = currentTime;
      }
    }

    float getLatestAverageShock() { // Getter for the latest average
      return latest_average_shock;
    }
};

// === Temperature Manager ===
class TempManager
{
  private:
    const float TEMP_THRESHOLD = 37.0;
    unsigned long pre_time = 0;
    const unsigned long INTERVAL = 1000;
    float temperature = 0;
    float overtemperature = 0;

  public:

    void begin() 
    {
    }

    void measure_Temperature()
    {
      int adc_value = analogRead(TEMP_SENSEOR_PIN);
      float voltage = adc_value * (5.0 / 1024.0);
      temperature = voltage * 100;
      // Serial.print("temperature : ");
      // Serial.println(temperature);
    }

    float getTemp()
    {
      return temperature;
    }


    float getoverTemp()
    {
      return overtemperature ;
    }

    void update()
    {
      unsigned long now_time = millis();

      if (now_time - pre_time >= INTERVAL) 
      {
        pre_time = now_time;
        measure_Temperature();
      }
    }
};

// === Ambient Light Manager ===
class AmbientLightManager 
{
  private:
    const static int LIGHTSAMPLESIZE = 10;
    int light_samples[LIGHTSAMPLESIZE] = {0};

    int light_sample_index = 0;
    int total_lights = 0;
    float avg_light = 0;
    unsigned long now_light_measuretime = 0;
    unsigned long pre_light_measuretime = 0;
    unsigned long light_measuretime_interval = 50;

    const int LIGHT_THRESHOLD = 140;

  public:
    void begin() 
    {
      pinMode(HEADLIGHT_LED_PIN, OUTPUT);
      pinMode(LIGHT_SENSOR_PIN, INPUT);

      digitalWrite(HEADLIGHT_LED_PIN, LOW);
    }

    void result_avg_Light() 
    {
      int light = analogRead(LIGHT_SENSOR_PIN);
      int map_light = map(constrain(light, 50, 1020), 50, 1020, 255, 0);

      total_lights -= light_samples[light_sample_index];
      light_samples[light_sample_index] = map_light;
      total_lights += light_samples[light_sample_index];

      light_sample_index = (light_sample_index + 1) % LIGHTSAMPLESIZE;

      if (light_sample_index == 0) 
      {
        avg_light = total_lights / LIGHTSAMPLESIZE;
      }
    }

    void led_state_byThreshold() 
    {
      if (avg_light > LIGHT_THRESHOLD) 
      {
        digitalWrite(HEADLIGHT_LED_PIN, HIGH);
      }
      else 
      {
        digitalWrite(HEADLIGHT_LED_PIN, LOW);
      }
    }

    void update() 
    {
      now_light_measuretime = millis();
      if (now_light_measuretime - pre_light_measuretime >= light_measuretime_interval) 
      {
        result_avg_Light();
        led_state_byThreshold();
        pre_light_measuretime = now_light_measuretime;
        Serial.println(avg_light);
      }
    }
};


// === RFID Manager ===
class RFIDManager 
{
  private:
    MFRC522 mfrc;
    bool isCardPresent = false;
    bool wasCardPresent = false;
    unsigned long lastSeen = 0;
    String currentUID = "";
    StaticJsonDocument<200> doc;
    SoftwareSerial &espSerial;
  public:
    
    RFIDManager(byte ssPin, byte rstPin, SoftwareSerial &esp) : mfrc(ssPin, rstPin),espSerial(esp) {}

    void begin() {
      mfrc.PCD_Init();
    }

    void update() {
      byte bufferATQA[2];
      byte bufferSize = sizeof(bufferATQA);
      //카드가 리더 범위에 들어왔는지 판단
      MFRC522::StatusCode status = mfrc.PICC_RequestA(bufferATQA, &bufferSize);
      //카드가 통신되면 lastSeen 시간 저장, 카드 존재 true 저장.
      if (status == MFRC522::STATUS_OK) 
      {
        lastSeen = millis();
        isCardPresent = true;
      } 
      //카드가 통신안되면
      else 
      { //카드가 존재했었고, 없어진 시간이 debounce 보다 크면 카드 존재 false저장.
        if (isCardPresent && millis() - lastSeen > RFID_DEBOUNCE_TIME) 
        {
          isCardPresent = false;
        }
      }
      //카드가 처음 통신되었으면 UID 읽어서 Serial로 전송
      if (isCardPresent && !wasCardPresent) 
      {
        if (mfrc.PICC_ReadCardSerial()) {
          currentUID = getUIDStringFromReader(); // Use a distinct name for the method that reads from mfrc.uid
          doc.clear(); // Clear previous data
          doc["purpose"] = "verification";
          doc["rfid_uid"] = currentUID;
          
          // Serial.print("Card detected, UID: " + currentUID + ". Sending to ESP for verification: ");
          // serializeJson(doc, Serial); // Debug print
          // Serial.println();

          serializeJson(doc, espSerial); // Send to ESP32
          espSerial.println(); // Ensure newline for parsing on ESP side
          
          mfrc.PICC_HaltA();        // ★ 카드 통신 종료
          mfrc.PCD_StopCrypto1();   // ★ 암호화 종료
        } 
        else 
        {
          Serial.println("card detected, but UID read failed");
          currentUID = "";
        }
        wasCardPresent = true;
      }
      //카드가 처음 통신 안되면 UID 초기화
      if (!isCardPresent && wasCardPresent) 
      {
        Serial.println("card removed");
        wasCardPresent = false;
        currentUID = "";
      }
    }
    //UID 카드에서 읽어서 저장
    String getUIDString() 
    {
      String uid = "";
      for (byte i = 0; i < mfrc.uid.size; i++) 
      {
        if (mfrc.uid.uidByte[i] < 0x10) uid += "0";
        uid += String(mfrc.uid.uidByte[i], HEX);
      }
      uid.toUpperCase();
      return uid;
    }

    String getActiveUID() { // Getter for the current UID, used for sensor bundle
      return currentUID;
    }
};

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
LCDManager lcdManager(LCD_RS_PIN,LCD_EN_PIN,LCD_D4_PIN,LCD_D5_PIN,LCD_D6_PIN,LCD_D7_PIN);
AlcoholManager alcoholManager;
DriveManager driveManager;
RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN, espSerial);
ESPManager espManager(espSerial);
BluetoothManager bluetoothManager(btSerial);
ObstacleManager obstacleManager;
ShockManager shockManager(SHOCK_SENSOR_PIN);
TempManager tempManager;
AmbientLightManager ambientLightManager;
SystemManager systemManager(lcdManager, alcoholManager, driveManager, rfidManager,
                            espManager, bluetoothManager, obstacleManager,
                            shockManager, tempManager, ambientLightManager);

// For periodic sensor data bundle sending
static unsigned long lastSensorBundleSendTime = 0;
const unsigned long sensorBundleInterval = 1000; // Send every 1 second

void setup() 
{
  Serial.begin(BAUD_RATE);
  // espSerial.begin(ESP_BAUD_RATE);
  // btSerial.begin(BT_BAUD_RATE);
  SPI.begin();
  lcdManager.begin();
  driveManager.begin();
  rfidManager.begin();
  obstacleManager.begin();
  shockManager.begin();
  tempManager.begin();
  ambientLightManager.begin();

  lcdManager.printLine(0, "System Ready");
  lcdManager.printLine(1, "Scan RFID Card");
  // currentState is already WAIT_FOR_AUTH by default
}


void loop() 
{
  rfidManager.update();
  shockManager.update(); // Handles shock detection and calculates average internally
  tempManager.update(); // Handles temperature reading internally
  float temp = tempManager.getTemp();
  float overtemp = tempManager.getoverTemp();
  ambientLightManager.update(); // Handles light sensor and headlights
  
  // Handle incoming data from ESP32 (RFID verification results, YOLO commands)
  String espData = espManager.getResponse();
  if (espData != "") {
    Serial.print("Received from ESP: "); Serial.println(espData);
    if (espData == "PASS" || espData == "FAIL") {
      systemManager.handleEspResponse(espData);
    } else {
      Serial.println("Unknown command from ESP: " + espData);
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
