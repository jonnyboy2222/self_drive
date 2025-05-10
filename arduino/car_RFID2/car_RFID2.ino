#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

#define BAUD_RATE 9600
#define ESP_BAUD_RATE 9600

#define LCD_RS_PIN 2
#define LCD_EN_PIN 3
#define LCD_D4_PIN 4
#define LCD_D5_PIN 5
#define LCD_D6_PIN 6
#define LCD_D7_PIN 7

#define RFID_RST_PIN 9
#define RFID_SS_PIN 10
#define RFID_DEBOUNCE_TIME 500

#define ESP_RX_PIN A0
#define ESP_TX_PIN A1

#define ALCOHOL_SENSOR_PIN A2
#define SWITCH_PIN 8

#define MOTOR_L_IN1 4
#define MOTOR_L_IN2 5
#define MOTOR_R_IN1 6
#define MOTOR_R_IN2 7

enum DriveState {
    WAIT_FOR_AUTH,
    MEASURING,
    ACCESS_GRANTED,
    ACCESS_DENIED
};

// === LCD Manager ===
class LCDManager
{
  private:
    LiquidCrystal lcd;

  public:
    LCDManager(uint8_t rs, uint8_t en, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
      : lcd(rs, en, d4, d5, d6, d7)
    {
    }

    void begin()
    {
      lcd.begin(16, 2);
      lcd.clear();
      lcd.print("Init");
    }

    void printLine(int row, const String &text)
    {
      lcd.setCursor(0, row);
      lcd.print("                ");
      lcd.setCursor(0, row);
      lcd.print(text);
    }

    void clear()
    {
      lcd.clear();
    }
};

// === RFID Manager ===
class RFIDManager {
private:
  MFRC522 mfrc;
  bool isCardPresent = false;
  bool wasCardPresent = false;
  unsigned long lastSeen = 0;
  String UID = ""; 
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
    if (isCardPresent && !wasCardPresent) {
      if (mfrc.PICC_ReadCardSerial()) {
        UID = getUIDString();  // UID 멤버 변수에 저장
        doc["rfid_tag"] = UID;
        Serial.println("card detected");
        serializeJson(doc,espSerial);
        serializeJson(doc,Serial);
        Serial.println("");
        mfrc.PICC_HaltA();        // ★ 카드 통신 종료
        mfrc.PCD_StopCrypto1();   // ★ 암호화 종료
      } else {
        Serial.println("card detected, but UID read failed");
        UID = "";
      }
      wasCardPresent = true;
    }
    //카드가 처음 통신 안되면 UID 초기화
    if (!isCardPresent && wasCardPresent) {
      Serial.println("card removed");
      wasCardPresent = false;
      UID = "";
    }
  }
  //UID 카드에서 읽어서 저장
  String getUIDString() {
    String uid = "";
    for (byte i = 0; i < mfrc.uid.size; i++) {
      if (mfrc.uid.uidByte[i] < 0x10) uid += "0";
      uid += String(mfrc.uid.uidByte[i], HEX);
    }
    uid.toUpperCase();
    return uid;
  }
};

// === ESP32 Manager ===
class ESPManager
{
  private:
    SoftwareSerial &esp;

  public:
    ESPManager(SoftwareSerial &serial) : esp(serial)
    {
    }

    void sendUID(const String &uid)
    {
      esp.println(uid);
    }

    String getResponse()
    {
      if (esp.available())
      {
        String res = esp.readStringUntil('\n');
        res.trim();
        return res;
      }
      return "";
    }
};
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


// === Drive Motor Manager ===
class DriveManager
{
  private:
    bool reversingFlag = false;
  public:
    bool isReversing()
    {
      return reversingFlag;
    }
    void begin()
    {
      pinMode(MOTOR_L_IN1, OUTPUT);
      pinMode(MOTOR_L_IN2, OUTPUT);
      pinMode(MOTOR_R_IN1, OUTPUT);
      pinMode(MOTOR_R_IN2, OUTPUT);
      stop();
    }

    void forward()
    {
      reversingFlag = false;
      digitalWrite(MOTOR_L_IN1, HIGH);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, HIGH);
      digitalWrite(MOTOR_R_IN2, LOW);
    }

    void backward()
    {
      reversingFlag = true;
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, HIGH);
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, HIGH);
    }

    void left()
    {
      reversingFlag = false;
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, HIGH);
      digitalWrite(MOTOR_R_IN2, LOW);
    }

    void right()
    {
      reversingFlag = false;
      digitalWrite(MOTOR_L_IN1, HIGH);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, LOW);
    }

    void stop()
    {
      reversingFlag = false;
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, LOW);
    }
};

// === System Manager ===
class SystemManager
{
  private:
    LCDManager &lcd;
    AlcoholManager &alcohol;
    DriveManager &drive;
    DriveState currentState = WAIT_FOR_AUTH;
  public:
    SystemManager(LCDManager &l, AlcoholManager &a, DriveManager &d)
      : lcd(l), alcohol(a), drive(d)
    {
    }

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
        drive.stop();
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
              drive.stop();
              currentState = ACCESS_DENIED;
            }
          }
        }
      }
    }


    void handleDriveCommand(const String &cmd)
    {
      if (currentState != ACCESS_GRANTED)
      {
        lcd.printLine(0, "CMD BLOCKED");
        return;
      }

      if (cmd == "전진")
        drive.forward();
      else if (cmd == "후진")
        drive.backward();
      else if (cmd == "좌회전")
        drive.left();
      else if (cmd == "우회전")
        drive.right();
      else if (cmd == "정지")
        drive.stop();
      else
        lcd.printLine(1, "BT: UNKNOWN");
    }
};

LCDManager lcdManager(LCD_RS_PIN,LCD_EN_PIN,LCD_D4_PIN,LCD_D5_PIN,LCD_D6_PIN,LCD_D7_PIN);
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);
ESPManager espManager(espSerial);
RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN, espSerial);
DriveManager driveManager;
AlcoholManager alcoholManager;
SystemManager systemManager(lcdManager, alcoholManager, driveManager);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  espSerial.begin(ESP_BAUD_RATE);
  SPI.begin();

  driveManager.begin();
  lcdManager.begin();
  rfidManager.begin();
  alcoholManager.begin(); 

  Serial.println("Initialized");

}

void loop() {
  // put your main code here, to run repeatedly:
  rfidManager.update();
  String espResponse = espManager.getResponse();
  if(espResponse != "")
  {
    systemManager.handleResponse(espResponse);
  }
  systemManager.update();
  
}