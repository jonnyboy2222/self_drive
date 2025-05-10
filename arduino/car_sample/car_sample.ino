#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define BAUD_RATE 9600
#define ESP_BAUD_RATE 9600
#define BT_BAUD_RATE 9600

// === 핀 정의 ===

// 통신
#define ESP_RX_PIN 14
#define ESP_TX_PIN 15
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);  // ESP32

#define BT_RXD 2
#define BT_TXD 3
SoftwareSerial btSerial(BT_RXD, BT_TXD);    // Bluetooth

// 제어
#define MOTOR_L_IN1 22
#define MOTOR_L_IN2 23
#define MOTOR_R_IN1 24
#define MOTOR_R_IN2 25
#define MOTOR_1_SPEED 5
#define MOTOR_2_SPEED 6
#define SERVO 9

Servo steering;
String input = ""

// 센서

#define SHOCK_SENSOR_PIN 26
// #define SHOCK_THRESHOLD  600

#define ULTRASONIC_TRIG  28
#define ULTRASONIC_ECHO  29
#define MAX_DISTANCE_CM  150
#define MIN_DISTANCE_CM  10

#define TEMP_SENSOR_PIN  A0
#define TEMP_THRESHOLD_C 40

#define ALCOHOL_SENSOR_PIN A1

#define HEADLIGHT_LED_PIN A2

// 출력
#define LCD_RS_PIN 30
#define LCD_EN_PIN 31
#define LCD_D4_PIN 32
#define LCD_D5_PIN 33
#define LCD_D6_PIN 34
#define LCD_D7_PIN 35

#define BUZZER_PIN 10

#define LIGHT_SENSOR_PIN 13
#define LIGHT_THRESHOLD  300

// RFID
#define RFID_RST_PIN 44
#define RFID_SS_PIN 10
#define RFID_DEBOUNCE 1000
#define THRESHOLD 400


// === LCD Manager ===
class LCDManager {
private:
  LiquidCrystal lcd;
public:
  LCDManager(uint8_t rs, uint8_t en, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
    : lcd(rs, en, d4, d5, d6, d7) {}

  void begin() { lcd.begin(16, 2); lcd.clear(); }
  void printLine(int row, const String &text) {
    lcd.setCursor(0, row);
    lcd.print("                ");
    lcd.setCursor(0, row);
    lcd.print(text);
  }
  void clear() { lcd.clear(); }
};

// === Alcohol Sensor Manager ===
class AlcoholManager {
public:
  int read() { return analogRead(ALCOHOL_SENSOR_PIN); }
  bool isSafe(int value) { return value < THRESHOLD; }
};

// === Drive Motor Manager ===
class DriveManager {
  public:
    void begin() {
      pinMode(MOTOR_L_IN1, OUTPUT);
      pinMode(MOTOR_L_IN2, OUTPUT);
      pinMode(MOTOR_R_IN1, OUTPUT);
      pinMode(MOTOR_R_IN2, OUTPUT);
      pinMode(MOTOR_1_SPEED, OUTPUT);
      pinMode(MOTOR_2_SPEED, OUTPUT);
      steering.attach(SERVO);
      steering.write(90);
    }

    void moveForward(int speed = 150) {
      digitalWrite(MOTOR_L_IN1, HIGH);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, HIGH);
      digitalWrite(MOTOR_R_IN2, LOW);
      analogWrite(MOTOR_1_SPEED, speed);
      analogWrite(MOTOR_2_SPEED, speed);
    }

    void moveBackward(int speed = 150) {
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, HIGH);
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, HIGH);
      analogWrite(MOTOR_1_SPEED, speed);
      analogWrite(MOTOR_2_SPEED, speed);
    }

    void stopMotors() {
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, LOW);
      analogWrite(MOTOR_1_SPEED, 0);
      analogWrite(MOTOR_2_SPEED, 0);
    }
};

// === Obstacle Alert Manager (Ultrasonic + Buzzer) ===
class ObstacleManager {
public:
  void begin() {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
  }

  void update() {
    if (!isReversing) return;
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
    int distance = duration * 0.034 / 2;

    if (distance == 0 || distance > MAX_DISTANCE_CM) {
      noTone(BUZZER_PIN);
    } else {
      int freq = map(distance, MIN_DISTANCE_CM, MAX_DISTANCE_CM, 2000, 400);
      int delayMs = map(distance, MIN_DISTANCE_CM, MAX_DISTANCE_CM, 50, 500);
      tone(BUZZER_PIN, freq);
      delay(delayMs);
      noTone(BUZZER_PIN);
    }
  }
};

// === Shock Sensor Manager ===
class ShockManager {
public:
  void begin() { pinMode(SHOCK_SENSOR_PIN, INPUT); }
  void update() {
    int value = analogRead(SHOCK_SENSOR_PIN);
    if (value > SHOCK_THRESHOLD) {
      espSerial.println("shock:" + String(value));
    }
  }
};

// === Temperature Manager ===
class TempManager {
public:
  void begin() { pinMode(TEMP_SENSOR_PIN, INPUT); }
  void update() {
    int raw = analogRead(TEMP_SENSOR_PIN);
    float voltage = raw * 5.0 / 1023.0;
    float tempC = voltage * 100;
    if (tempC > TEMP_THRESHOLD_C) {
      espSerial.println("temp:" + String(tempC));
    }
  }
};

// === Ambient Light Manager ===
class AmbientLightManager {
public:
  void begin() {
    pinMode(LIGHT_SENSOR_PIN, INPUT);
    pinMode(HEADLIGHT_LED_PIN, OUTPUT);
  }

  void update() {
    int lightValue = analogRead(LIGHT_SENSOR_PIN);
    if (lightValue < LIGHT_THRESHOLD) {
      digitalWrite(HEADLIGHT_LED_PIN, HIGH);
    } else {
      digitalWrite(HEADLIGHT_LED_PIN, LOW);
    }
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
class ESPManager {
private:
  SoftwareSerial &esp;
public:
  ESPManager(SoftwareSerial &serial) : esp(serial) {}
  void sendUID(const String &uid) { esp.println(uid); }
  String getResponse() {
    if (esp.available()) {
      String res = esp.readStringUntil('\n');
      res.trim();
      return res;
    }
    return "";
  }
};

// === Bluetooth Manager ===
class BluetoothManager {
private:
  SoftwareSerial &bt;
public:
  BluetoothManager(SoftwareSerial &serial) : bt(serial) {}
  String getCommand() {
    if (bt.available()) {
      String cmd = bt.readStringUntil('\n');
      cmd.trim();
      return cmd;
    }
    return "";
  }
};

// === System Manager ===
class SystemManager {
private:
  LCDManager &lcd;
  AlcoholManager &alcohol;
  DriveManager &drive;
  DriveState currentState = WAIT_FOR_AUTH;
public:
  SystemManager(LCDManager &l, AlcoholManager &a, DriveManager &d)
    : lcd(l), alcohol(a), drive(d) {}

  void handleResponse(const String &cmd) {
    if (cmd == "pass" && currentState == WAIT_FOR_AUTH) {
      lcd.printLine(0, "Measuring...");
      currentState = MEASURING;
    } else if (cmd == "fail") {
      lcd.printLine(0, "ACCESS DENIED");
      drive.stop();
      currentState = ACCESS_DENIED;
    }
  }

  void update() {
    if (currentState == MEASURING) {
      delay(2000);
      int val = alcohol.read();
      lcd.printLine(0, "Alcohol: " + String(val));
      if (alcohol.isSafe(val)) {
        lcd.printLine(1, "ACCESS GRANTED");
        currentState = ACCESS_GRANTED;
      } else {
        lcd.printLine(1, "ALC TOO HIGH");
        drive.stop();
        currentState = ACCESS_DENIED;
      }
    }
  }

  void handleDriveCommand(const String &cmd) {
    if (cmd.startsWith("X")) {
      int angle = cmd.substring(1).toInt();
      steering.write(angle);
    } else if (cmd.length() > 0) {
      char action = cmd.charAt(0);
      switch (action) {
        case 'F': motor.moveForward(); break;
        case 'B': motor.moveBackward(); break;
        case 'S': motor.stopMotors(); break;
      }
    }
  }
};

// === 인스턴스 생성 ===
LCDManager lcdManager(LCD_RS_PIN,LCD_EN_PIN,LCD_D4_PIN,LCD_D5_PIN,LCD_D6_PIN,LCD_D7_PIN);
AlcoholManager alcoholManager;
DriveManager driveManager;
RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN);
ESPManager espManager(espSerial);
BluetoothManager bluetoothManager(btSerial);
SystemManager systemManager(lcdManager, alcoholManager, driveManager);
ObstacleManager obstacleManager;
ShockManager shockManager;
TempManager tempManager;
AmbientLightManager ambientLightManager;

void setup() {
  Serial.begin(BAUD_RATE);
  espSerial.begin(ESP_BAUD_RATE);
  btSerial.begin(BT_BAUD_RATE);
  SPI.begin();
  lcdManager.begin();
  driveManager.begin();
  rfidManager.begin();
  espManager.sendUID("");  // 초기화
  obstacleManager.begin();
  shockManager.begin();
  tempManager.begin();
  ambientLightManager.begin();
  lcdManager.printLine(0, "WAITING RFID");
}

void loop() {
  String uid = rfidManager.checkNewUID();
  if (uid != "") {
    lcdManager.printLine(0, "Send UID: " + uid);
    espManager.sendUID(uid);
  }

  String espRes = espManager.getResponse();
  if (espRes != "") systemManager.handleResponse(espRes);

  String btCmd = bluetoothManager.getCommand();
  if (btCmd != "") systemManager.handleDriveCommand(btCmd);

  systemManager.update();
  obstacleManager.update();
  shockManager.update();
  tempManager.update();
  ambientLightManager.update();
}