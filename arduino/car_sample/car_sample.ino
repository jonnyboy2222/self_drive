#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <MFRC522.h>

// === 핀 정의 ===
#define ALCOHOL_SENSOR_PIN A0
#define MOTOR_L_IN1 4
#define MOTOR_L_IN2 5
#define MOTOR_R_IN1 6
#define MOTOR_R_IN2 7
#define RFID_RST_PIN 9
#define RFID_SS_PIN 10
#define RFID_DEBOUNCE 1000
#define THRESHOLD 400
#define ULTRASONIC_TRIG  A1
#define ULTRASONIC_ECHO  A2
#define BUZZER_PIN       A3
#define SHOCK_SENSOR_PIN A4
#define SHOCK_THRESHOLD  600
#define TEMP_SENSOR_PIN  A5
#define TEMP_THRESHOLD_C 40
#define LIGHT_SENSOR_PIN A6
#define LIGHT_THRESHOLD  300
#define HEADLIGHT_LED_PIN A7
#define MAX_DISTANCE_CM  150
#define MIN_DISTANCE_CM  10

SoftwareSerial espSerial(12, 13);  // ESP32
SoftwareSerial btSerial(8, 11);    // Bluetooth

enum DriveState {
  WAIT_FOR_AUTH,
  MEASURING,
  ACCESS_GRANTED,
  ACCESS_DENIED
};


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
    stop();
  }
  void forward() {
    isReversing = false;
    digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
  }
  void backward() {
    isReversing = true;
    digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, HIGH);
    digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, HIGH);
  }
  void left() {
    isReversing = false;
    digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
  }
  void right() {
    isReversing = false;
    digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, LOW);
  }
  void stop() {
    isReversing = false;
    digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, LOW);
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

public:
  RFIDManager(byte ssPin, byte rstPin) : mfrc(ssPin, rstPin) {}
  void begin() { mfrc.PCD_Init(); }

  String checkNewUID() {
    byte bufferATQA[2];
    byte bufferSize = sizeof(bufferATQA);
    MFRC522::StatusCode status = mfrc.PICC_RequestA(bufferATQA, &bufferSize);
    if (status == MFRC522::STATUS_OK) {
      lastSeen = millis(); isCardPresent = true;
    } else if (isCardPresent && millis() - lastSeen > RFID_DEBOUNCE) {
      isCardPresent = false;
    }
    if (isCardPresent && !wasCardPresent && mfrc.PICC_ReadCardSerial()) {
      wasCardPresent = true; return getUIDString();
    }
    if (!isCardPresent && wasCardPresent) wasCardPresent = false;
    return "";
  }

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
    if (currentState != ACCESS_GRANTED) {
      lcd.printLine(0, "CMD BLOCKED");
      return;
    }
    if (cmd == "전진") drive.forward();
    else if (cmd == "후진") drive.backward();
    else if (cmd == "좌회전") drive.left();
    else if (cmd == "우회전") drive.right();
    else if (cmd == "정지") drive.stop();
    else lcd.printLine(1, "BT: UNKNOWN");
  }
};

// === 인스턴스 생성 ===
LCDManager lcdManager(2, 3, 8, 9, 10, 11);
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
  Serial.begin(9600);
  espSerial.begin(9600);
  btSerial.begin(9600);
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