#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal.h>

#define RST_PIN 9
#define SS_PIN 10
#define DEBOUNCE_TIME 1000

class RFIDHandler {
private:
  MFRC522 mfrc;
  bool isCardPresent = false;
  bool wasCardPresent = false;
  unsigned long lastSeen = 0;

public:
  String UID = "";  // UID 정보를 외부에서 접근 가능하게

  RFIDHandler(byte ssPin, byte rstPin) : mfrc(ssPin, rstPin) {}

  void begin() {
    mfrc.PCD_Init();
  }

  void update() {
    byte bufferATQA[2];
    byte bufferSize = sizeof(bufferATQA);
    MFRC522::StatusCode status = mfrc.PICC_RequestA(bufferATQA, &bufferSize);

    if (status == MFRC522::STATUS_OK) {
      lastSeen = millis();
      isCardPresent = true;
    } else {
      if (isCardPresent && millis() - lastSeen > DEBOUNCE_TIME) {
        isCardPresent = false;
      }
    }

    if (isCardPresent && !wasCardPresent) {
      if (mfrc.PICC_ReadCardSerial()) {
        UID = getUIDString();  // UID 멤버 변수에 저장
        Serial.println("card detected");
        Serial.print("read uid tag: ");
        Serial.println(UID);
      } else {
        Serial.println("card detected, but UID read failed");
        UID = "";
      }
      wasCardPresent = true;
    }

    if (!isCardPresent && wasCardPresent) {
      Serial.println("card removed");
      wasCardPresent = false;
      UID = "";
    }
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

RFIDHandler rfid(SS_PIN, RST_PIN);
LiquidCrystal lcd(2,3,4,5,6,7);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  rfid.begin();
  lcd.begin(16,2);
  Serial.println("RFID initialized");
}
String lastUID = "";
void loop() {
  // put your main code here, to run repeatedly:
  rfid.update();
  if (rfid.UID != lastUID) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(rfid.UID);
    lastUID = rfid.UID;
  }
}