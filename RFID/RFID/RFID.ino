#include <SPI.h>
#include <MFRC522.h>

#define RFID_RST_PIN 9
#define RFID_SS_PIN 10
#define RFID_DEBOUNCE_TIME 500

// === RFID Manager ===
class RFIDManager {
private:
  MFRC522 mfrc;
  bool isCardPresent = false;
  bool wasCardPresent = false;
  unsigned long lastSeen = 0;
  String UID = ""; 

public:
  
  RFIDManager(byte ssPin, byte rstPin) : mfrc(ssPin, rstPin) {}

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
        Serial.println("card detected");
        Serial.print("UID:");
        Serial.println(UID);
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

RFIDManager rfid(RFID_SS_PIN, RFID_RST_PIN);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  rfid.begin();
  Serial.println("RFID initialized");

}

void loop() {
  // put your main code here, to run repeatedly:
  rfid.update();
}