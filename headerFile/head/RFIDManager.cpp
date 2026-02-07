#include "RFIDManager.h"
#include "Arduino.h"

RFIDManager::RFIDManager(byte ssPin, byte rstPin, SoftwareSerial &esp)
  : mfrc(ssPin, rstPin), espSerial(esp),
    isCardPresent(false), wasCardPresent(false),
    lastSeen(0), currentUID("") {}

void RFIDManager::begin() {
  mfrc.PCD_Init();
}

void RFIDManager::update() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  MFRC522::StatusCode status = mfrc.PICC_RequestA(bufferATQA, &bufferSize);

  if (status == MFRC522::STATUS_OK) {
    lastSeen = millis();
    isCardPresent = true;
  } else {
    if (isCardPresent && millis() - lastSeen > RFID_DEBOUNCE_TIME) {
      isCardPresent = false;
    }
  }

  if (isCardPresent && !wasCardPresent) {
    if (mfrc.PICC_ReadCardSerial()) {
      currentUID = getUIDStringFromReader();

      doc.clear();
      doc["purpose"] = "verification";
      doc["rfid_uid"] = currentUID;

      serializeJson(doc, espSerial);
      espSerial.println(); // JSON 파싱을 위한 줄바꿈

      mfrc.PICC_HaltA();
      mfrc.PCD_StopCrypto1();
    } else {
      Serial.println("Card detected, but UID read failed");
      currentUID = "";
    }
    wasCardPresent = true;
  }

  if (!isCardPresent && wasCardPresent) {
    Serial.println("Card removed");
    wasCardPresent = false;
    currentUID = "";
  }
}

String RFIDManager::getUIDStringFromReader() {
  String uid = "";
  for (byte i = 0; i < mfrc.uid.size; i++) {
    if (mfrc.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(mfrc.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

String RFIDManager::getActiveUID() {
  return currentUID;
}
