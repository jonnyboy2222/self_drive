#ifndef RFIDMANAGER_H
#define RFIDMANAGER_H

#include <Arduino.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// RFID 감지 해제까지 기다릴 시간 (ms 단위)
#define RFID_DEBOUNCE_TIME 500

#define RFID_RST_PIN 44
#define RFID_SS_PIN 10
#define RFID_DEBOUNCE_TIME 1000

class RFIDManager 
{
  private:
    MFRC522 mfrc;
    bool isCardPresent;
    bool wasCardPresent;
    unsigned long lastSeen;
    String currentUID;
    StaticJsonDocument<200> doc;
    SoftwareSerial &espSerial;

    String getUIDStringFromReader(); // 내부용 UID 읽기 메서드

  public:
    RFIDManager(byte ssPin, byte rstPin, SoftwareSerial &esp);
    void begin();
    void update();
    String getActiveUID();
};

#endif
