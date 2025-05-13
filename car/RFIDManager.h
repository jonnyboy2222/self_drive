#ifndef RFID_MANAGER_H
#define RFID_MANAGER_H

#include <MFRC522.h>

class RFIDManager
{
  private:
    static const unsigned long DEBOUNCE_TIME = 500;  // ← 클래스 내부 상수

    MFRC522 mfrc;
    bool isCardPresent = false;
    bool wasCardPresent = false;
    bool newCardDetected = false;
    unsigned long lastSeen = 0;
    byte UID[4] = {0};

  public:
    RFIDManager(byte ssPin, byte rstPin);

    void begin();
    void update();

    const byte* getUIDBytes() const;
    bool isNewCardDetected() const;
};

#endif
