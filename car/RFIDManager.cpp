#include "RFIDManager.h"
#include <string.h>  // for memset

RFIDManager::RFIDManager(byte ssPin, byte rstPin)
  : mfrc(ssPin, rstPin)
{
}

void RFIDManager::begin()
{
  mfrc.PCD_Init();
}

void RFIDManager::update()
{
  newCardDetected = false;

  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  MFRC522::StatusCode status = mfrc.PICC_RequestA(bufferATQA, &bufferSize);

  if (status == MFRC522::STATUS_OK)
  {
    lastSeen = millis();
    isCardPresent = true;
  }
  else
  {
    if (isCardPresent && millis() - lastSeen > DEBOUNCE_TIME)
    {
      isCardPresent = false;
    }
  }

  if (isCardPresent && !wasCardPresent)
  {
    if (mfrc.PICC_ReadCardSerial() && mfrc.uid.size == 4)
    {
      for (byte i = 0; i < 4; i++)
      {
        UID[i] = mfrc.uid.uidByte[i];
      }
      newCardDetected = true;
      mfrc.PICC_HaltA();
      mfrc.PCD_StopCrypto1();
    }
    else
    {
      memset(UID, 0, 4);
    }
    wasCardPresent = true;
  }

  if (!isCardPresent && wasCardPresent)
  {
    wasCardPresent = false;
  }
}

const byte* RFIDManager::getUIDBytes() const
{
  return UID;
}

bool RFIDManager::isNewCardDetected() const
{
  return newCardDetected;
}
