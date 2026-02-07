#ifndef AUTH_MANAGER_H
#define AUTH_MANAGER_H

#include <Arduino.h>
#include "RFIDManager.h"
#include "AlcoholManager.h"

#include <MFRC522.h>

class AuthManager {
  public:
    AuthManager(RFIDManager& r, AlcoholManager& a);
    void begin();
    void update();
    void handleResponse(bool isPass);

  private:
    RFIDManager& rfid;
    AlcoholManager& alcohol;

    enum State {
      WAIT_FOR_AUTH,
      MEASURING,
      ACCESS_GRANTED,
      ACCESS_DENIED
    };

    State currentState = WAIT_FOR_AUTH;

    void sendResultPacket(char signal);
};

#endif
