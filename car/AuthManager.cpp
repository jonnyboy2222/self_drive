#include "AuthManager.h"

const char CMD_VERIFY[] = "VF";
const char CMD_PF[] = "PF";
const byte PACKET_HEADER = 0xAA;
const int PF_PACKET_SIZE = 1 + 2 + 1;
const char SIGNAL_P = 'P';
const char SIGNAL_F = 'F';

AuthManager::AuthManager(RFIDManager& r, AlcoholManager& a)
  : rfid(r), alcohol(a)
{}

void AuthManager::begin() {
  rfid.begin();
  alcohol.begin();
}

void AuthManager::handleResponse(bool isPass) {
  if (!isPass) {
    currentState = WAIT_FOR_AUTH;
    return;
  }

  if (currentState == WAIT_FOR_AUTH) {
    currentState = MEASURING;
  }
}

void AuthManager::update() {
  rfid.update();

  if (rfid.isNewCardDetected()) {
    const int VF_PACKET_SIZE = 1 + 2 + 4;
    char send_buffer[VF_PACKET_SIZE];
    send_buffer[0] = PACKET_HEADER;
    memcpy(send_buffer + 1, CMD_VERIFY, 2);
    memcpy(send_buffer + 3, rfid.getUIDBytes(), 4);
    Serial.write((const uint8_t *)send_buffer, VF_PACKET_SIZE);
  }

  if (currentState == MEASURING) {
    if (!alcohol.isMeasuring()) {
      if (alcohol.isSwitchPressed()) {
        alcohol.startMeasuring();
      }
    } else {
      bool isSafe = alcohol.update();

      if (!alcohol.isMeasuring()) {
        if (isSafe) {
          sendResultPacket(SIGNAL_P);
        } else {
          sendResultPacket(SIGNAL_F);
        }
        currentState = WAIT_FOR_AUTH;
      }
    }
  }
}

void AuthManager::sendResultPacket(char signal) {
  char send_buffer[PF_PACKET_SIZE];
  send_buffer[0] = PACKET_HEADER;
  memcpy(send_buffer + 1, CMD_VERIFY, 2);
  send_buffer[3] = signal;
  Serial.write((const uint8_t *)send_buffer, PF_PACKET_SIZE);
}
