#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>
#include "RFIDManager.h"
#include "AlcoholManager.h"

#define BAUD_RATE 9600

#define RFID_RST_PIN 9
#define RFID_SS_PIN 10

#define ALCOHOL_SENSOR_PIN A2
#define SWITCH_PIN 8

#define MOTOR_L_IN1 4
#define MOTOR_L_IN2 5
#define MOTOR_R_IN1 6
#define MOTOR_R_IN2 7

const char CMD_VERIFY[] = "VF";

const byte PACKET_HEADER = 0xAA;
const int VF_PACKET_SIZE = 1 + 2 + 4; // 1(header) + 2(command) + 4(uid)

const char CMD_PF[] = "PF";
const int PF_PACKET_SIZE = 1 + 2 + 1;
const char SIGNAL_P = 'P';
const char SIGNAL_F = 'F';
int idx=0;
char recv_buffer[4];

enum State {
    WAIT_FOR_AUTH,
    MEASURING,
    ACCESS_GRANTED,
    ACCESS_DENIED
};

// === System Manager ===
class SystemManager
{
  private:
    AlcoholManager &alcohol;
    State currentState = WAIT_FOR_AUTH;
  public:
    SystemManager(AlcoholManager &a)
      : alcohol(a)
    {
    }

    void handleResponse(bool isPass)
    {
      if (!isPass)
      {
        drive.stop();
        currentState = WAIT_FOR_AUTH;
        return;
      }

      if (currentState == WAIT_FOR_AUTH)
      {
        currentState = MEASURING;
      }
    }


    void update()
    {
      if (currentState == MEASURING)
      {
        if (!alcohol.isMeasuring())
        {
          if (alcohol.isSwitchPressed())
          {
            alcohol.startMeasuring();
          }
        }
        else
        {
          bool isSafe = alcohol.update(); // true면 정상

          // 측정이 끝났을 경우
          if (!alcohol.isMeasuring())
          {
            if (isSafe)
            {

              char send_buffer[PF_PACKET_SIZE];
              send_buffer[0] = PACKET_HEADER;
              memcpy(send_buffer + 1, CMD_VERIFY, 2);
              memcpy(send_buffer + 3, &SIGNAL_P ,1);
              Serial.write((const uint8_t *)send_buffer, PF_PACKET_SIZE);
              currentState = WAIT_FOR_AUTH;
            }
            else
            {

              char send_buffer[PF_PACKET_SIZE];
              send_buffer[0] = PACKET_HEADER;
              memcpy(send_buffer + 1, CMD_VERIFY, 2);
              memcpy(send_buffer + 3, &SIGNAL_F ,1);
              Serial.write((const uint8_t *)send_buffer, PF_PACKET_SIZE);
              drive.stop();
              currentState = WAIT_FOR_AUTH;
            }
          }
        }
      }
    }

};

RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN);
AlcoholManager alcoholManager;
SystemManager systemManager(alcoholManager);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  SPI.begin();

  driveManager.begin();
  rfidManager.begin();
  alcoholManager.begin(); 

}

void loop()
{
  rfidManager.update();
  systemManager.update();

  // 카드 감지 시 UID 전송
  if (rfidManager.isNewCardDetected())
  {
    char send_buffer[VF_PACKET_SIZE];
    send_buffer[0] = PACKET_HEADER;
    memcpy(send_buffer + 1, CMD_VERIFY, 2);
    memcpy(send_buffer + 3, rfidManager.getUIDBytes(), 4);
    Serial.write((const uint8_t *)send_buffer, VF_PACKET_SIZE);
  }

  // UID 등록 확인 결과 수신
  static int idx = 0;
  static char recv_buffer[4];

  while (Serial.available())
  {
    char byte = Serial.read();

    // 헤더가 아닐 경우 무시
    if (idx == 0 && (uint8_t)byte != PACKET_HEADER)
    {
      continue;
    }

    recv_buffer[idx++] = byte;

    if (idx == 4)
    {
      if (recv_buffer[1] == 'V' && recv_buffer[2] == 'F')
      {
        if ((uint8_t)recv_buffer[3] == 1)
        {
          systemManager.handleResponse(true);   // PASS
        }
        else if ((uint8_t)recv_buffer[3] == 0)
        {
          systemManager.handleResponse(false);  // FAIL
        }
        else
        {
        }
      }
      idx = 0;
    }

  }
}