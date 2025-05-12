#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

#define BAUD_RATE 9600

#define RFID_RST_PIN 9
#define RFID_SS_PIN 10
#define RFID_DEBOUNCE_TIME 500

#define ALCOHOL_SENSOR_PIN A2
#define SWITCH_PIN 8

#define MOTOR_L_IN1 4
#define MOTOR_L_IN2 5
#define MOTOR_R_IN1 6
#define MOTOR_R_IN2 7

const char CMD_VERIFY[] = "VF";

const byte PACKET_HEADER = 0xAA;
const int VF_PACKET_SIZE = 1 + 2 + 4; // 1(header) + 2(command) + 4(uid)

bool flag = true;

enum DriveState {
    WAIT_FOR_AUTH,
    MEASURING,
    ACCESS_GRANTED,
    ACCESS_DENIED
};

// === RFID Manager ===
class RFIDManager
{
  private:
    MFRC522 mfrc;
    bool isCardPresent = false;
    bool wasCardPresent = false;
    bool newCardDetected = false;
    unsigned long lastSeen = 0;
    byte UID[4] = {0};

  public:
    RFIDManager(byte ssPin, byte rstPin) : mfrc(ssPin, rstPin)
    {
    }

    void begin()
    {
      mfrc.PCD_Init();
    }

    void update()
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
        if (isCardPresent && millis() - lastSeen > RFID_DEBOUNCE_TIME)
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
          //Serial.println("card detected");
          mfrc.PICC_HaltA();
          mfrc.PCD_StopCrypto1();
        }
        else
        {
          //Serial.println("card detected, but UID read failed");
          memset(UID, 0, 4);
        }
        wasCardPresent = true;
      }

      if (!isCardPresent && wasCardPresent)
      {
        //Serial.println("card removed");
        wasCardPresent = false;
        memset(UID, 0, 4);
      }
    }

    const byte* getUIDBytes() const
    {
      return UID;
    }

    bool isNewCardDetected() const
    {
      return newCardDetected;
    }
};

//=== Alcohol Sensor Manager ===
class AlcoholManager
{
  private:
    const int THRESHOLD = 200;
    const unsigned long MEASUREDURATION = 5000;
    const unsigned long SAMPLEINTERVAL = 100;

    bool measuring = false;
    unsigned long startTime = 0;
    unsigned long lastSampleTime = 0;
    unsigned long sum = 0;
    int count = 0;

  public:
    void begin()
    {
      pinMode(ALCOHOL_SENSOR_PIN, INPUT);
      pinMode(SWITCH_PIN, INPUT_PULLUP);
    }

    bool isSwitchPressed()
    {
      static bool lastSwitch = HIGH;
      bool current = digitalRead(SWITCH_PIN);
      bool pressed = (lastSwitch == HIGH && current == LOW);
      lastSwitch = current;
      return pressed;
    }

    void startMeasuring()
    {
      measuring = true;
      startTime = millis();
      lastSampleTime = 0;
      sum = 0;
      count = 0;
    }

    bool isMeasuring()
    {
      return measuring;
    }

    bool update()
    {
      if (!measuring) return false;

      unsigned long now = millis();
      if (now - lastSampleTime >= SAMPLEINTERVAL)
      {
        lastSampleTime = now;
        int val = analogRead(ALCOHOL_SENSOR_PIN);
        sum += val;
        count++;
      }

      if (now - startTime >= MEASUREDURATION)
      {
        measuring = false;
        float avg = (float)sum / count;
        Serial.print("Average: ");
        Serial.println(avg);
        if (avg < THRESHOLD)
        {
          return true;
        }
        else
        {
          return false;
        }
        
      }

      return false; // 아직 측정 중
    }
};


// === Drive Motor Manager ===
class DriveManager
{
  private:
    bool reversingFlag = false;
  public:
    bool isReversing()
    {
      return reversingFlag;
    }
    void begin()
    {
      pinMode(MOTOR_L_IN1, OUTPUT);
      pinMode(MOTOR_L_IN2, OUTPUT);
      pinMode(MOTOR_R_IN1, OUTPUT);
      pinMode(MOTOR_R_IN2, OUTPUT);
      stop();
    }

    void forward()
    {
      reversingFlag = false;
      digitalWrite(MOTOR_L_IN1, HIGH);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, HIGH);
      digitalWrite(MOTOR_R_IN2, LOW);
    }

    void backward()
    {
      reversingFlag = true;
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, HIGH);
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, HIGH);
    }

    void left()
    {
      reversingFlag = false;
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, HIGH);
      digitalWrite(MOTOR_R_IN2, LOW);
    }

    void right()
    {
      reversingFlag = false;
      digitalWrite(MOTOR_L_IN1, HIGH);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, LOW);
    }

    void stop()
    {
      reversingFlag = false;
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, LOW);
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, LOW);
    }
};

// === System Manager ===
class SystemManager
{
  private:
    AlcoholManager &alcohol;
    DriveManager &drive;
    DriveState currentState = WAIT_FOR_AUTH;
  public:
    SystemManager(AlcoholManager &a, DriveManager &d)
      : alcohol(a), drive(d)
    {
    }

    void handleResponse(const String &cmd)
    {
      if (cmd == "PASS" && currentState == WAIT_FOR_AUTH)
      {
        Serial.println("MEASURING");

        currentState = MEASURING;
      }
      else if (cmd == "FAIL")
      {
        Serial.println("ACCESS DENIED");

        drive.stop();
        currentState = ACCESS_DENIED;
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

            Serial.println("Measuring Start");
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

              Serial.println("ACCESS GRANTED");
              currentState = ACCESS_GRANTED;
            }
            else
            {

              Serial.println("ACCESS DENIED");
              drive.stop();
              currentState = ACCESS_DENIED;
            }
          }
        }
      }
    }


    void handleDriveCommand(const String &cmd)
    {
      if (currentState != ACCESS_GRANTED)
      {
        return;
      }

      if (cmd == "전진")
        drive.forward();
      else if (cmd == "후진")
        drive.backward();
      else if (cmd == "좌회전")
        drive.left();
      else if (cmd == "우회전")
        drive.right();
      else if (cmd == "정지")
        drive.stop();
      else
      {
        //Serial.println("cmd error");
      }
        
    }
};

RFIDManager rfidManager(RFID_SS_PIN, RFID_RST_PIN);
DriveManager driveManager;
AlcoholManager alcoholManager;
SystemManager systemManager(alcoholManager, driveManager);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  SPI.begin();

  driveManager.begin();
  rfidManager.begin();
  alcoholManager.begin(); 

  //Serial.println("Initialized");

}

void loop() {
  // put your main code here, to run repeatedly:
  rfidManager.update();
  systemManager.update();

  
  if (rfidManager.isNewCardDetected())
  {
    char send_buffer[VF_PACKET_SIZE];
    send_buffer[0] = PACKET_HEADER;
    memcpy(send_buffer + 1, CMD_VERIFY, 2);     // 명령어 2바이트
    memcpy(send_buffer + 3, rfidManager.getUIDBytes(), 4); // UID 4바이트
    send_buffer[7] = '\n'; // 패킷 끝 표시 (선택 사항)

    Serial.write((const uint8_t *)send_buffer, VF_PACKET_SIZE);
    //Serial.println("UID sent to host.");
  }
  
}