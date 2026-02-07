#include <LiquidCrystal.h>

class BackingManager
{
  private:
    //인식 핀 연결
    const int TRIG = 8;
    const int ECHO = 7;
    const int BUZZER = 9;
    const int BUTTON = 10;


    LiquidCrystal LCD;
    /* LiquidCrystal : LCD를 제어하기 위한 객체(클래스)
      LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

      rs : Register Select (데이터/명령 선택 핀)
      en : Enable (데이터 읽기/쓰기 시작 핀)
      d4~d7 : 데이터 핀 (4비트 모드로 사용)*/


    //필요한 전역 변수 생성

    //후진 상태 체크 관련 변수
    bool pre_buttonstate = LOW;
    bool isbackingup = false;
    bool current_buttonstate = LOW;

    //초음파 관련 변수
    const static int SAMPLESIZE = 10;                // num==10
    float samples[SAMPLESIZE] = {0};         // 10개의 칸을 가진 float 배열(samples)이 되고, 모두 0으로 시작!
    int sampleindex = 0;
    float total_distance = 0;
    float avg_distance = 0;                  // 평균
    unsigned long pre_measuretime = 0;       //이전 거리측정 시간
    unsigned long checktime_interval = 15;

    //수동부저 관련 변수
    unsigned long prebeeptime = 0;
    bool buzzerstate = false;
    int beepfreq = 0;
    int beepinterval = 0;

  public:
    BackingManager(int rs, int en, int d4, int d5, int d6, int d7)
    : LCD(rs, en, d4, d5, d6, d7) {}

    void begin()
    {
      pinMode(TRIG, OUTPUT);
      pinMode(ECHO, INPUT);
      pinMode(BUZZER, OUTPUT);
      pinMode(BUTTON, INPUT);

      LCD.begin(16, 2);
      LCD.setCursor(0, 0);
      LCD.print("                "); 
    }

    ///////////////////////////////////////////////////////////////////////////////후진 중? 체크 함수////////////////////////////////////////////////////////////
    void isBackState()
    {
      current_buttonstate = digitalRead(BUTTON);
      if (pre_buttonstate == LOW && current_buttonstate == HIGH)
      {
        isbackingup = !isbackingup;
      }
    }

/////////////////////////////////////////////////////////////////////////////초음파 평균값거리 측정함수////////////////////////////////////////////////////////
    void avgDistance()
    {
      unsigned long nowtime = millis();

      if (nowtime - pre_measuretime >= checktime_interval)
      {
        //센서 이용해서 거리 구하기
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);

        unsigned long duration = pulseIn(ECHO, HIGH, 20000);           //부딪혔다가 돌아오는 시간(20ms초과하면 0반환)
        float distance = duration * 0.034 / 2;                         //cm로 환산

        if (distance>0 && distance<340)                                //이상값 제한(pulseIn(ECHO, HIGH, 20000)로 측정가능한게 340cm까지)
        {
          total_distance -= samples[sampleindex];
          samples[sampleindex] = distance;
          total_distance += samples[sampleindex];                      // total_distance 구하기
          sampleindex = (sampleindex+1) % SAMPLESIZE ;                 // samples 안에 distance 10개 채우는 과정
        }
        pre_measuretime = nowtime;
    
        // avg_distance 구하기
        if (sampleindex == 0)
        {
          avg_distance = total_distance / SAMPLESIZE;
          Serial.print("AVG_DISTANCE: ");
          Serial.println(avg_distance);
        }
      }     
    }
///////////////////////////////////////////////////////////////////beepfreq 결정함수///////////////////////////////////////////////////////////////
    void setBeepfreqByDistance()
    {
      if (avg_distance <= 10) 
      {
        beepfreq = 2000; 
        beepinterval = 30;
      } 
      else if (avg_distance <= 20) 
      {
        beepfreq = 1500; 
        beepinterval = 50;
      } 
      else if (avg_distance <= 40) 
      {
        beepfreq = 1000; 
        beepinterval = 100;
      } 
      else if (avg_distance <= 70) 
      {
        beepfreq = 700;  
        beepinterval = 200;
      } 
      else if (avg_distance <= 100) 
      {
        beepfreq = 400;  
        beepinterval = 250;
      } 
      else 
      {
        beepfreq = 0;    
        beepinterval = 0;
        noTone(BUZZER);
      }
    }

////////////////////////////////////////////////////////////////////수동부저 제어 함수///////////////////////////////////////////////////////////////
    void controldBuzzer()
    {
      unsigned long nowtime = millis();
      if (beepfreq>0 && nowtime - prebeeptime >= beepinterval)
      {
        prebeeptime = nowtime;
        if (buzzerstate)
        {
          noTone(BUZZER);
          buzzerstate = false;
        }
        else
        {
          tone(BUZZER, beepfreq);
          buzzerstate = true;
        }
      }
    }

//////////////////////////////////////////////////////////////////////////backingUpdateLCD 함수 //////////////////////////////////////////////////////
    void backingUpdateLCD()
    {
      static String prevline2 = "";
      String line2 = "";

      LCD.setCursor(0, 0);
      LCD.print("backing up...   ");
      if (avg_distance <=100)
      {
        int step = constrain((100 - avg_distance) / 10, 0, 10);
        int cPos = 2 + step;  
        int numStars = 12 - cPos;  

        for (int i = 0; i < cPos; i++) 
        { 
          line2 += " ";
        }
        line2 += "C";
        for (int i = 0; i < numStars; i++) 
        {
          line2 += "*";
        }
        line2 += "O";
      }
      else
      {
        line2 = "                "; 
      }
      /////////깜빡임 방지
      if (line2 != prevline2) 
      {
        LCD.setCursor(0, 1);
        LCD.print(line2);
        prevline2 = line2;
      }
    }

    void update()
    {
      isBackState();

      if(isbackingup)
      {
        avgDistance();
        setBeepfreqByDistance();
        backingUpdateLCD();
        controldBuzzer();
      }
      pre_buttonstate = current_buttonstate;

      if (!isbackingup) 
      {
        noTone(BUZZER);
        buzzerstate = false;
        LCD.clear();
        return;  // 후진 중이 아니면 함수 빠져나가기
      }
    }
};

BackingManager backingManager(12,11,5,4,3,2);

void setup()
{
  Serial.begin(9600);
  backingManager.begin();
}

void loop()
{
  backingManager.update();
}


