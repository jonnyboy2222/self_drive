const int LED = 13;
const int CENSOR = A0;
int shock_cnt = 0;
int interval = 0;
double per = 0;

unsigned long prevMillis = millis();
unsigned long prevMillis2 = millis();

const long delayTime = 200;
const long delayTime2 = 1000;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(CENSOR, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long curMillis = millis();
  unsigned long curMillis2 = millis();

  if (curMillis - prevMillis >= delayTime) {
    int tilt = digitalRead(CENSOR);

    if (tilt == 1) {
      shock_cnt += 1;
    }

    Serial.println(shock_cnt);
    Serial.println(interval);
    Serial.println(per);
    Serial.println("\n");

    prevMillis = curMillis;
  }
  
  if (curMillis2 - prevMillis2 >= delayTime2) {
    // shock_cnt = 0;
    interval += 1;

    if (interval != 0){
      per = (double)shock_cnt / interval;
    }

    prevMillis2 = curMillis2;
  }
}


// === Shock Sensor Manager ===
class ShockManager {
public:
  void begin() {
    pinMode(SHOCK_SENSOR_PIN, INPUT);
  }
  
  void update() {
    int value = analogRead(SHOCK_SENSOR_PIN);
    if (value > SHOCK_THRESHOLD) {
      espSerial.println("shock:" + String(value));
    }
  }
};


// if (curMillis2 - prevMillis2 >= delayTime2) {
//     int tilt = digitalRead(CENSOR);
//     detect += 1;

//     if (tilt == 0) {
//       digitalWrite(LED, LOW);
//     }

//     else {
//       digitalWrite(LED, HIGH);
//       shock_cnt += 1;
//     }

//     prevMillis2 = curMillis2;
//   }

//   if (curMillis - prevMillis >= delayTime) {
//     Serial.println(shock_cnt);
//     Serial.println(detect);
//     Serial.println(per);
//     Serial.println("\n");

//     prevMillis = curMillis;
//   }
