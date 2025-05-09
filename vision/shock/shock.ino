const int LED = 13;
const int CENSOR = A0;
int shock_cnt = 0;
int detect = 0;

unsigned long prevMillis = millis();
unsigned long prevMillis2 = millis();

const long delayTime = 1000;
const long delayTime2 = 500;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(CENSOR, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  double per = shock_cnt / detect * 100;

  unsigned long curMillis = millis();
  unsigned long curMillis2 = millis();
  int tilt = digitalRead(CENSOR);

  if (curMillis2 - prevMillis2 >= delayTime2) {
    int tilt = digitalRead(CENSOR);
    detect += 1;

    if (tilt == 0) {
      digitalWrite(LED, LOW);
    }

    else {
      digitalWrite(LED, HIGH);
      shock_cnt += 1;
    }

    prevMillis2 = curMillis2;
  }

  if (curMillis - prevMillis >= delayTime) {
    Serial.println(shock_cnt);
    Serial.println(detect);
    Serial.println(per);
    Serial.println("\n");

    prevMillis = curMillis;
  }

  
  // Serial.println(tilt);
  // if (tilt == 0) {
  // digitalWrite(LED, LOW);
  // }

  // else {
  //   digitalWrite(LED, HIGH);
  //   shock_cnt += 1;
  // }

}
