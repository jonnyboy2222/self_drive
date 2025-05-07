const int LED = 13;
const int CENSOR = A0;

unsigned long prevMillis = millis();
const long delayTime = 5000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(CENSOR, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long curMillis = millis();
  int tilt = digitalRead(CENSOR);

  if (curMillis - prevMillis >= delayTime) {
    Serial.println(tilt);
  }

  if (tilt == 0) {
    digitalWrite(LED, LOW);
  }

  else {
    digitalWrite(LED, HIGH);
  }

}
