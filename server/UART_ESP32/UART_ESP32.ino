#define RXD2 3
#define TXD2 1

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2); // arduino connection

}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial1.available()) 
  {
    String sensorData = Serial1.readStringUntil('\n'); // read JSON line
    Serial1.println("Received: " + sensorData);
  }
  delay(100);
}
