// 시리얼통신으로 제어

# include <Servo.h>

int angle = 90;
Servo servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  servo.attach(3);
}

void loop() {
  // put your main code here, to run repeatedly:
    if (Serial.available()) {
    char input = Serial.read();

    if (input == 'a') {
      // left
      Serial.println(input);

      for (int i = 0; i < 15; i++) {
        angle = angle + 1;
        if (angle >= 135 {
          angle = 135;
        }

        servo.write(angle);
        delay(10);
      }

      Serial.print("\t\t");
      Serial.println(angle);
    }

    else if (input == 'd') {
      Serial.println(input);

      for (int i = 0; i < 15; i++) {
        angle = angle - 1;
        if (angle <= 45) {
          angle = 45;
        }

        servo.write(angle);
        delay(10);
      }

      Serial.print("\t\t");
      Serial.println(angle);
    }
  }
}


// 가변저항으로 제어

// #include <Servo.h>

// // int angle = 90;
// Servo servo;
// int potPin  = A0;
// int potVal;
// int angle;

// void setup() {
//   // put your setup code here, to run once:
//   servo.attach(3);
//   // Serial.println(9600);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   potVal = analogRead(potPin);

//   angle = map(potVal, 0, 1023, 45, 135);

//   servo.write(angle);
//   delay(15);
// }



