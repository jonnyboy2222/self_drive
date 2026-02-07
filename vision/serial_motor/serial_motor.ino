#include <Servo.h>

Servo servo;

const int M1_1 = 5;
const int M1_2 = 6;
const int M2_1 = 9;
const int M2_2 = 10;
const int MS = 3;

int angle = 90;

void setup() {
  pinMode(M1_1, OUTPUT);
  pinMode(M1_2, OUTPUT);
  pinMode(M2_1, OUTPUT);
  pinMode(M2_2, OUTPUT);
  servo.attach(MS);

  Serial.begin(9600);
}
void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    switch (command) {
      case 'w':  // forward
        forward();
        break;
      case 's':  // backward
        backward();
        break;
      case 'a':  // turn left
        left();
        break;
      case 'd':  // turn right
        right();
        break;
      case 'x':
        stop();
        break;
      default:
        stop();
        break;
    }
  }
}

void forward(){
  digitalWrite(M1_1, HIGH);
  digitalWrite(M1_2, LOW);
  digitalWrite(M2_1, HIGH);
  digitalWrite(M2_2, LOW);
}

void backward() {
  digitalWrite(M1_1, LOW);
  digitalWrite(M1_2, HIGH);
  digitalWrite(M2_1, LOW);
  digitalWrite(M2_2, HIGH);
}

void left() {
  // digitalWrite(M1_1, LOW);
  // digitalWrite(M1_2, HIGH);
  // digitalWrite(M2_1, HIGH);
  // digitalWrite(M2_2, LOW);
  for (int i = 0; i < 15; i++) {
    angle = angle + 1;
    if (angle >= 135) {
      angle = 135;
    }

    servo.write(angle);
  }
}

void right() {
  // digitalWrite(M1_1, HIGH);
  // digitalWrite(M1_2, LOW);
  // digitalWrite(M2_1, LOW);
  // digitalWrite(M2_2, HIGH);
  for (int i = 0; i < 15; i++) {
    angle = angle - 1;
    if (angle <= 45) {
      angle = 45;
    }

    servo.write(angle);
  }
}

void stop() {
  digitalWrite(M1_1, LOW);
  digitalWrite(M1_2, LOW);
  digitalWrite(M2_1, LOW);
  digitalWrite(M2_2, LOW);
}











// class movement {
//   public:
//     void forward(){
//       digitalWrite(M1_1, HIGH);
//       digitalWrite(M1_2, LOW);
//       digitalWrite(M2_1, HIGH);
//       digitalWrite(M2_2, LOW);
//     }

//     void backward() {
//       digitalWrite(M1_1, LOW);
//       digitalWrite(M1_2, HIGH);
//       digitalWrite(M2_1, LOW);
//       digitalWrite(M2_2, HIGH);
//     }

//     void left() {
//       digitalWrite(M1_1, LOW);
//       digitalWrite(M1_2, HIGH);
//       digitalWrite(M2_1, HIGH);
//       digitalWrite(M2_2, LOW);
//     }

//     void right() {
//       digitalWrite(M1_1, HIGH);
//       digitalWrite(M1_2, LOW);
//       digitalWrite(M2_1, LOW);
//       digitalWrite(M2_2, HIGH);
//     }
// }