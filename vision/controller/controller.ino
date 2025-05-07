#include <Servo.h>

const int IN1 = 5;
const int IN2 = 6;
const int IN3 = 9;
const int IN4 = 10;

Servo steering;

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  steering.attach(3); // 서보는 D3번 핀
  steering.write(90); // 초기 각도
}

String input = "";

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}

void handleCommand(String cmd) {
  if (cmd.startsWith("X")) {
    int angle = cmd.substring(1).toInt();
    steering.write(angle);
  } else if (cmd.length() > 0) {
    char action = cmd.charAt(0);
    switch (action) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'S': stopMotors(); break;
    }
  }
}

void moveForward() {
  analogWrite(IN1, 150);
  analogWrite(IN2, 0);
  analogWrite(IN3, 150);
  analogWrite(IN4, 0);
}

void moveBackward() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 150);
  analogWrite(IN3, 0);
  analogWrite(IN4, 150);
}

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}
