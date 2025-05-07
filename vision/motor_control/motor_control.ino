#include <Servo.h>

class MotorControl {
  private:
    int in1, in2, in3, in4;

  public:
    MotorControl(int _in1, int _in2, int _in3, int _in4) {
      in1 = _in1;
      in2 = _in2;
      in3 = _in3;
      in4 = _in4;

      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(in4, OUTPUT);
    }

    void moveForward(int speed = 150) {
      analogWrite(in1, speed);
      analogWrite(in2, 0);
      analogWrite(in3, speed);
      analogWrite(in4, 0);
    }

    void moveBackward(int speed = 150) {
      analogWrite(in1, 0);
      analogWrite(in2, speed);
      analogWrite(in3, 0);
      analogWrite(in4, speed);
    }

    void stopMotors() {
      analogWrite(in1, 0);
      analogWrite(in2, 0);
      analogWrite(in3, 0);
      analogWrite(in4, 0);
    }
};

// 핀 번호 상수
const int IN1 = 5;
const int IN2 = 6;
const int IN3 = 10;
const int IN4 = 11;

MotorControl motor(IN1, IN2, IN3, IN4); // 모터 컨트롤러 객체 생성
Servo steering;

String input = "";

void setup() {
  Serial.begin(9600);
  steering.attach(3); // 서보모터 핀
  steering.write(90); // 초기 각도
}

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
      case 'F': motor.moveForward(); break;
      case 'B': motor.moveBackward(); break;
      case 'S': motor.stopMotors(); break;
    }
  }
}
