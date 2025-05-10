#include <Servo.h>
#include <SoftwareSerial.h>

#define BT_RXD 2
#define BT_TXD 3
SoftwareSerial bluetooth(BT_RXD, BT_TXD);

class MotorControl {
  private:
    int in1, in2, in3, in4, in5, in6;

  public:
    MotorControl(int _in1, int _in2, int _in3, int _in4, int _in5, int _in6) {
      in1 = _in1;
      in2 = _in2;
      in3 = _in3;
      in4 = _in4;
      in5 = _in5;
      in6 = _in6;

      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(in3, OUTPUT);
      pinMode(in4, OUTPUT);
      pinMode(in5, OUTPUT);
      pinMode(in6, OUTPUT);
    }

    void moveForward(int speed = 150) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(in5, speed);
      analogWrite(in6, speed);
    }

    void moveBackward(int speed = 150) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(in5, speed);
      analogWrite(in6, speed);
    }

    void stopMotors() {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(in5, 0);
      analogWrite(in6, 0);
    }
};

// 핀 번호 상수
const int IN1 = 12;
const int IN2 = 13;
const int IN3 = 7;
const int IN4 = 8;
const int IN5 = 5;
const int IN6 = 6;


MotorControl motor(IN1, IN2, IN3, IN4, IN5, IN6); // 모터 컨트롤러 객체 생성
Servo steering;

String input = "";

void setup() {
  bluetooth.begin(9600);  
  steering.attach(11); // 서보모터 핀
  steering.write(90); // 초기 각도
}

void loop() {
  while (bluetooth.available()) {
    char c = bluetooth.read();
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
