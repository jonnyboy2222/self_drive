#include <Servo.h>

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
const int IN1 = 22;
const int IN2 = 23;
const int IN3 = 24;
const int IN4 = 25;
const int IN5 = 5;
const int IN6 = 6;


MotorControl motor(IN1, IN2, IN3, IN4, IN5, IN6); // 모터 컨트롤러 객체 생성
Servo steering;

char cmd_buffer[10];

void setup() {
  Serial.begin(9600);  
  steering.attach(9); // 서보모터 핀
  steering.write(90); // 초기 각도
}

void loop() {
  // while (Serial.available()) {
  //   char buffer[8];
  //   int c = Serial.readBytesUntil('\n', buffer, 2);
  //   if (c > 0) {
  //     Serial.println(buffer);
  //   }
  //   else{
  //     Serial.println("ERROR");
  //   }

  //   if (c == '\n') {
  //     handleCommand(input);
  //     input = "";
  //   } else {
  //     input += c;
  //   }
  // }

  if (Serial.available() > 0) {
    int bytesRead = Serial.readBytesUntil('\n', cmd_buffer, sizeof(cmd_buffer)-1);
    Serial.println(bytesRead);

    if (bytesRead > 0) {
      cmd_buffer[bytesRead] = '\0';
      handleCommand(String(cmd_buffer));
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
