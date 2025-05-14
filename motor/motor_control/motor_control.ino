class MotorControl {
  private:
    int in1, in2, in3, in4, in5, in6, in7, in8, in9, in10;

  public:
    MotorControl(int _in1, int _in2, int _in3, int _in4, int _in5, int _in6, int _in7, int _in8, int _in9, int _in10) {
      in1 = _in1;
      in2 = _in2;
      in3 = _in3;
      in4 = _in4;
      in5 = _in5;
      in6 = _in6;
      in7 = _in7;
      in8 = _in8;
      in9 = _in9;
      in10 = _in10;

      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(in3, OUTPUT);
      pinMode(in4, OUTPUT);
      pinMode(in5, OUTPUT);
      pinMode(in6, OUTPUT);
      pinMode(in7, OUTPUT);
      pinMode(in8, OUTPUT);
      pinMode(in9, OUTPUT);
      pinMode(in10, OUTPUT);
    }

    void moveForward(int speed = 150) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(in5, speed);
      analogWrite(in6, speed);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }

    void moveBackward(int speed = 150) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(in5, speed);
      analogWrite(in6, speed);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }

    void turnLeft(int speed = 150) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(in5, speed);
      analogWrite(in6, speed);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }

    void turnRight(int speed = 150) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(in5, speed);
      analogWrite(in6, speed);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }


    void stopMotors() {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(in5, 0);
      analogWrite(in6, 0);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }
};

// 핀 번호 상수
const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 6;
const int IN4 = 7;
const int IN5 = 3;
const int IN6 = 9;
const int IN7 = 8;
const int IN8 = 10;
const int IN9 = 11;
const int IN10 = 12;


MotorControl motor(IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, IN9, IN10); // 모터 컨트롤러 객체 생성
int speed = 150;

String input = "";

void setup() {
  Serial.begin(9600);
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(input);
      input = "";
    } else {
      input += c;
    }

    Serial.print(c);
  }
}

void handleCommand(String cmd) {
  if (cmd.startsWith("X")) {
    speed = cmd.substring(1).toInt();
  } else if (cmd.length() > 0) {
    char action = cmd.charAt(1);
    switch (action) {
      case 'F': motor.moveForward(speed); break;
      case 'B': motor.moveBackward(speed); break;
      case 'L': motor.turnLeft(); break;
      case 'R': motor.turnRight(); break;
      case 'S': motor.stopMotors(); break;
    }
  }
}
