const int M1_1 = 5;
const int M1_2 = 6;
const int M2_1 = 9;
const int M2_2 = 10;

void setup() {
  // put your setup code here, to run once:
  pinMode(M1_1, OUTPUT);
  pinMode(M1_2, OUTPUT);
  pinMode(M2_1, OUTPUT);
  pinMode(M2_2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int input;

  if (input == 1) {
    foward()
  }

  if (input == 1) {
    backward()
  }

  if (input == 1) {
    right()
  }

  if (input == 1) {
    left()
  }
}

void foward() {
  analogWrite(M1_1, 150);
  analogWrite(M1_2, 0);
  analogWrite(M2_1, 150);
  analogWrite(M2_2, 0);
}

void backward() {
  analogWrite(M1_1, 150);
  analogWrite(M1_2, 0);
  analogWrite(M2_1, 150);
  analogWrite(M2_2, 0);
}

void right() {
  analogWrite(M1_1, 0);
  analogWrite(M1_2, 150);
  analogWrite(M2_1, 150);
  analogWrite(M2_2, 0);
}

void left() {
  analogWrite(M1_1, 150);
  analogWrite(M1_2, 0);
  analogWrite(M2_1, 0);
  analogWrite(M2_2, 15);
}