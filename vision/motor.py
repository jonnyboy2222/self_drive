import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6.QtCore import Qt, QTimer
from PyQt6 import uic
import serial

# # ui 파일 연결 - 코드 파일과 같은 폴더 내에 위치
# from_class = uic.loadUiType("calculator.ui")[0]

# 화면 클래스
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

class RCController(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CONTROLLER")
        self.setFixedSize(200, 200)

        self.keys_pressed = set()
        self.servo_angle = 90
        self.servo_dir = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_command)
        self.timer.start(50)

    def keyPressEvent(self, event):
        self.keys_pressed.add(event.key())

    def keyReleaseEvent(self, event):
        self.keys_pressed.discard(event.key())

    def update_command(self):
        if Qt.Key.Key_W in self.keys_pressed:
            ser.write(b'F\n')
        elif Qt.Key.Key_S in self.keys_pressed:
            ser.write(b'B\n')
        else:
            ser.write(b'S\n')

        if Qt.Key.Key_A in self.keys_pressed and Qt.Key.Key_D not in self.keys_pressed:
            self.servo_dir = -1
        elif Qt.Key.Key_D in self.keys_pressed and Qt.Key.Key_A not in self.keys_pressed:
            self.servo_dir = 1
        else:
            self.servo_dir = 0

        if self.servo_dir != 0:
            self.servo_angle += self.servo_dir * 3
            self.servo_angle = max(45, min(135, self.servo_angle))
            command = f"X{self.servo_angle}\n"
            ser.write(command.encode())

# Python 메인 함수
if __name__ == "__main__":
    app = QApplication(sys.argv) # 프로그램 실행
    window = RCController()     # 화면 클래스 생성
    window.show()              # 프로그램 화면 보이기
    sys.exit(app.exec())         # 프로그램 종료까지 동작
    ser.close()