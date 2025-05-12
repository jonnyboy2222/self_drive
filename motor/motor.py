import sys
import serial
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import Qt, QTimer
import time
# ser = serial.Serial('/dev/rfcomm0', 9600, timeout=1)

class RCController(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RC카 제어기")
        self.setFixedSize(200, 200)

        self.ser = None
        try:
            self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
            print("Successfully connected to /dev/ttyACM0")
            time.sleep(1)
        except serial.SerialException as e:
            print(f"Error opening serial port /dev/ttyACM0: {e}. Please check the connection and permissions.")

        self.keys_pressed = set()
        self.speed = 150
        self.speed_dir = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_command)
        self.timer.start(50)  # 20 FPS

    def keyPressEvent(self, event):
        self.keys_pressed.add(event.key())

    def keyReleaseEvent(self, event):

        self.keys_pressed.discard(event.key())

    def update_command(self):
        if not self.ser or not self.ser.is_open:
            print("Serial port /dev/ttyACM0 not available. Cannot send command.")
            return 
        try:
            # 모터 제어 (전진/후진)
            if Qt.Key.Key_W in self.keys_pressed:
                self.ser.write(b'MF\n')
                print(self.ser.readline(), 'MF')
            elif Qt.Key.Key_S in self.keys_pressed:
                self.ser.write(b'MB\n')
                print(self.ser.readline(), 'MB')
            elif Qt.Key.Key_A in self.keys_pressed:
                self.ser.write(b'TL\n')
                print(self.ser.readline(), 'TL')
            elif Qt.Key.Key_D in self.keys_pressed:
                self.ser.write(b'TR\n')
                print(self.ser.readline(), 'TR')
            else:
                self.ser.write(b'MS\n')
                print(self.ser.readline(), 'MS')

            # 속도 제어
            if Qt.Key.Key_Q in self.keys_pressed and Qt.Key.Key_E not in self.keys_pressed:
                self.speed_dir = -1
            elif Qt.Key.Key_E in self.keys_pressed and Qt.Key.Key_Q not in self.keys_pressed:
                self.speed_dir = 1
            else:
                self.speed_dir = 0

            if self.speed_dir != 0:
                self.speed += self.speed_dir * 10
                command = f"X{self.speed}\n"
                self.ser.write(command.encode())
                print(self.speed)
        except serial.SerialException as e:
            print(f"Serial write error on /dev/ttyACM0: {e}. Connection may be lost.")
            if self.ser and self.ser.is_open:
                self.ser.close() 

    def closeEvent(self, event):
        """Properly close the serial port when the application exits."""
        if self.ser and self.ser.is_open:
            print("Closing serial port /dev/ttyACM0.")
            try:
                self.ser.write(b'S\n') 
            except serial.SerialException:
                pass 
            self.ser.close()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)        
    window = RCController()
    window.show()
    sys.exit(app.exec())



         