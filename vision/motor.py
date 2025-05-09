import sys
import serial
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import Qt, QTimer

# ser = serial.Serial('/dev/rfcomm0', 9600, timeout=1)

class RCController(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RC카 제어기")
        self.setFixedSize(200, 200)

        self.ser = None
        try:
            self.ser = serial.Serial('/dev/rfcomm0', 9600, timeout=1)
            print("Successfully connected to /dev/rfcomm0")
        except serial.SerialException as e:
            print(f"Error opening serial port /dev/rfcomm0: {e}. Please check the connection and permissions.")

        self.keys_pressed = set()
        self.servo_angle = 90
        self.servo_dir = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_command)
        self.timer.start(50)  # 20 FPS

    def keyPressEvent(self, event):
        self.keys_pressed.add(event.key())

    def keyReleaseEvent(self, event):

        self.keys_pressed.discard(event.key())

    def update_command(self):
        if not self.ser or not self.ser.is_open:
            print("Serial port /dev/rfcomm0 not available. Cannot send command.")
            return 
        try:
            # 모터 제어 (전진/후진)
            if Qt.Key.Key_W in self.keys_pressed:
                self.ser.write(b'F\n')
            elif Qt.Key.Key_S in self.keys_pressed:
                self.ser.write(b'B\n')
            else:
                self.ser.write(b'S\n')

            # 서보 조향 제어
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
                self.ser.write(command.encode())
        except serial.SerialException as e:
            print(f"Serial write error on /dev/rfcomm0: {e}. Connection may be lost.")
            if self.ser and self.ser.is_open:
                self.ser.close() 

    def closeEvent(self, event):
        """Properly close the serial port when the application exits."""
        if self.ser and self.ser.is_open:
            print("Closing serial port /dev/rfcomm0.")
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



         