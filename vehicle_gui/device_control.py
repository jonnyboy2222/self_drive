import sys
import serial
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
from PyQt6 import QtCore, QtGui
from PyQt6.QtCore import *
import time

MAIN_UI = "/home/lee/project/self_drive/vehicle_gui/main.ui"
STATUS_UI = "/home/lee/project/self_drive/vehicle_gui/status.ui"
INFO_UI = "/home/lee/project/self_drive/vehicle_gui/info.ui"

main_window = uic.loadUiType(MAIN_UI)[0]
status_window = uic.loadUiType(STATUS_UI)[0]
info_window = uic.loadUiType(INFO_UI)[0]

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

class MainWindow(QWidget, main_window):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("Main")

        self.power_on = False

        # 타이머
        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self.update_time)
        self.clock_timer.start(1000)
        self.update_time()

        self.sensor_timer = QTimer()
        # self.sensor_timer.timeout.connect(self.update_sensor)

        # 이벤트 연결
        self.power_btn.clicked.connect(self.toggle_power)
        self.status_btn.clicked.connect(self.show_status)
        self.info_btn.clicked.connect(self.show_info)

        # 비활성화
        self.status_btn.setEnabled(False)
        self.info_btn.setEnabled(False)

    def update_time(self):
        self.time_edit.setText(QTime.currentTime().toString("hh:mm:ss"))

    def toggle_power(self):
        self.power_on = not self.power_on
        if self.power_on:
            self.power_btn.setText("OFF")
            self.status_btn.setEnabled(True)
            self.info_btn.setEnabled(True)
            self.sensor_timer.start(1000)
        else:
            self.power_btn.setText("ON")
            self.status_btn.setEnabled(False)
            self.info_btn.setEnabled(False)
            self.sensor_timer.stop()

    def show_status(self):
        self.status_window = StatusWindow(self)
        self.status_window.show()
        self.hide()

    def show_info(self):
        self.info_window = InfoWindow(self)
        self.info_window.show()
        self.hide()


class StatusWindow(QWidget, status_window):
    def __init__(self, parent):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Status")
        self.parent = parent

        self.temp_edit.setText("온도°C")
        self.shock_edit.setText("충격")
        self.speed_edit.setText("속도km/h")

        self.main_btn.clicked.connect(self.return_main)

    def return_main(self):
        self.parent.show()
        self.close()


class InfoWindow(QWidget,info_window):
    def __init__(self, parent):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Info")
        self.parent = parent

        self.driverText: QTextEdit = self.findChild(QTextEdit, "driverText")
        self.backButton: QPushButton = self.findChild(QPushButton, "backButton")

        self.load_driver_data()
        self.main_btn.clicked.connect(self.return_main)

    def load_driver_data(self):
        # cur = local.cursor(buffered=True)
        # cur.execute('SELECT temp, shcok FROM sensor_data')
        # result = cur.fetchall()

        # self.driverText.clear()

        # for row in result:
        #     self.driverText.append(f"Temp: {row[0]}, Shock: {row[1]}")
        # cur.close()
        self.info_edit.setText("info")

    def return_main(self):
        self.parent.show()
        self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window1 = RCController()
    window1.show()

    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


    
