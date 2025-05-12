import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
from PyQt6 import QtCore, QtGui
from PyQt6.QtCore import *

import mysql.connector

# local = mysql.connector.connect(
#     host = 'localhost',
#     port = '3306',
#     user = 'root',
#     database = 'leebase',
#     password = '0303'
# )

MAIN_UI = "/home/lee/project/self_drive/vehicle_gui/main.ui"
STATUS_UI = "/home/lee/project/self_drive/vehicle_gui/status.ui"
INFO_UI = "/home/lee/project/self_drive/vehicle_gui/info.ui"

main_window = uic.loadUiType(MAIN_UI)[0]
status_window = uic.loadUiType(STATUS_UI)[0]
info_window = uic.loadUiType(INFO_UI)[0]

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

    # def update_sensor(self):
    #     value = "Value"
    #     self.sensorText.append(value)

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
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


    
