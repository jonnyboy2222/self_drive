import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
from PyQt6 import QtCore, QtGui
from PyQt6.QtCore import *

# --- PC1: Serial to PC2 with DB insert and VF response handling ---

import serial
import struct
import socket
import time
import pymysql
from dbutils.pooled_db import PooledDB
import threading

SERIAL_PORT = '/dev/ttyACM0'
# SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 9600
TIMEOUT_S = 1.0

TCP_SERVER_IP = '192.168.0.42'
TCP_SERVER_PORT = 12345

PACKET_HEADER = 0xAA
DB_PACKET_SIZE = 15 # 1(header) + 2(command) + 4(uid) + 8(floats)
VF_PACKET_SIZE = 7
VF_RESPONSE_SIZE = 6 # 1(header) + 2(command) + 1(VF_RESP) + 2 (padding)

# Database connection pool
# db_pool = PooledDB(
#     creator=pymysql,
#     maxconnections=5,
#     mincached=2,
#     host="localhost",
#     user="root",
#     password="4582",
#     database="johnbase",
#     charset="utf8mb4",
#     autocommit=True
# )

db_pool = PooledDB(
    creator=pymysql,
    maxconnections=5,
    mincached=2,
    host="localhost",
    user="root",
    password="0303",
    database="leebase",
    charset="utf8mb4",
    autocommit=True
)

def get_db_connection():
    return db_pool.connection()

def insert_to_db(shock, temp):
    try:
        conn = get_db_connection()
        with conn.cursor() as cur:
            cur.execute("INSERT INTO sensor_data (shock, temperature) VALUES (%s, %s)", (shock, temp))
    except Exception as e:
        print(f"[DB ERROR] {e}")
    finally:
        conn.close()

def read_aligned_packet(ser):
    while True:
        byte = ser.read(1)
        if not byte:
            continue
        if byte[0] == PACKET_HEADER:
            cmd_bytes = ser.read(2)
            if len(cmd_bytes) < 2:
                continue
            command = cmd_bytes.decode("ascii", errors="replace")
            if command == "DB":
                rest = ser.read(DB_PACKET_SIZE - 3)
                if len(rest) == DB_PACKET_SIZE - 3:
                    return byte + cmd_bytes + rest
            elif command == "VF":
                rest = ser.read(VF_PACKET_SIZE - 3)
                if len(rest) == VF_PACKET_SIZE - 3:
                    return byte + cmd_bytes + rest
        else:
            continue

def listen_response(sock, ser):
    while True:
        try:
            resp = sock.recv(VF_RESPONSE_SIZE)
            if len(resp) == VF_RESPONSE_SIZE and resp[0] == PACKET_HEADER:
                command = resp[1:3].decode("ascii", errors="replace")
                if command == "VF":
                    print(f"[RESP] VF response from PC2: {resp}")
                    ser.write(resp)
        except Exception as e:
            print(f"[TCP Read Error] {e}")
            break

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_S) as ser, \
             socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:

            print("Connecting to TCP server...")
            sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
            print("Connected to TCP server.")

            threading.Thread(target=listen_response, args=(sock, ser), daemon=True).start()

            while True:
                packet = read_aligned_packet(ser)
                if not packet or len(packet) not in (DB_PACKET_SIZE, VF_PACKET_SIZE):
                    continue

                command = packet[1:3].decode("ascii", errors="replace")

                if command == "DB":
                    shock = struct.unpack('<f', packet[7:11])[0]
                    temp = struct.unpack('<f', packet[11:15])[0]
                    insert_to_db(shock, temp)
                    print(f"[PC1] Stored DB: Shock={shock:.2f}, Temp={temp:.2f}")

                sock.sendall(packet)
                print(f"[PC1] Forwarded {command} to PC2")

                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[PC1] Exiting.")
    except Exception as e:
        print(f"[PC1 ERROR] {e}")
    finally:
        print("[PC1] Terminated.")


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

        self.sensor_thread = SensorThread()
        self.sensor_thread.new_data.connect(self.update_display)
        self.sensor_thread.start()

        self.main_btn.clicked.connect(self.return_main)

    def update_display(self, shock, temp):
        self.temp_edit.setText(f"{temp: .1f}°C")
        self.shock_edit.setText(f"{shock} times")
        self.speed_edit.setText("속도km/h")

    def return_main(self):
        self.parent.show()
        self.close()


class InfoWindow(QWidget,info_window):
    def __init__(self, parent):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Info")
        self.parent = parent

        self.load_driver_data()
        self.main_btn.clicked.connect(self.return_main)

    def load_driver_data(self):
        try:
            conn = get_db_connection()
            with conn.cursor() as cur:
                cur.execute("SELECT AVG(shock) as Shock, AVG(temperature) as Temp FROM sensor_data ")
                result = cur.fetchall()

                for row in result:
                    self.info_edit.append(f"Temp: {row[0]} \n Shock: {row[1]}")

        except Exception as e:
            print(f"[DB ERROR] {e}")
        finally:
            conn.close()

    def return_main(self):
        self.parent.show()
        self.close()

class SensorThread(QThread):
    new_data = pyqtSignal(float, float)  # shock, temp

    def __init__(self,ser):
        super().__init__()
        self.ser = ser

    def run(self):
        while True:
            shock, temp = self.get_sensor_data()
            if shock is not None and temp is not None:
                insert_to_db(shock, temp)
                self.new_data.emit(shock, temp)  # 시그널 발생
            time.sleep(1)

    def get_sensor_data(self):
        packet = read_aligned_packet(self.ser)
        command = packet[1:3].decode("ascii", errors="replace")
        if command == "DB":
            shock = struct.unpack('<f', packet[7:11])[0]
            temp = struct.unpack('<f', packet[11:15])[0]
            return shock, temp
        return None, None

if __name__ == "__main__":
    app = QApplication(sys.argv)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_S)

    window = MainWindow()
    window.sensor_thread = SensorThread(ser)
    window.show()

    # DB 삽입 스레드 시작
    db_thread = threading.Thread(target=main, daemon=True)
    db_thread.start()


    sys.exit(app.exec())
    
