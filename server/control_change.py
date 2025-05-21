import sys
import serial
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
from PyQt6 import QtCore, QtGui
from PyQt6.QtCore import *
import time
import queue
# --- PC1: Serial to PC2 with DB insert and VF response handling ---
import serial
import struct
import socket
import time
import pymysql
from dbutils.pooled_db import PooledDB
import threading

from collections import deque

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_PORT2 = "/dev/ttyACM1"

BAUD_RATE = 9600
TIMEOUT_S = 1.0

TCP_SERVER_IP = "192.168.2.33"
TCP_SERVER_PORT = 12345

PACKET_HEADER = 0xAA
DB_PACKET_SIZE = 15 # 1(header) + 2(command) + 4(uid) + 8(floats)
VF_PACKET_SIZE = 7
VF_RESPONSE_SIZE = 4 # 1(header) + 2(command) + 1(VF_RESP)
LS_PACKET_SIZE = 4 # 1(header) + 2(command) + 1(VF_RESP)
UR_PACKET_SIZE = 7

temp_queue = queue.Queue()
shock_queue = queue.Queue()
cmd_queue = queue.Queue(maxsize=2)
alc_queue = queue.Queue()
ls_queue = queue.Queue(maxsize=1)
ur_queue = queue.Queue()
uid_queue = queue.Queue()
engine_queue = queue.Queue()


# Database connection pool
db_pool = PooledDB(
    creator=pymysql,
    maxconnections=5,
    mincached=2,
    host="localhost",
    user="root",
    password="4582",
    database="johnbase",
    charset="utf8mb4",
    autocommit=True
)

def get_db_connection():
    return db_pool.connection()
def insert_to_db(uid_hex, shock, temp):
    try:
        conn = get_db_connection()
        with conn.cursor() as cur:
            cur.execute("INSERT INTO sensor_data (uid, shock, temperature) VALUES (%s, %s, %s)", (uid_hex, shock, temp))
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
                    uid = rest[:4]
                    if uid == b'\x00\x00\x00\x00':
                        # print("[RECV] Ignored DB packet with null UID")
                        return
                    packet = byte + cmd_bytes + rest
                    print(f"[RECV] DB Packet: {packet.hex().upper()}")
                    return packet
            elif command == "VF":
                # 먼저 1바이트를 읽어 응답인지 요청인지 판단
                lookahead = ser.read(1)
                if not lookahead:
                    continue
                # 응답이면 (1바이트 추가만 있음 → 총 4바이트)
                if ser.in_waiting == 0:
                    packet = byte + cmd_bytes + lookahead
                    print(f"[RECV] VF Response: {packet.hex().upper()}")
                    alc_queue.put(chr(lookahead[0]))
                # 아니면 요청 (나머지 3바이트 추가로 읽기 → 총 7바이트)
                rest = ser.read(3)
                if len(rest) == 3:
                    packet = byte + cmd_bytes + lookahead + rest
                    print(f"[RECV] VF Request: {packet.hex().upper()}")
                    return packet
            elif command == "LS":
                print("CMD LS received")
                rest = ser.read(LS_PACKET_SIZE - 3)
                if len(rest) == LS_PACKET_SIZE - 3:
                    print(f"Received LS byte : {rest[0]}")
                    ls_queue.put(rest[0])

            elif command == "UR":
                print("CMD UR received")
                rest = ser.read(UR_PACKET_SIZE - 3)
                if len(rest) == UR_PACKET_SIZE - 3:
                    print(f"Received UR byte : {rest}")
                    ur_queue.put(rest)
        else:
            continue
def listen_response(sock, ser):
    sock.settimeout(1.0)
    while True:
        try:
            resp = sock.recv(VF_RESPONSE_SIZE)
            if len(resp) == VF_RESPONSE_SIZE and resp[0] == PACKET_HEADER:
                command = resp[1:3].decode("ascii", errors="replace")
                if command == "VF":
                    print(f"[RESP] VF response from PC2: {resp}")
                    ser.write(resp)
                    uid_queue.put(resp[3])
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[TCP Read Error] {e}")
            break

        # 큐에서 명령을 확인하고, "MB"가 있으면 "MB"를 전송하고, 없으면 "ST"를 전송
        if not cmd_queue.empty():
            # 큐에서 모든 명령을 확인
            queue_items = []
            while not cmd_queue.empty():
                queue_items.append(cmd_queue.get_nowait())
            # "MB"가 있으면 "MB" 보내기, 없으면 "ST" 보내기
            packet = bytearray()
            packet.append(PACKET_HEADER)
            if "MB" in queue_items:
                packet += b'MB'
            else:
                packet += b'ST'
            packet.append(0x00)  # 명령의 끝 부분을 0x00으로 설정
            ser.write(packet)
            #print(f"[CMD] Sent: {packet.hex()}")
        
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
                    uid_hex = packet[3:7].hex().upper()
                    if uid_hex != "00000000":
                        shock = struct.unpack('<f', packet[7:11])[0]
                        temp = struct.unpack('<f', packet[11:15])[0]
                        insert_to_db(uid_hex, shock, temp)
                        shock_queue.put(f"{shock:.2f}")
                        temp_queue.put(f"{temp:.2f}")
                        print(f"[PC1] Stored DB: Shock={shock:.2f}, Temp={temp:.2f}")
                    else:
                        print("[PC1] Ignored DB packet with null UID")
                sock.sendall(packet)
                #print(f"[PC1] Forwarded {command} to PC2")
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[PC1] Exiting.")
    except Exception as e:
        print(f"[PC1 ERROR] {e}")
    finally:
        print("[PC1] Terminated.")

# GUI ---------------------

MAIN_UI = "/home/john/dev_ws/yolo/iot_project/main.ui"
# STATUS_UI = "/home/john/dev_ws/yolo/status.ui"
# INFO_UI = "/home/john/dev_ws/yolo/info.ui"
OUT_DISP_UI = "/home/john/dev_ws/yolo/iot_project/outside_disp.ui"

main_window = uic.loadUiType(MAIN_UI)[0]
# status_window = uic.loadUiType(STATUS_UI)[0]
# info_window = uic.loadUiType(INFO_UI)[0]
out_window = uic.loadUiType(OUT_DISP_UI)[0]

def getDistance():
    try:
        if not ur_queue.empty():
            raw = ur_queue.get_nowait()
            if len(raw) == 4:  # 데이터 길이 확인
                dist = struct.unpack('<f', raw)[0]
                print(f"dist received {dist}")
                return dist
    except Exception as e:
        print(f"[getDistance ERROR] {e}")
    # return None

        
class OutsideDisplay(QWidget, out_window):

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("Outside Display")

        self.uid_check_timer = QTimer()
        self.uid_check_timer.timeout.connect(self.check_uid_queue)
        self.uid_check_timer.start(1000)  # 1초마다 확인

        self.open = False
        
    def check_uid_queue(self):
        try:
            if not uid_queue.empty() and self.open == False:
                self.updateDisplay(uid_queue.queue)

        except queue.Empty:
            pass

    def updateDisplay(self, uid):
        if any(pf == 0x01 for pf in list(uid)[:]):
            self.message = "Welcome Back"
            self.open = True
        else:
            self.message = "Wrong UID"

        self.display.setText(self.message)
        QTimer.singleShot(3000, self.display.clear)

class MainWindow(QWidget, main_window):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("Main")

        self.rc_controller = RCController(instance=self)

        self.power_on = False
        self.light_on = False
        self.alc_test = False

        self.temp = 0

        # 타이머
        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self.update_time)
        self.clock_timer.start(1000)
        self.update_time()

        self.dist = 0

        # 이벤트 연결
        self.power_btn.clicked.connect(self.toggle_power)
        # self.status_btn.clicked.connect(self.show_status)
        # self.info_btn.clicked.connect(self.show_info)

        # 비활성화
        self.power_btn.setEnabled(False)
        self.dist_edit.hide()
        # self.status_btn.setEnabled(False)
        # self.info_btn.setEnabled(False)

        # 데이터 들어있는 queue 주기적으로 체크
        self.data_poll_timer = QTimer()
        self.data_poll_timer.timeout.connect(self.poll_data_from_thread)
        self.data_poll_timer.start(500)

        self.message_manager = MessageManager(self.main_edit)

        self.main_edit.setText("")

        if self.power_on:
            self.poll_data_from_thread()

        self.updateStatus(0, 0)

        # 데이터 들어있는 queue 주기적으로 체크
        self.data_poll_timer2 = QTimer()
        self.data_poll_timer2.timeout.connect(self.poll_data_from_thread2)
        self.data_poll_timer2.start(700)

        self.poll_data_from_thread2()

        self.temp_edit.hide()
        self.shock_edit.hide()
        self.dist_edit.hide()

    def update_time(self):
        self.time_edit.setText(QTime.currentTime().toString("hh:mm:ss"))


    def toggle_power(self):
        if self.power_on == False:
            self.power_btn.setText("OFF")
            self.temp_edit.show()
            self.shock_edit.show()
            self.updateDisplay("Engine ON \n Drive Safe")
            engine_queue.put("ON")
            self.power_on = True

        else:
            self.power_btn.setText("ON")
            self.temp_edit.hide()
            self.shock_edit.hide()
            self.updateDisplay("Hope to see you again")
            engine_queue.put("OFF")
            self.power_on == False

    def updateDisplay(self, message):
        self.message_manager.show_message(message)

    def poll_data_from_thread(self):
        try:
            if self.power_on == False and self.alc_test == False: 
                if self.checkAuth() == True:
                    self.message = "Hello! Drive Safe"
                    self.updateDisplay(self.message)
                    self.power_btn.setEnabled(True)
                else:
                    self.message = "Please authenticate first"
                    self.main_edit.setText(self.message)
                    if self.alc_test == True:
                        self.message = "You are drunk!!!!!"
                        self.updateDisplay(self.message)

                    
                        

            else:
                if any(cmd == "MB" for cmd in list(cmd_queue.queue)[:2]):
                    self.message = "You are Moving Backward"
                    self.main_edit.setText(self.message)

                    if not ur_queue.empty():
                        raw = ur_queue.get_nowait()
                        if len(raw) == 4:
                            self.dist = struct.unpack('<f', raw)[0]
                            print(f"dist received {self.dist}")
                            
                            # UI 갱신을 메인 스레드에서 실행
                            # self.main_edit.setText(f"Distance: {self.dist :.1f} cm")
                            self.dist_edit.show()
                            self.dist_edit.setText(f"Distance: {self.dist :.1f} cm")

                            self.checkDist(self.dist)


                            # self.updateDisplay(f"{self.dist} cm")
                            # if self.dist <= 10.0:
                            #     self.updateDisplay("WARNING: Too close")

                            # self.checkDist(self.dist)

                    else:
                        self.checkDist(dist=None)

                    
                    
                else:
                    self.main_edit.clear()


                light_value = self.checkLight()

                if light_value == 1:
                    if not self.light_on:
                        self.message = "HeadLight ON"
                        self.light_edit.setText(self.message)
                        self.light_on = True
                elif light_value == 0:
                    if self.light_on:
                        self.message = "HeadLight OFF" 
                        self.light_edit.setText(self.message)
                        self.light_on = False

        except queue.Empty:
            pass

    def checkDist(self, dist=None):
        if dist == None:
            #print("Dist None")
            return True
        elif dist > 40:
            #print("Dist > 10")
            return True
        else:
            #print("Dist < 10")
            return False

    def checkAuth(self):
        if any(pf == 'P' for pf in list(alc_queue.queue)[:]):
            self.alc_test = True
            return True
        elif any(pf == 'F' for pf in list(alc_queue.queue)[:]):
            self.alc_test = True
            return False

    # def checkDist(self):
        
        # self.update(f"{dist:.1f} cm")

        # if dist <= 10.0:
        #     self.updateDisplay("WARNING: Too close")
        # elif dist <= self.temp:
        #     self.updateDisplay("Getting closer")
            
        # self.temp = dist

        # try:
        #     if not ur_queue.empty():
        #         raw = ur_queue.get()  # 4바이트 바이트열
        #         dist = struct.unpack('f', raw)[0]  # little endian float 추출
        #         self.updateDisplay(f"{dist:.1f} cm")

        #         if dist <= 10.0:
        #             self.updateDisplay("WARNING: Too close")
        #         elif dist <= self.temp:
        #             self.updateDisplay("Getting closer")
                
        #         self.temp = dist
        # except queue.Empty:
        #     pass


    def checkLight(self):
        try:
            ls = ls_queue.get_nowait()
            if ls == 1:
                return 1
            else:
                return 0
        except queue.Empty:
            pass
            
    def load_driver_data(self):
        try:
            conn = get_db_connection()
            with conn.cursor() as cur:
                cur.execute("SELECT temperature, shock FROM sensor_data ORDER BY id DESC LIMIT 1")
                results = cur.fetchall()
                self.temp_edit.clear()
                self.shock_edit.clear()

                for temp, shock in results:
                    self.updateStatus(temp, shock)

        except Exception as e:
            print(f"[DB LOAD ERROR] {e}")
        finally:
            conn.close()

    def updateStatus(self, temp, shock):
        self.temp_edit.setText(f'{temp} °C')
        self.shock_edit.setText(f'{shock} times')

    def poll_data_from_thread2(self):
        try:
            temp = temp_queue.get_nowait()
            shock = shock_queue.get_nowait()
            self.updateStatus(temp, shock)

        except queue.Empty:
            pass

class RCController(QWidget):
    def __init__(self, instance, parent=None):
        super().__init__(parent)
        self.instance = instance
        self.setWindowTitle("RC카 제어기")
        self.setFixedSize(200, 200)

        self.ser = None
        try:
            self.ser = serial.Serial(SERIAL_PORT2, BAUD_RATE, timeout=TIMEOUT_S)
            print(f"Successfully connected to {SERIAL_PORT2}")
            time.sleep(1)
        except serial.SerialException as e:
            print(f"Error opening serial port {SERIAL_PORT2} : {e}. Please check the connection and permissions.")

        self.keys_pressed = set()
        self.speed = 150
        self.speed_dir = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_command)
        self.timer.start(50)  # 20 FPS

        self.auth_result = False

        
        


    def keyPressEvent(self, event):
        self.keys_pressed.add(event.key())

    def keyReleaseEvent(self, event):
        self.keys_pressed.discard(event.key())

    def update_command(self):
        # if self.main_window_instance is None:
        #     # print("[ERROR] main_window is not set in update_command")
        #     return
        if not self.ser or not self.ser.is_open:
            print("Serial port /dev/ttyACM1 not available. Cannot send command.")
            return
        try:
            if (self.checkAuth() == True) and (self.checkEngine() == True):
                # 모터 제어 (전진/후진)
                if Qt.Key.Key_W in self.keys_pressed:
                    self.ser.write(b'MF\n')
                    # print(self.ser.readline(), 'MF')

                    check = True
                    if cmd_queue.full():
                        cmd_queue.get_nowait()
                    if check:
                        cmd_queue.put("MF")
                        check = False

                elif Qt.Key.Key_S in self.keys_pressed:
                    # dist = None

                    # if not ur_queue.empty():
                    #     raw = ur_queue.get_nowait()
                    #     if len(raw) == 4:  # 데이터 길이 확인
                    #         dist = struct.unpack('<f', raw)[0]
                    #         print(f"dist received {dist}")

                    if self.instance.checkDist(self.instance.dist):
                        self.ser.write(b'MB\n')
                    else:
                        #print("Motor Stop")
                        self.ser.write(b'MS\n')
                    # print(self.ser.readline(), 'MB')

                    check = True
                    if cmd_queue.full():
                        cmd_queue.get_nowait()
                    if check:
                        cmd_queue.put("MB")
                        check = False

                elif Qt.Key.Key_A in self.keys_pressed:
                    self.ser.write(b'TL\n')
                    # print(self.ser.readline(), 'TL')

                    check = True
                    if cmd_queue.full():
                        cmd_queue.get_nowait()
                    if check:
                        cmd_queue.put("TL")
                        check = False

                elif Qt.Key.Key_D in self.keys_pressed:
                    self.ser.write(b'TR\n')
                    # print(self.ser.readline(), 'TR')

                    check = True
                    if cmd_queue.full():
                        cmd_queue.get_nowait()
                    if check:
                        cmd_queue.put("TR")
                        check = False

                elif Qt.Key.Key_X in self.keys_pressed:
                    self.ser.write(b'MS\n')
                    # print(self.ser.readline(), 'MS')

                    check = True
                    if cmd_queue.full():
                        cmd_queue.get_nowait()
                    if check:
                        cmd_queue.put("ST")
                        check = False

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
            print("Closing serial port /dev/ttyACM1.")
            try:
                self.ser.write(b'S\n') 
            except serial.SerialException:
                pass 
            self.ser.close()
        super().closeEvent(event)

    def checkAuth(self):
        if any(pf == 'P' for pf in list(alc_queue.queue)[:]):
            return True
        else:
            return False
    
    def checkEngine(self):
        if any(pf == 'ON' for pf in list(engine_queue.queue)[:]):
            return True
        else:
            return False

    # def checkDist(self, dist=None):
    #     if dist <= 10.0:
    #         return False
    #     elif dist == None:
    #         return True
    #     else:
    #         return True

        # try:
        #     if not ur_queue.empty():
        #         raw = ur_queue.get()  # 4바이트 바이트열
        #         dist = struct.unpack('f', raw)[0]  # little endian float 추출

        #         if dist <= 10.0:
        #             return False
                
        # except queue.Empty:
        #     pass

            
class MessageManager:
    def __init__(self, text_edit: QTextEdit):
        self.text_edit = text_edit
        self.message_queue = []
        self.current_message = None

        self.display_timer = QTimer()
        self.display_timer.setSingleShot(True)
        self.display_timer.timeout.connect(self.remove_current_message)

    def show_message(self, msg: str):
        if self.current_message is None:
            # 현재 표시 중인 메시지가 없으면 바로 표시
            self.current_message = msg
            self.text_edit.setPlainText(msg)
            self.display_timer.start(1000)
        else:
            # 표시 중이면 큐에 추가하고 두 줄로 출력
            self.message_queue.append(msg)
            self._refresh_display()

    def remove_current_message(self):
        # 현재 메시지 제거
        if self.message_queue:
            # 대기 메시지 있으면 교체
            self.current_message = self.message_queue.pop(0)
            self._refresh_display()
            self.display_timer.start(2000)
        else:
            # 없으면 모두 지움
            self.current_message = None
            self.text_edit.clear()

    def _refresh_display(self):
        lines = [self.current_message] if self.current_message else []
        if self.message_queue:
            lines.append(self.message_queue[0])
        self.text_edit.setPlainText("\n".join(lines))

if __name__ == "__main__":
    threading.Thread(target=main, daemon=True).start()

    app = QApplication(sys.argv)
    # window1 = RCController()
    # window1.show()

    window2 = OutsideDisplay()
    window2.show()

    window = MainWindow()
    window.show()

    control = RCController(instance=window)
    control.show()
    
    sys.exit(app.exec())


    
