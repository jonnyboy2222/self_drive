import sys
import serial
import struct
import socket
import time
import pymysql 
from dbutils.pooled_db import PooledDB 
import threading

from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QTextEdit, QMessageBox
from PyQt6.QtGui import QKeyEvent
from PyQt6 import uic
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, QTime

# Serial port for Arduino communication
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 9600
SERIAL_TIMEOUT_S = 1.0 

# TCP Server (PC2) details 
TCP_SERVER_IP = '192.168.0.42'
TCP_SERVER_PORT = 12345

# Packet structure constants 
PACKET_HEADER = 0xAA
DB_PACKET_SIZE = 15  # 1(header) + 2(command) + 4(uid) + 4(float) + 4(float)
VF_PACKET_SIZE = 7   # 1(header) + 2(command) + 4(uid)
VF_RESPONSE_SIZE = 6 # 1(header) + 2(command) + 1(result) + 2(padding)

# --- UI File Paths ---
MAIN_UI_PATH = "/home/lee/project/self_drive/vehicle_gui/main.ui"
STATUS_UI_PATH = "/home/lee/project/self_drive/vehicle_gui/status.ui"
INFO_UI_PATH = "/home/lee/project/self_drive/vehicle_gui/info.ui"

# --- Load UI ---
try:
    main_window_ui, _ = uic.loadUiType(MAIN_UI_PATH)
    status_window_ui, _ = uic.loadUiType(STATUS_UI_PATH)
    info_window_ui, _ = uic.loadUiType(INFO_UI_PATH)
except FileNotFoundError as e:
    print(f"CRITICAL ERROR: UI file not found: {e}. Please check paths.")
    sys.exit(1)


# --- Database Setup (from control_pc.py) ---
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

def insert_to_local_db(shock, temp):
    conn = None
    try:
        conn = get_db_connection()
        with conn.cursor() as cur:
            cur.execute("INSERT INTO sensor_data (shock, temperature) VALUES (%s, %s)", (shock, temp))
        # print(f"[DB_LOCAL] Stored: Shock={shock:.2f}, Temp={temp:.2f}")
    except Exception as e:
        print(f"[DB_LOCAL ERROR] Failed to insert shock={shock}, temp={temp}: {e}")
    finally:
        if conn:
            conn.close()

# --- Serial Packet Reading Function (from control_pc.py) ---
def read_aligned_packet(ser_conn):
    """
    Reads data from serial port until a valid packet header and command are found.
    Returns the full packet (bytearray) or None if timeout/error.
    """
    while True: 
        try:
            byte = ser_conn.read(1) 
            if not byte: 
                return None
            
            if byte[0] == PACKET_HEADER:
                cmd_bytes = ser_conn.read(2)
                if len(cmd_bytes) < 2: 
                    # print("read_aligned_packet: Incomplete command read after header.")
                    continue 
                
                command = cmd_bytes.decode("ascii", errors="ignore") 
                if command == "DB":
                    rest = ser_conn.read(DB_PACKET_SIZE - 3)
                    if len(rest) == DB_PACKET_SIZE - 3:
                        return byte + cmd_bytes + rest
                    else:
                        # print(f"read_aligned_packet: Incomplete DB packet. Expected {DB_PACKET_SIZE-3}, got {len(rest)}")
                        continue 
                elif command == "VF":
                    rest = ser_conn.read(VF_PACKET_SIZE - 3) 
                    if len(rest) == VF_PACKET_SIZE - 3:
                        return byte + cmd_bytes + rest
                    else:
                        # print(f"read_aligned_packet: Incomplete VF packet. Expected {VF_PACKET_SIZE-3}, got {len(rest)}")
                        continue 
                # else:
                    # print(f"read_aligned_packet: Unknown command '{command}' after header.")
                    # Continue searching for a new PACKET_HEADER
            # else:
                # print(f"read_aligned_packet: Discarding byte {byte.hex()}, not header.")
                # Continue reading until header or timeout
        except serial.SerialTimeoutException:
            return None 
        except Exception as e:
            print(f"read_aligned_packet: Error during serial read: {e}")
            return None


# --- Backend Thread for Serial Input, TCP Communication, and DB ---
class BackendThread(QThread):
    new_sensor_data = pyqtSignal(float, float)  
    log_message = pyqtSignal(str)               

    def __init__(self, serial_instance, parent=None):
        super().__init__(parent)
        self.ser = serial_instance
        self.sock = None
        self.tcp_listener_thread = None
        self._is_running = True

    def _listen_tcp_responses(self):
        if not self.sock:
            self.log_message.emit("[TCP Listener] Socket not initialized.")
            return
        self.log_message.emit("[TCP Listener] Started.")
        while self._is_running:
            try:
                resp = self.sock.recv(VF_RESPONSE_SIZE) 
                if not resp:  
                    if self._is_running: self.log_message.emit("[TCP Listener] PC2 connection closed.")
                    break
                if len(resp) == VF_RESPONSE_SIZE and resp[0] == PACKET_HEADER:
                    command = resp[1:3].decode("ascii", errors="ignore")
                    if command == "VF":
                        self.log_message.emit(f"[TCP Listener] VF response from PC2: {resp.hex().upper()}")
                        if self.ser and self.ser.is_open:
                            self.ser.write(resp)  
            except socket.timeout:
                continue 
            except Exception as e:
                if self._is_running: 
                    self.log_message.emit(f"[TCP Listener] Read Error: {e}")
                break
        self.log_message.emit("[TCP Listener] Stopped.")

    def run(self):
        self.log_message.emit("BackendThread: Started.")
        try:
            self.log_message.emit(f"BackendThread: Connecting to TCP server {TCP_SERVER_IP}:{TCP_SERVER_PORT}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3.0) 
            self.sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
            self.log_message.emit("BackendThread: Connected to TCP server.")

            self.tcp_listener_thread = threading.Thread(target=self._listen_tcp_responses, daemon=True)
            self.tcp_listener_thread.start()

            while self._is_running:
                if not self.ser or not self.ser.is_open:
                    # self.log_message.emit("BackendThread: Serial port not open. Waiting...")
                    time.sleep(0.5) 
                    continue

                try:
                    packet = read_aligned_packet(self.ser)
                    if not packet: 
                        continue
                    if not self._is_running: break 

                    command = packet[1:3].decode("ascii", errors="ignore")
                    # self.log_message.emit(f"BackendThread: Read packet type '{command}' from Arduino.")

                    if command == "DB":
                        if len(packet) == DB_PACKET_SIZE:
                            shock = struct.unpack('<f', packet[7:11])[0]
                            temp = struct.unpack('<f', packet[11:15])[0]
                            insert_to_local_db(shock, temp)
                            self.new_sensor_data.emit(shock, temp)
                            # self.log_message.emit(f"BackendThread: DB data processed: Shock={shock:.2f}, Temp={temp:.2f}")
                            if self.sock:
                                self.sock.sendall(packet)
                                # self.log_message.emit("BackendThread: Forwarded DB packet to PC2.")
                        else:
                            self.log_message.emit(f"BackendThread: Malformed DB packet, len {len(packet)}")
                    elif command == "VF":
                        if len(packet) == VF_PACKET_SIZE:
                            if self.sock:
                                self.sock.sendall(packet)
                                # self.log_message.emit("BackendThread: Forwarded VF packet to PC2.")
                        else:
                            self.log_message.emit(f"BackendThread: Malformed VF packet, len {len(packet)}")
                    # else:
                        # self.log_message.emit(f"BackendThread: Unknown/ignored packet type '{command}' from Arduino.")

                except serial.SerialTimeoutException:
                    continue
                except Exception as e:
                    if self._is_running:
                        self.log_message.emit(f"BackendThread: Error in serial processing loop: {e}")
                    time.sleep(0.1) 

        except socket.error as e:
            self.log_message.emit(f"BackendThread: TCP connection error: {e}")
        except Exception as e: 
            self.log_message.emit(f"BackendThread: Critical error in run setup: {e}")
        finally:
            self._is_running = False 
            if self.tcp_listener_thread and self.tcp_listener_thread.is_alive():
                self.tcp_listener_thread.join(timeout=1.0)
            if self.sock:
                self.sock.close()
                self.sock = None
            self.log_message.emit("BackendThread: Stopped.")

    def stop(self):
        self.log_message.emit("BackendThread: Stop requested.")
        self._is_running = False


# --- GUI Classes ---
class MainWindow(QWidget, main_window_ui):
    def __init__(self, serial_instance, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.setWindowTitle("Vehicle Control and Monitoring")

        self.ser = serial_instance
        self.power_on = False
        self.backend_thread = None
        self.status_window_instance = None
        self.info_window_instance = None

        # RC Control variables 
        self.keys_pressed = set()
        self.servo_angle = 90  
        self.motor_speed_setting = 150 

        # RC Command Timer
        if (auth == True): # auth: 모터 제어 허용 (음주 통과, event 발생 x)
            self.rc_timer = QTimer(self)
            self.rc_timer.timeout.connect(self.update_rc_command)

        # Clock Timer
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self.update_time_display)
        self.clock_timer.start(1000)
        self.update_time_display()

        # Connect GUI element signals to methods
        if hasattr(self, 'power_btn'):
            self.power_btn.clicked.connect(self.toggle_power)
        else:
            print("Warning: MainWindow UI missing 'power_btn'")
        
        if hasattr(self, 'status_btn'):
            self.status_btn.clicked.connect(self.show_status_window)
            self.status_btn.setEnabled(False)
        else:
            print("Warning: MainWindow UI missing 'status_btn'")

        if hasattr(self, 'info_btn'):
            self.info_btn.clicked.connect(self.show_info_window)
            self.info_btn.setEnabled(False)
        else:
            print("Warning: MainWindow UI missing 'info_btn'")
        
        # For logging messages from backend or RC commands
        if not hasattr(self, 'sensorText'):
             print("Warning: MainWindow UI missing 'sensorText' for logging.")


        # Focus for key events
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def update_time_display(self):
        if hasattr(self, 'time_edit'): 
            self.time_edit.setText(QTime.currentTime().toString("hh:mm:ss"))
        elif hasattr(self, 'timeLabel'):
             self.timeLabel.setText(QTime.currentTime().toString("hh:mm:ss"))


    def toggle_power(self):
        self.power_on = not self.power_on
        if self.power_on:
            if not self.ser or not self.ser.is_open:
                self.log_to_gui("ERROR: Serial port not available. Cannot turn ON.")
                self.power_on = False 
                if hasattr(self, 'power_btn'): self.power_btn.setText("ON")
                return

            if hasattr(self, 'power_btn'): self.power_btn.setText("OFF")
            if hasattr(self, 'status_btn'): self.status_btn.setEnabled(True)
            if hasattr(self, 'info_btn'): self.info_btn.setEnabled(True)
            
            self.rc_timer.start(50) 
            self.log_to_gui("RC control timer started.")

            if not self.backend_thread or not self.backend_thread.isRunning():
                self.backend_thread = BackendThread(self.ser)
                self.backend_thread.new_sensor_data.connect(self.handle_new_sensor_data)
                self.backend_thread.log_message.connect(self.log_to_gui)
                self.backend_thread.start()
                self.log_to_gui("Backend thread started.")
            self.setFocus() 

        else: # Power OFF
            if hasattr(self, 'power_btn'): self.power_btn.setText("ON")
            if hasattr(self, 'status_btn'): self.status_btn.setEnabled(False)
            if hasattr(self, 'info_btn'): self.info_btn.setEnabled(False)
            
            self.rc_timer.stop()
            self.log_to_gui("RC control timer stopped.")

            if self.backend_thread and self.backend_thread.isRunning():
                self.backend_thread.stop()
                # self.backend_thread.wait() # Wait for thread to finish, can block GUI
                self.log_to_gui("Backend thread stop requested.")
            
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b"S\n")
                    self.log_to_gui("Sent motor STOP command (S).")
                except Exception as e:
                    self.log_to_gui(f"Error sending STOP on power off: {e}")
            
            if hasattr(self, 'sensorText'): self.sensorText.clear()


    def keyPressEvent(self, event: QKeyEvent):
        if not self.power_on: return 
        if not event.isAutoRepeat():
            self.keys_pressed.add(event.key())
        # self.update_rc_command() # Optionally call here for immediate response

    def keyReleaseEvent(self, event: QKeyEvent):
        if not self.power_on: return
        if not event.isAutoRepeat():
            self.keys_pressed.discard(event.key())
            # If W or S released, send Stop command immediately
            if event.key() == Qt.Key.Key_W or event.key() == Qt.Key.Key_S:
                 if self.ser and self.ser.is_open:
                    try:
                        self.ser.write(b"S\n")
                        # self.log_to_gui("KeyRelease: Sent S")
                    except Exception as e:
                        self.log_to_gui(f"Serial error on key release stop: {e}")


    def update_rc_command(self):
        if not self.power_on or not self.ser or not self.ser.is_open:
            print("Serial port /dev/ttyACM0 not available. Cannot send command.")
            return

        try:
            # Motor control
            motor_command_sent = False
            if Qt.Key.Key_W in self.keys_pressed:
                self.ser.write(b'MF\n')
                motor_command_sent = True
                print(self.ser.readline(), 'MF')
            elif Qt.Key.Key_S in self.keys_pressed:
                self.ser.write(b'MB\n')
                motor_command_sent = True
                print(self.ser.readline(), 'MB')
            elif Qt.Key.Key_A in self.keys_pressed:
                self.ser.write(b'TL\n')
                motor_command_sent = True
                print(self.ser.readline(), 'TL')
            elif Qt.Key.Key_D in self.keys_pressed:
                self.ser.write(b'TR\n')
                motor_command_sent = True
                print(self.ser.readline(), 'TR')

            if not motor_command_sent:
                self.ser.write(b'MS\n')
                print(self.ser.readline(), 'MS')

            # Speed setting (Q/E) - Modifies a local variable.
            if Qt.Key.Key_Q in self.keys_pressed and Qt.Key.Key_E not in self.keys_pressed:
                self.motor_speed_setting -= 10
                if self.motor_speed_setting < 0: self.motor_speed_setting = 0
                self.log_to_gui(f"RC: Motor speed setting: {self.motor_speed_setting} (not sent)")

                command = f"X{self.motor_speed_setting}\n"
                self.ser.write(command.encode())
                print(self.motor_speed_setting)

            elif Qt.Key.Key_E in self.keys_pressed and Qt.Key.Key_Q not in self.keys_pressed:
                self.motor_speed_setting += 10
                if self.motor_speed_setting > 250: self.motor_speed_setting = 250
                self.log_to_gui(f"RC: Motor speed setting: {self.motor_speed_setting} (not sent)")

                command = f"X{self.motor_speed_setting}\n"
                self.ser.write(command.encode())
                print(self.motor_speed_setting)

        except serial.SerialException as e:
            print(f"Serial write error on /dev/ttyACM0: {e}. Connection may be lost.")
            if self.ser and self.ser.is_open:
                self.ser.close() 


        except serial.SerialException as e:
            self.log_to_gui(f"RC ERROR: Serial write failed: {e}")
            self.rc_timer.stop() # Stop timer on serial failure

        except Exception as e:
            self.log_to_gui(f"RC ERROR: Unexpected in update_rc_command: {e}")


    def handle_new_sensor_data(self, shock, temp):
        if self.status_window_instance and self.status_window_instance.isVisible():
            self.status_window_instance.update_display(shock, temp)
        # self.log_to_gui(f"Sensor Data: Shock={shock:.1f}, Temp={temp:.1f}°C") # Optional: log to main window too

    def log_to_gui(self, message):
        print(message) 
        if hasattr(self, 'sensorText') and self.sensorText: 
            self.sensorText.append(message) 
            self.sensorText.ensureCursorVisible()

    def show_status_window(self):
        if not self.status_window_instance:
            self.status_window_instance = StatusWindow(self)
        self.status_window_instance.show()
        self.hide()

    def show_info_window(self):
        if not self.info_window_instance:
            self.info_window_instance = InfoWindow(self)
        self.info_window_instance.show()
        self.hide()

    def closeEvent(self, event: QKeyEvent):
        self.log_to_gui("Application closing...")
      
        if self.power_on:
            self.toggle_power()

        if self.backend_thread and self.backend_thread.isRunning():
            self.backend_thread.stop()
            self.backend_thread.wait(2000) 
            if self.backend_thread.isRunning():
                 self.log_to_gui("Warning: Backend thread did not terminate gracefully.")

        if self.ser and self.ser.is_open:
            self.log_to_gui("Closing serial port.")
            self.ser.close()
        
        self.log_to_gui("Exiting.")
        super().closeEvent(event)


class StatusWindow(QWidget, status_window_ui):
    def __init__(self, main_window_ref): 
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Vehicle Status")
        self.main_window_ref = main_window_ref

        if hasattr(self, 'main_btn'): 
            self.main_btn.clicked.connect(self.return_to_main)
        elif hasattr(self, 'backButton'):
            self.backButton.clicked.connect(self.return_to_main)
        else:
            print("Warning: StatusWindow UI missing 'main_btn' or 'backButton'")
        
        # Initialize text fields
        if hasattr(self, 'temp_edit'): self.temp_edit.setText("Temp: --°C")
        if hasattr(self, 'shock_edit'): self.shock_edit.setText("Shock: --")
        if hasattr(self, 'speed_edit'): self.speed_edit.setText("Speed: -- km/h")


    def update_display(self, shock, temp):
        if hasattr(self, 'temp_edit'):
            self.temp_edit.setText(f"Temp: {temp:.1f}°C")
        if hasattr(self, 'shock_edit'):
            self.shock_edit.setText(f"Shock: {shock:.0f}")

    def return_to_main(self):
        # self.main_window_ref.show() # MainWindow should not be hidden
        self.close()


class InfoWindow(QWidget, info_window_ui):
    def __init__(self, main_window_ref):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Vehicle Information")
        self.main_window_ref = main_window_ref

        if hasattr(self, 'main_btn'):
            self.main_btn.clicked.connect(self.return_to_main)
        elif hasattr(self, 'backButton'):
            self.backButton.clicked.connect(self.return_to_main)
        else:
            print("Warning: InfoWindow UI missing 'main_btn' or 'backButton'")

        if not hasattr(self, 'info_edit') and not hasattr(self, 'main_edit'):
            print("Warning: InfoWindow UI missing 'info_edit' or 'main_edit' for data display.")

        self.load_driver_data_from_db()

    def load_driver_data_from_db(self):
        text_widget = None
        if hasattr(self, 'info_edit'): 
            text_widget = self.info_edit
        elif hasattr(self, 'main_edit'): 
            text_widget = self.main_edit
        
        if not text_widget:
            print("InfoWindow: No text widget found to display data.")
            return

        text_widget.clear()
        conn = None
        try:
            conn = get_db_connection()
            with conn.cursor() as cur:
                cur.execute("SELECT AVG(shock) AS avg_shock, AVG(temperature) AS avg_temp FROM sensor_data")
                result = cur.fetchone()
                if result and result[0] is not None and result[1] is not None:
                    text_widget.append(f"Average Temperature: {result[1]:.1f}°C")
                    text_widget.append(f"Average Shock Level: {result[0]:.1f}")
                else:
                    text_widget.append("No sensor data available for averages.")

                text_widget.append("\nLast 5 Sensor Readings:")
                cur.execute("SELECT temperature, shock, timestamp FROM sensor_data ORDER BY timestamp DESC LIMIT 5")
                results = cur.fetchall()
                if results:
                    for row in results:
                        ts = row[2].strftime('%Y-%m-%d %H:%M:%S') if row[2] else 'N/A'
                        text_widget.append(f"- Temp: {row[0]:.1f}°C, Shock: {row[1]:.0f} (at {ts})")
                else:
                    text_widget.append("No recent readings.")

        except Exception as e:
            print(f"[DB_INFO_ERROR] {e}")
            if text_widget: text_widget.append(f"Error loading data: {e}")
        finally:
            if conn:
                conn.close()

    def return_to_main(self):
        self.main_window_ref.show()
        self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    ser_connection = None
    try:
        ser_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_S)
        print(f"Successfully connected to serial port {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"ERROR: Could not open serial port {SERIAL_PORT}: {e}")
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Icon.Critical)
        msg_box.setText(f"Failed to open serial port {SERIAL_PORT}.\n{e}\n\nThe application will run without serial communication features.")
        msg_box.setWindowTitle("Serial Port Error")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Ok)
        msg_box.exec()

    main_win = MainWindow(ser_connection)
    main_win.show()
    
    exit_code = app.exec()
    
    if main_win.backend_thread and main_win.backend_thread.isRunning():
        print("Main: Ensuring backend thread is stopped...")
        main_win.backend_thread.stop()
        main_win.backend_thread.wait(1000) # Wait a bit
    if ser_connection and ser_connection.is_open:
        print("Main: Ensuring serial port is closed...")
        ser_connection.close()
        
    sys.exit(exit_code)

