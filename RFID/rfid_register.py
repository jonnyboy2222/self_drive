import serial
import sys
import socket
import json
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6 import uic
import struct

PACKET_HEADER = 0xAA
COMMAND = b'JS'

from_class = uic.loadUiType("rfid_register.ui")[0]

class SerialReader(QThread):
    data_received = pyqtSignal(str, bool)

    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True

    def run(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=1) as ser:
                while self.running:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8').strip()
                        if "UID" in line:
                            uid = line.split(":")[1].strip()
                            self.data_received.emit(uid, True)

        except serial.SerialException as e:
            self.data_received.emit(str(e), False)

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("RFID 등록")

        self.labelStatus.setText("RFID 인식 기다리는 중...")
        self.tableWidget.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)

        self.reader = SerialReader(port='/dev/ttyACM0', baudrate=9600)
        self.reader.data_received.connect(self.update_data)
        self.reader.start()
        self.host = '192.168.2.234'  # 서버 ip
        self.port = 12345

        self.btnConfirm.clicked.connect(self.confirm_data)
        self.btnReset.clicked.connect(self.reset)
        self.btnRegister.clicked.connect(self.register_data)

        self.disable()

    def enable(self):
        self.editUid.setDisabled(False)
        self.editName.setDisabled(False)
        self.editBirth.setDisabled(False)
        self.editHeight.setDisabled(False)
        self.editWeight.setDisabled(False)
        self.editPhone.setDisabled(False)
        self.editLicense.setDisabled(False)

        self.btnConfirm.setDisabled(False)
        self.btnRegister.setDisabled(False)

    def disable(self):
        self.editUid.setDisabled(True)
        self.editName.setDisabled(True)
        self.editBirth.setDisabled(True)
        self.editHeight.setDisabled(True)
        self.editWeight.setDisabled(True)
        self.editPhone.setDisabled(True)
        self.editLicense.setDisabled(True)

        self.btnConfirm.setDisabled(True)
        self.btnRegister.setDisabled(True)

    def update_data(self, data, success):
        if success:
            # 중복 체크
            check_data = {
                "purpose" : "verification",
                "rfid_uid" : data
            }
            check_res = self.send_tcp_data(check_data, self.host, self.port)
            if check_res:
                try:
                    res_json = json.loads(check_res)
                    if res_json.get("message") == "FAIL":
                        self.enable()
                        self.editUid.setText(data)
                        self.labelStatus.setText("인식 완료!")
                    else:
                        QMessageBox.warning(
                            self, "등록 오류", "모든 데이터가 이미 등록되어 있습니다.") 
                        existing_user = res_json.get("user_data")
                        self.labelStatus.setText("초기화 버튼을 누른 뒤 다시 등록해주세요.")
                        # 모든 행 제거
                        self.tableWidget.setRowCount(0)
            
                        # 중복된 사용자 정보를 테이블에 표시
                        if existing_user:
                            duplicate_row = self.tableWidget.rowCount()
                            self.tableWidget.insertRow(duplicate_row)
                        
                            # 기존 데이터를 빨간색으로 표시
                            for index, (key, value) in enumerate(existing_user.items()):
                                item = QTableWidgetItem(str(value))
                                item.setForeground(QColor('red'))
                                self.tableWidget.setItem(duplicate_row, index, item)
                except Exception as e:
                    QMessageBox.warning(
                        self, "등록 오류", f"JSON 파싱 오류:{e}")
        else:
            self.labelStatus.setText(f"에러 발생: {data}")

    def confirm_data(self):
        row = self.tableWidget.rowCount()
        self.tableWidget.insertRow(row)
        self.tableWidget.setItem(row, 0, QTableWidgetItem(self.editUid.text()))
        self.tableWidget.setItem(row, 1, QTableWidgetItem(self.editName.text()))
        self.tableWidget.setItem(row, 2, QTableWidgetItem(self.editBirth.text()))
        self.tableWidget.setItem(row, 3, QTableWidgetItem(self.editHeight.text()))
        self.tableWidget.setItem(row, 4, QTableWidgetItem(self.editWeight.text()))
        self.tableWidget.setItem(row, 5, QTableWidgetItem(self.editPhone.text()))
        self.tableWidget.setItem(row, 6, QTableWidgetItem(self.editLicense.text()))

        # 입력 필드 초기화
        self.editUid.clear()
        self.editName.clear()
        self.editBirth.clear()
        self.editHeight.clear()
        self.editWeight.clear()
        self.editPhone.clear()
        self.editLicense.clear()

    def register_data(self):
        if self.tableWidget.rowCount() == 0:
            self.labelStatus.setText("")
            QMessageBox.warning(
                    self, "등록 오류", "저장할 데이터가 없습니다!")
            #self.labelStatus.setText("저장할 데이터가 없습니다!")
            return
        
        try:
            # 모든 행에 대해 반복
            for row in range(self.tableWidget.rowCount() - 1, -1, -1):
                # 모든 필드가 채워져 있는지 확인
                is_empty = False
                for col in range(7):  # 8개의 컬럼 확인
                    item = self.tableWidget.item(row, col)
                    if not item or not item.text().strip():
                        is_empty = True
                        break
                
                if is_empty:
                    QMessageBox.warning(self, "등록 오류", 
                        f"사용자 정보가 불충분합니다.")
                    self.tableWidget.removeRow(row)
                    return
                
                uid = self.tableWidget.item(row, 0).text()
                
                value_data = {
                    "purpose" : "db",
                    "rfid_uid" : uid,
                    "name" : self.tableWidget.item(row, 1).text(),
                    "birth_date" : self.tableWidget.item(row, 2).text(),
                    "height" : self.tableWidget.item(row, 3).text(),
                    "weight" : self.tableWidget.item(row, 4).text(),
                    "phone_num" : self.tableWidget.item(row, 5).text(),
                    "license_num" : self.tableWidget.item(row, 6).text()
                }
                insert_res = self.send_tcp_data(value_data, self.host, self.port)
                if insert_res:
                    in_res_json = json.loads(insert_res)
                    if in_res_json.get("message") == "PASS":
                        message = "데이터 등록 성공"
                        QMessageBox.information(self, "등록 결과", message)
                        # 모든 행 제거
                        self.tableWidget.setRowCount(0)
                        self.disable()
                        #self.labelStatus.setText(message)
                    else:
                        #self.labelStatus.setText("모든 데이터가 이미 등록되어 있습니다.")
                        error_msg = "등록 실패!"
                        QMessageBox.warning(
                            self, "등록 오류", error_msg)          
                            
        except Exception as e:
            QMessageBox.warning(self, "등록 오류", f"JSON 파싱 오류:{e}")

    def send_tcp_data(self, data, host, port):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                json_bytes = json.dumps(data).encode("utf-8")
                length = len(json_bytes)

                packet = bytearray()
                packet.append(PACKET_HEADER)
                packet += COMMAND
                packet += struct.pack("<H", length)
                packet += json_bytes
                
                s.connect((host, port))
                s.sendall(packet)

                response = s.recv(1024)
                return response.decode()
        except Exception as e:
            print(f"TCP 전송 오류: {e}")

    def reset(self):
        self.tableWidget.setRowCount(0)

    def closeEvent(self, event):
        self.reader.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec())
