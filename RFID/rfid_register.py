import serial
import time
import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6 import uic
import mysql.connector

from_class = uic.loadUiType("rfid_register.ui")[0]

car_db = mysql.connector.connect(
    host = "localhost",
    port = 3306,
    user = "kth",
    password = "th0708csi!",
    database = "car_db"
)

# 실행 전 아두이노 포트 확인할 것!(/dev/ttyACM0 or /dev/ttyACM1 or /dev/ttyUSB0)

class SerialReader(QThread):
    data_received = pyqtSignal(str, bool)

    def __init__(self, port='/dev/ttyACM1', baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True

    def run(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=1) as ser:
                time.sleep(2)
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

        self.db_conn = car_db
        self.reader = SerialReader(port='/dev/ttyACM1', baudrate=9600)
        self.reader.data_received.connect(self.update_data)
        self.reader.start()

        self.btnConfirm.clicked.connect(self.confirm_data)
        self.btnRegister.clicked.connect(self.register_data)

    def update_data(self, data, success):
        if success:
            self.editUid.setText(data)
            self.labelStatus.setText("인식 완료!")
        else:
            self.labelStatus.setText(f"에러 발생: {data}")

    def confirm_data(self):
        row = self.tableWidget.rowCount()
        self.tableWidget.insertRow(row)
        self.tableWidget.setItem(row, 0, QTableWidgetItem(self.editUid.text()))
        self.tableWidget.setItem(row, 1, QTableWidgetItem(self.editName.text()))
        self.tableWidget.setItem(row, 2, QTableWidgetItem(self.editBirth.text()))
        self.tableWidget.setItem(row, 3, QTableWidgetItem(self.editSn.text()))
        self.tableWidget.setItem(row, 4, QTableWidgetItem(self.editHeight.text()))
        self.tableWidget.setItem(row, 5, QTableWidgetItem(self.editWeight.text()))
        self.tableWidget.setItem(row, 6, QTableWidgetItem(self.editPhone.text()))
        self.tableWidget.setItem(row, 7, QTableWidgetItem(self.editLicense.text()))

        # 입력 필드 초기화
        self.editUid.clear()
        self.editName.clear()
        self.editBirth.clear()
        self.editSn.clear()
        self.editHeight.clear()
        self.editWeight.clear()
        self.editPhone.clear()
        self.editLicense.clear()

    def register_data(self):
        if self.tableWidget.rowCount() == 0:
            self.labelStatus.setText("저장할 데이터가 없습니다!")
            return
            
        cursor = self.db_conn.cursor(buffered=True)
        success_count = 0
        error_count = 0
        
        try:
            # 모든 행에 대해 반복
            for row in range(self.tableWidget.rowCount()):
                uid = self.tableWidget.item(row, 0).text()
                
                # 중복 체크
                check_sql = "SELECT * FROM user WHERE uid = %s"
                cursor.execute(check_sql, (uid,))
                
                if cursor.fetchone():
                    error_count += 1
                    continue
                
                # 데이터 타입 변환
                try:
                    # birth_date를 date 형식으로 변환 (YYYY-MM-DD 형식 가정)
                    birth_date = self.tableWidget.item(row, 2).text()
                    if not birth_date:
                        raise ValueError("생년월일이 비어있습니다.")
                    
                    # height와 weight를 float으로 변환
                    height = float(self.tableWidget.item(row, 4).text() or 0)
                    weight = float(self.tableWidget.item(row, 5).text() or 0)
                    
                    # 데이터 등록
                    insert_sql = """
                        INSERT INTO user (uid, user_name, birth_date, 
                        serial_num, height, weight, phone_num, license_num)
                        VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                    """
                    values = (
                        uid,
                        self.tableWidget.item(row, 1).text(),
                        birth_date,
                        self.tableWidget.item(row, 3).text(),
                        height,
                        weight,
                        self.tableWidget.item(row, 6).text(),
                        self.tableWidget.item(row, 7).text()
                    )
                    
                    cursor.execute(insert_sql, values)
                    success_count += 1
                    
                except ValueError as e:
                    self.labelStatus.setText(f"데이터 형식 오류: {str(e)}")
                    continue
                except Exception as e:
                    self.labelStatus.setText(f"데이터 처리 오류: {str(e)}")
                    continue
            
            self.db_conn.commit()
            
            # 결과 메시지 표시
            if success_count > 0:
                message = f"{success_count}개의 데이터가 성공적으로 등록되었습니다."
                if error_count > 0:
                    message += f"\n{error_count}개의 데이터는 이미 등록되어 있어 건너뛰었습니다."
                self.labelStatus.setText(message)
            else:
                self.labelStatus.setText("모든 데이터가 이미 등록되어 있습니다!")
            
            self.tableWidget.setRowCount(0)
            
        except mysql.connector.Error as err:
            self.labelStatus.setText(f"데이터베이스 오류: {err}")
            self.db_conn.rollback()
            
        finally:
            cursor.close()

    def closeEvent(self, event):
        self.reader.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec())