# --- PC2: Receives packet from PC1, handles DB storage and VF check ---

import socket
import struct
import threading
import pymysql
from dbutils.pooled_db import PooledDB
import json
import datetime

TCP_SERVER_IP = '0.0.0.0'
TCP_SERVER_PORT = 12345

PACKET_HEADER = 0xAA

# === Packet sizes (only 3 remain) ===
DB_PACKET_SIZE = 15  # 1(header) + 2(command) + 4(uid) + 4(float) + 4(float)
VF_PACKET_SIZE = 7   # 1(header) + 2(command) + 4(uid)
VF_RESPONSE_SIZE = 6 # 1(header) + 2(command) + 1(result) + 2(padding)


# DB setup
db_pool = PooledDB(
    creator=pymysql,
    maxconnections=5,
    mincached=2,
    host="localhost",
    user="kth",
    password="th0708csi!",
    database="car_db",
    charset="utf8mb4",
    autocommit=True
)

def get_db_connection():
    return db_pool.connection()

def insert_to_db(uid_hex, shock, temp):
    try:
        conn = get_db_connection()
        with conn.cursor() as cur:
            cur.execute("INSERT INTO sensor_data (uid, shock, temperature) VALUES (%s, %s)", (uid_hex, shock, temp))
    except Exception as e:
        print(f"[DB ERROR] {e}")
    finally:
        conn.close()

def handle_client(conn, addr):
    print(f"[PC2] Connected from {addr}")
    try:
        while True:
            header = conn.recv(1)
            if not header or header[0] != PACKET_HEADER:
                continue

            cmd_bytes = conn.recv(2)
            if len(cmd_bytes) < 2:
                continue
            command = cmd_bytes.decode("ascii", errors="replace")

            if command == "DB":
                rest = conn.recv(DB_PACKET_SIZE - 3)
                if len(rest) < DB_PACKET_SIZE - 3:
                    print("[PC2] Incomplete DB packet received")
                    continue
                uid_hex = rest[0:4].hex().upper()
                shock = struct.unpack('<f', rest[4:8])[0]
                temp = struct.unpack('<f', rest[8:12])[0]
                insert_to_db(uid_hex, shock, temp)
                print(f"[PC2] Stored DB: UID={uid_hex}, Shock={shock:.2f}, Temp={temp:.2f}")

            elif command == "VF":
                rest = conn.recv(VF_PACKET_SIZE - 3)
                if len(rest) < VF_PACKET_SIZE - 3:
                    continue

                uid_hex = rest[0:4].hex().upper()
                print(f"[PC2] Received VF request: UID={uid_hex}")

                try:
                    with db_pool.connection() as conn_db, conn_db.cursor() as cur:
                        cur.execute("SELECT EXISTS(SELECT 1 FROM user WHERE uid = %s)", (uid_hex,))
                        exists = cur.fetchone()[0]
                        result = 1 if exists else 0
                except Exception as e:
                    print(f"[DB ERROR] {e}")
                    result = 0

                response = bytearray()
                response.append(PACKET_HEADER)
                response += b'VF'
                response.append(result)

                conn.sendall(response)
                print(f"[PC2] Sent VF response: {response}")

            elif command == "JS":
                try:
                    length_bytes = conn.recv(2)
                    length = struct.unpack('<H', length_bytes)[0]
                    rest = conn.recv(length)
                    if not rest:
                        continue
                    try:
                        json_data = json.loads(rest.decode("utf-8"))
                    except UnicodeDecodeError:
                        print("[PC2] UTF-8 Decode Error")
                        continue
                    purpose = json_data.get("purpose")

                    if purpose == "verification":
                        uid = json_data.get("rfid_uid")
                        print(f"[PC2][JSON-VERIFICATION] UID={uid}")

                        with db_pool.connection() as conn_db, conn_db.cursor() as cur:
                            cur.execute("SELECT EXISTS(SELECT 1 FROM user WHERE uid = %s)", (uid,))
                            exists = cur.fetchone()[0]

                            #result_msg = {"message": "PASS" if exists else "FAIL"}
                            if exists:
                                cur.execute("SELECT * FROM user WHERE uid = %s", (uid,))
                                existing_user = cur.fetchone()
                                # 컬럼 이름 가져오기
                                columns = [desc[0] for desc in cur.description]
                                # 컬럼과 값을 딕셔너리로 변환
                                user_data = dict(zip(columns, existing_user))
                                result_msg = {
                                    "message": "PASS",
                                    "user_data": user_data
                                }
                            else:
                                result_msg = {"message": "FAIL"}
                            conn.sendall(json.dumps(result_msg).encode("utf-8"))
                            print(f"[PC2] Sent verification result: {result_msg}")

                    elif purpose == "db":
                        # 예: DB 테이블: rfid_users (uid, name, birth_date, height, weight, phone_num, license_num)
                        try:
                            with db_pool.connection() as conn_db, conn_db.cursor() as cur:
                                cur.execute("""
                                    INSERT INTO user (uid, user_name, birth_date, height, weight, phone_num, license_num)
                                    VALUES (%s, %s, %s, %s, %s, %s, %s)
                                """, (
                                    json_data.get("rfid_uid"),
                                    json_data.get("name"),
                                    json_data.get("birth_date"),
                                    json_data.get("height"),
                                    json_data.get("weight"),
                                    json_data.get("phone_num"),
                                    json_data.get("license_num")
                                ))
                                conn.sendall(json.dumps({"message": "PASS"}).encode("utf-8"))
                                print(f"[PC2] Stored user: {json_data}")
                        except Exception as e:
                            print(f"[PC2][DB-INSERT ERROR] {e}")
                            conn.sendall(json.dumps({"message": "FAIL"}).encode("utf-8"))

                except Exception as e:
                    print(f"[PC2][JSON ERROR] {e}")
                    break

    except Exception as e:
        print(f"[PC2 ERROR] {e}")
    finally:
        conn.close()
        print(f"[PC2] Disconnected from {addr}")

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((TCP_SERVER_IP, TCP_SERVER_PORT))
        server.listen()
        print(f"[PC2] TCP Server listening on {TCP_SERVER_IP}:{TCP_SERVER_PORT}")

        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    main()
