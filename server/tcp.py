import socket
import threading
import json
import pymysql
from dbutils.pooled_db import PooledDB

# --- Database Connection Pool ---
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

def handle_client(conn, addr):
    print(f"[INFO] Connected by {addr}")
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                print("[INFO] Connection closed by client.")
                break

            try:
                message = json.loads(data.decode())
            except json.JSONDecodeError:
                print("[ERROR] Invalid JSON received.")
                continue

            sensor_type = message.get("sensor")
            timestamp = message.get("time")
            value = message.get("data")
            rfid = message.get("rfid_tag")

            if rfid:
                rfid_tag = rfid
                print(f"[INFO] RFID Tag Received: {rfid_tag}")
                verified = False

                with get_db_connection() as conn_db:
                    with conn_db.cursor() as cursor:
                        sql = "SELECT COUNT(*) FROM authorized_rfids WHERE tag_id = %s"
                        cursor.execute(sql, (rfid_tag,))
                        result = cursor.fetchone()
                        if result and result[0] > 0:
                            verified = True

                response = "PASS" if verified else "FAIL"
                conn.sendall(response.encode() + b"\n")
                print(f"[INFO] RFID Tag {rfid_tag} verification result sent: {response}")

            else:
                # stores data to db
                value_to_store = json.dumps(value) if isinstance(value, list) else value
                with get_db_connection() as conn_db:
                    with conn_db.cursor() as cursor:
                        sql = "INSERT INTO test (sensor, realtime, data) VALUES (%s, %s, %s)"
                        cursor.execute(sql, (sensor_type, timestamp, value_to_store))
                print(f"[INFO] Sensor data from {sensor_type} stored in DB.")

    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
    finally:
        conn.close()

# server run
HOST = "0.0.0.0"
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    print(f"[INFO] TCP Server listening on {HOST}:{PORT}")
    while True:
        conn, addr = server_socket.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
