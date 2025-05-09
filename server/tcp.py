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
                print(f"[DEBUG] Received data: {data.decode()}")
                message = json.loads(data.decode())
            except json.JSONDecodeError:
                print(f"[ERROR] Invalid JSON received: {data.decode()}")
                continue

            purpose = message.get("purpose")

            if purpose == "verification":
                rfid_uid = message.get("rfid_uid")
                print(f"[INFO] RFID UID Received: {rfid_uid}")
                verified = False

                with get_db_connection() as conn_db:
                    with conn_db.cursor() as cursor:
                        sql = "SELECT COUNT(*) FROM authorized_rfids WHERE uid = %s"
                        cursor.execute(sql, (rfid_uid,))
                        result = cursor.fetchone()
                        if result and result[0] > 0:
                            verified = True

                response = "PASS" if verified else "FAIL"
                conn.sendall(response.encode() + b"\n")
                print(f"[INFO] RFID UID {rfid_uid} verification result sent: {response}")

            else:
                rfid_uid = message.get("rfid_uid")
                shock = message.get("shock")
                temp = message.get("temperature")

                # stores data to db
                # value_to_store = json.dumps(value) if isinstance(value, list) else value
                with get_db_connection() as conn_db:
                    with conn_db.cursor() as cursor:
                        sql = "INSERT INTO test (uid, shock, temperature) VALUES (%s, %s, %s)"
                        cursor.execute(sql, (rfid_uid, shock, temp))
                print(f"[INFO] Sensor data successfully stored in DB.")

    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
    finally:
        conn.close()

# server run
HOST = "0.0.0.0"
PORT = 12345

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    print(f"[INFO] TCP Server listening on {HOST}:{PORT}")
    while True:
        conn, addr = server_socket.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

except KeyboardInterrupt:
    print("[INFO] Server interrupted by user (Ctrl+C).")
finally:
    server_socket.close()  # Ensure socket is properly closed when the server is interrupted
    print("[INFO] Server socket closed. Port released.")