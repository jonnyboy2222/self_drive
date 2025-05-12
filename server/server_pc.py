# --- PC2: Receives packet from PC1, handles DB storage and VF check ---

import socket
import struct
import threading
import pymysql
from dbutils.pooled_db import PooledDB

TCP_SERVER_IP = '0.0.0.0'
TCP_SERVER_PORT = 12345

PACKET_HEADER = 0xAA

# === Packet sizes (only 3 remain) ===
DB_PACKET_SIZE = 15  # 1(header) + 2(command) + 4(uid) + 4(float) + 4(float)
VF_PACKET_SIZE = 7   # 1(header) + 2(command) + 4(uid)
VF_RESPONSE_SIZE = 6 # 1(header) + 2(command) + 1(result) + 2(padding)

# Valid UID list
VALID_UIDS = {"5A4B", "1234", "ABCD"}

# DB setup
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

def insert_to_db(shock, temp):
    try:
        conn = get_db_connection()
        with conn.cursor() as cur:
            cur.execute("INSERT INTO sensor_data (shock, temperature) VALUES (%s, %s)", (shock, temp))
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
                    continue
                uid = rest[0:4].decode("ascii", errors="replace")
                shock = struct.unpack('<f', rest[4:8])[0]
                temp = struct.unpack('<f', rest[8:12])[0]
                insert_to_db(shock, temp)
                print(f"[PC2] Stored DB: UID={uid}, Shock={shock:.2f}, Temp={temp:.2f}")

            elif command == "VF":
                rest = conn.recv(VF_PACKET_SIZE - 3)
                if len(rest) < VF_PACKET_SIZE - 3:
                    continue
                uid = rest[0:4].decode("ascii", errors="replace")
                print(f"[PC2] Received VF request: UID={uid}")

                result = 1 if uid in VALID_UIDS else 0
                response = bytearray()
                response.append(PACKET_HEADER)
                response += b'VF'
                response.append(result)

                conn.sendall(response)
                print(f"[PC2] Sent VF response: {response}")

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
