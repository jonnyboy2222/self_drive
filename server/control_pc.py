# --- PC1: Serial to PC2 with DB insert and VF response handling ---

import serial
import struct
import socket
import time
import pymysql
from dbutils.pooled_db import PooledDB
import threading

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
TIMEOUT_S = 1.0

TCP_SERVER_IP = '192.168.0.42'
TCP_SERVER_PORT = 12345

PACKET_HEADER = 0xAA
DB_PACKET_SIZE = 15 # 1(header) + 2(command) + 4(uid) + 8(floats)
VF_PACKET_SIZE = 7
VF_RESPONSE_SIZE = 6 # 1(header) + 2(command) + 1(VF_RESP) + 2 (padding)

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

if __name__ == "__main__":
    main()
