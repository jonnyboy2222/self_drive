import socket

HOST = '0.0.0.0'  # 모든 IP로부터 수신
PORT = 12345

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.bind((HOST, PORT))
        server_sock.listen()
        print(f"Listening on port {PORT}...")

        conn, addr = server_sock.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                print("Received:", data.decode("utf-8"))

if __name__ == "__main__":
    start_server()
