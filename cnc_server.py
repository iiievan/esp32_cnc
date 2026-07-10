import socket

server_ip = "0.0.0.0"  # Слушать все интерфейсы
port = 8080

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((server_ip, port))
    s.listen()
    print(f"TCP Server listening on {server_ip}:{port}")
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f"Received: {data.decode()}")
            conn.sendall(b"Hello from Python server!\n")