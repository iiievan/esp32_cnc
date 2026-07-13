import socket

ESP32_IP = "192.168.1.85"
ESP32_PORT = 8080

def send_one_command(cmd):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"Sending: {cmd}")
    sock.sendto((cmd + "\n").encode(), (ESP32_IP, ESP32_PORT))
    
    sock.settimeout(1)
    try:
        data, _ = sock.recvfrom(1024)
        print(f"  -> {data.decode().strip()}")
    except socket.timeout:
        print("  -> Timeout")
    
    sock.close()

# Отправляем одну команду
send_one_command("G1 X10 Y10 F100")