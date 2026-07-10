import socket
import time
import math

ESP32_IP = "192.168.1.85"
ESP32_PORT = 8080

def send_commands(commands):
    """Отправляет команды и выводит ответы"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    for cmd in commands:
        print(f"Sending: {cmd}")
        sock.sendto((cmd + "\n").encode(), (ESP32_IP, ESP32_PORT))
        
        sock.settimeout(1)
        try:
            data, _ = sock.recvfrom(1024)
            print(f"  -> {data.decode().strip()}")
        except socket.timeout:
            print("  -> Timeout")
        
        time.sleep(0.05)  # Небольшая задержка
    
    sock.close()

# Генерируем круг из 72 точек
radius = 10
segments = 72
commands = ["G90"]  # Абсолютные координаты

for i in range(segments + 1):
    angle = 2 * math.pi * i / segments
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    if i == 0:
        commands.append(f"G1 X{x:.3f} Y{y:.3f} F100")
    else:
        commands.append(f"G1 X{x:.3f} Y{y:.3f}")

commands.append("G1 X0 Y0")  # Возврат в центр

print(f"Sending {len(commands)} commands for circle...")
send_commands(commands)