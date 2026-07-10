import socket
import time

# Настройки
ESP32_IP = "192.168.1.85"   # IP вашей ESP32
ESP32_PORT = 8080

# Создаем UDP сокет
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Команды, которые мы будем отправлять (похожи на GRBL)
commands = [
    "G1 X10 Y20\n",
    "G1 X-10 Y-20\n",
    "M5\n",  # Stop
    "G28\n"  # Home
]

for cmd in commands:
    print(f"Sending: {cmd.strip()}")
    sock.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))
    
    # Ожидаем ответ от ESP32
    sock.settimeout(2)  # Таймаут 2 секунды
    try:
        data, _ = sock.recvfrom(1024)
        print(f"Received from ESP32: {data.decode().strip()}")
    except socket.timeout:
        print("Timeout! No response from ESP32")
    
    time.sleep(1)  # Пауза между командами

sock.close()