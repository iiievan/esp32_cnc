import socket
import time

ESP32_IP = "192.168.1.85"
ESP32_PORT = 8080

def send_command(sock, cmd, wait_response=True):
    """Отправляет команду и опционально ждет ответ"""
    print(f"Sending: {cmd}")
    sock.sendto((cmd + "\n").encode(), (ESP32_IP, ESP32_PORT))
    
    if wait_response:
        sock.settimeout(0.5)
        try:
            data, _ = sock.recvfrom(1024)
            print(f"  -> {data.decode().strip()}")
        except socket.timeout:
            print("  -> Timeout")

def send_triangle():
    """Отправляет команды для движения по треугольнику"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Параметры треугольника
    size = 50  # размер в мм
    feedrate = 100  # скорость в мм/мин
    
    # Координаты вершин треугольника (равносторонний)
    # Вершина 1: (0, 0)
    # Вершина 2: (size, 0)  
    # Вершина 3: (size/2, size * 0.866)  # высота равностороннего треугольника
    
    x1, y1 = 0, 0
    x2, y2 = size, 0
    x3, y3 = size/2, size * 0.866
    
    print("\n" + "="*50)
    print("📐 SENDING TRIANGLE PATH")
    print("="*50)
    print(f"Size: {size} mm")
    print(f"Speed: {feedrate} mm/min")
    print(f"Points: ({x1:.1f}, {y1:.1f}) -> ({x2:.1f}, {y2:.1f}) -> ({x3:.1f}, {y3:.1f}) -> ({x1:.1f}, {y1:.1f})")
    print("-"*50)
    
    # Начинаем с абсолютного позиционирования
    send_command(sock, "G90")  # Абсолютное позиционирование
    send_command(sock, "G21")  # Единицы в мм
    
    # Движение по треугольнику
    #send_command(sock, f"G1 X{x1:.1f} Y{y1:.1f} F{feedrate}")  # Начальная точка
    send_command(sock, f"G1 X{x2:.1f} Y{y2:.1f} F{feedrate}")  # Первая линия
    send_command(sock, f"G1 X{x3:.1f} Y{y3:.1f} F{feedrate}")  # Вторая линия
    send_command(sock, f"G1 X{x1:.1f} Y{y1:.1f} F{feedrate}")  # Третья линия (замыкаем)
    
    print("="*50)
    print("✅ Triangle path sent!")
    
    sock.close()

if __name__ == "__main__":
    send_triangle()