import socket
import math

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

def send_hexagon():
    """Отправляет команды для вырезания правильного шестиугольника в первом квадранте"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Параметры шестиугольника
    radius = 25  # радиус описанной окружности в мм
    feedrate = 500  # скорость в мм/мин
    
    # Смещаем центр в положительную область
    # Чтобы шестиугольник полностью помещался в первом квадранте
    # Минимальное смещение = radius + небольшой запас
    offset_x = radius + 5  # смещение по X
    offset_y = radius + 5  # смещение по Y
    
    center_x = offset_x
    center_y = offset_y
    
    print("\n" + "="*60)
    print("⬡ SENDING HEXAGON PATH (Positive Coordinates Only)")
    print("="*60)
    print(f"Radius: {radius} mm")
    print(f"Speed: {feedrate} mm/min")
    print(f"Center: ({center_x:.1f}, {center_y:.1f}) mm")
    print(f"Offset: X={offset_x:.1f}, Y={offset_y:.1f} mm")
    print("-"*60)
    
    # Вычисляем вершины шестиугольника со смещением
    vertices = []
    for i in range(6):
        angle = math.radians(60 * i + 30)  # +30 для "плоской" ориентации
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        vertices.append((x, y))
    
    # Показываем вершины
    for i, (x, y) in enumerate(vertices, 1):
        print(f"Vertex {i}: ({x:.2f}, {y:.2f})")
    print("-"*60)
    
    # Настройка
    send_command(sock, "G90")  # Абсолютное позиционирование
    send_command(sock, "G21")  # Единицы в мм
    
    # Быстрый переход к первой вершине
    send_command(sock, f"G0 X{vertices[0][0]:.1f} Y{vertices[0][1]:.1f}")
    
    # Режем по всем вершинам
    for x, y in vertices:
        send_command(sock, f"G1 X{x:.1f} Y{y:.1f} F{feedrate}")
    
    # Замыкаем шестиугольник
    send_command(sock, f"G1 X{vertices[0][0]:.1f} Y{vertices[0][1]:.1f} F{feedrate}")
    
    print("="*60)
    print("✅ Hexagon path sent!")
    print(f"All coordinates are positive (X >= 0, Y >= 0)")
    
    sock.close()

if __name__ == "__main__":
    send_hexagon()