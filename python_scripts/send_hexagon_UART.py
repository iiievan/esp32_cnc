import sys
import time
import math
import serial
import serial.tools.list_ports

TARGET_VID = 0x10C4
TARGET_PID = 0xEA60
BAUDRATE = 115200

def find_serial_port():
    """Автопоиск порта по VID:PID"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            return port.device
    return None

def send_command(ser, cmd):
    print(f"Sending: {cmd}")
    ser.write((cmd + "\n").encode('utf-8'))
    time.sleep(0.05) 
    response = b""
    while ser.in_waiting > 0:
        response += ser.readline()
    if response:
        print(f"  -> {response.decode('utf-8').strip()}")

def send_hexagon():
    # Проверяем, передан ли порт аргументом командной строки
    if len(sys.argv) > 1:
        port_name = sys.argv[1]
        print(f"📡 Порт задан вручную: {port_name}")
    else:
        print("🔍 Порт не указан (но его можно узнать через sudo dmesg | grep tty). запускаю автопоиск...")
        port_name = find_serial_port()

    if not port_name:
        print("❌ Ошибка: Не удалось определить порт. Укажите его вручную($: sudo dmesg | grep tty)):")
        print("Пример: python3 send_hexagon.py /dev/ttyUSB0")
        return

    try:
        ser = serial.Serial(port_name, BAUDRATE, timeout=1)
        time.sleep(2) 
        ser.reset_input_buffer()
    except Exception as e:
        print(f"❌ Не удалось открыть порт {port_name}: {e}")
        print(f"Попробуйте добавить права: sudo chmod 666 {port_name}")
        return

    # Параметры геометрии
    radius = 25
    feedrate = 500
    center_x = center_y = radius + 5
    
    # Вычисление вершин
    vertices = []
    for i in range(6):
        angle = math.radians(60 * i + 30)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        vertices.append((x, y))
    
    # Отправка G-кода
    send_command(ser, "G90")
    send_command(ser, "G21")
    send_command(ser, f"G0 X{vertices[0][0]:.1f} Y{vertices[0][1]:.1f}")
    
    for x, y in vertices:
        send_command(ser, f"G1 X{x:.1f} Y{y:.1f} F{feedrate}")
        
    send_command(ser, f"G1 X{vertices[0][0]:.1f} Y{vertices[0][1]:.1f} F{feedrate}")
    
    print("="*60)
    print("✅ Траектория успешно отправлена!")
    ser.close()

if __name__ == "__main__":
    send_hexagon()