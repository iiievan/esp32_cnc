import os
import matplotlib
matplotlib.use('Agg')
import numpy as np
import re
from matplotlib import pyplot as plt

def parse_motion_log(log_content):
    """
    Парсит логи движения в формате:
    pos_x_mm:pos_y_mm:curr_vel:T_ms:dt_ms
    """
    # Ищем строки с данными движения (числовые значения через двоеточие)
    # Паттерн для строк с данными: начинаются с числа, затем : и еще числа
    pattern = re.compile(r'(\d+\.?\d*):(\d+\.?\d*):(\d+\.?\d*):(\d+):(\d+)')
    matches = pattern.findall(log_content)
    
    if not matches:
        print("No motion data found in log!")
        return None
    
    # Преобразуем данные в numpy массивы
    data = []
    for match in matches:
        # Конвертируем в float
        values = [float(v) for v in match]
        data.append(values)
    
    data = np.array(data)
    
    # Создаем структурированный массив
    dtype = [
        ('pos_x_mm', 'float64'),   # позиция X в мм
        ('pos_y_mm', 'float64'),   # позиция Y в мм
        ('vel_mm_min', 'float64'), # скорость в мм/мин
        ('T_ms', 'float64'),       # абсолютное время в мс
        ('dt_ms', 'float64')       # разница времени в мс
    ]
    
    structured_data = np.array([tuple(row) for row in data], dtype=dtype)
    
    return structured_data

def plot_velocity_vs_time(motion_data, output_dir):
    """
    Строит график скорости от времени (S-образная кривая)
    """
    time = motion_data['T_ms'] / 1000.0  # переводим в секунды
    velocity = motion_data['vel_mm_min']  # скорость в мм/мин
    
    # Переводим время в секунды от начала движения
    time_start = time[0]
    time_rel = time - time_start
    
    plt.figure(figsize=(28, 16))
    plt.title('S-curve Velocity Profile vs Time', fontsize=16)
    plt.plot(time_rel, velocity, linewidth=2, color='blue')
    plt.xlabel('Time [s]', fontsize=14)
    plt.ylabel('Velocity [mm/min]', fontsize=14)
    plt.grid(True, alpha=0.3)
    
    # Добавляем информацию на график
    max_vel = np.max(velocity)
    max_vel_idx = np.argmax(velocity)
    max_vel_time = time_rel[max_vel_idx]
    plt.axhline(y=max_vel, color='red', linestyle='--', alpha=0.5, 
                label=f'Max velocity: {max_vel:.1f} mm/min at {max_vel_time:.2f}s')
    plt.legend(fontsize=12)
    
    # Добавляем аннотации для начала и конца движения
    plt.axvline(x=time_rel[0], color='green', linestyle='--', alpha=0.3)
    plt.axvline(x=time_rel[-1], color='green', linestyle='--', alpha=0.3,
                label=f'Total time: {time_rel[-1]:.2f}s')
    plt.legend(fontsize=12)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'velocity_profile.png'), dpi=150)
    plt.close()
    print(f"✅ Velocity profile saved to {output_dir}/velocity_profile.png")

def plot_xy_trajectory(motion_data, output_dir):
    """
    Строит график траектории в координатах X-Y
    """
    pos_x = motion_data['pos_x_mm']
    pos_y = motion_data['pos_y_mm']
    
    # Получаем начальную и конечную позиции
    start_x, start_y = pos_x[0], pos_y[0]
    end_x, end_y = pos_x[-1], pos_y[-1]
    
    # Вычисляем расстояние и направление
    distance = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
    
    plt.figure(figsize=(24, 24))
    plt.title(f'XY Trajectory (Distance: {distance:.2f} mm)', fontsize=16)
    plt.plot(pos_x, pos_y, linewidth=2, color='blue', label='Path')
    
    # Отмечаем начальную и конечную точки
    plt.plot(start_x, start_y, 'go', markersize=12, label=f'Start ({start_x:.1f}, {start_y:.1f})')
    plt.plot(end_x, end_y, 'ro', markersize=12, label=f'End ({end_x:.1f}, {end_y:.1f})')
    
    # Добавляем стрелку направления движения (если есть движение)
    if len(pos_x) > 10:
        # Берем среднюю точку для стрелки
        mid_idx = len(pos_x) // 2
        dx = pos_x[mid_idx + 1] - pos_x[mid_idx] if mid_idx + 1 < len(pos_x) else 0
        dy = pos_y[mid_idx + 1] - pos_y[mid_idx] if mid_idx + 1 < len(pos_y) else 0
        if dx != 0 or dy != 0:
            # Нормализуем и рисуем стрелку
            arrow_length = 0.5
            dx_norm = dx / np.sqrt(dx**2 + dy**2) * arrow_length
            dy_norm = dy / np.sqrt(dx**2 + dy**2) * arrow_length
            plt.arrow(pos_x[mid_idx], pos_y[mid_idx], dx_norm, dy_norm,
                     head_width=0.3, head_length=0.3, fc='red', ec='red', alpha=0.5)
    
    # Настраиваем оси для квадратного отображения
    x_range = np.max(pos_x) - np.min(pos_x)
    y_range = np.max(pos_y) - np.min(pos_y)
    max_range = max(x_range, y_range)
    if max_range > 0:
        plt.xlim(np.mean(pos_x) - max_range*0.6, np.mean(pos_x) + max_range*0.6)
        plt.ylim(np.mean(pos_y) - max_range*0.6, np.mean(pos_y) + max_range*0.6)
    
    plt.xlabel('X Position [mm]', fontsize=14)
    plt.ylabel('Y Position [mm]', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=12)
    plt.axis('equal')  # Сохраняем пропорции осей
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'xy_trajectory.png'), dpi=150)
    plt.close()
    print(f"✅ XY trajectory saved to {output_dir}/xy_trajectory.png")

def plot_velocity_vs_position(motion_data, output_dir):
    """
    Дополнительный график: скорость от пройденного расстояния
    Показывает, как скорость меняется вдоль пути
    """
    pos_x = motion_data['pos_x_mm']
    pos_y = motion_data['pos_y_mm']
    velocity = motion_data['vel_mm_min']
    
    # Вычисляем пройденное расстояние по траектории
    distance = np.zeros(len(pos_x))
    for i in range(1, len(pos_x)):
        dx = pos_x[i] - pos_x[i-1]
        dy = pos_y[i] - pos_y[i-1]
        distance[i] = distance[i-1] + np.sqrt(dx**2 + dy**2)
    
    plt.figure(figsize=(24, 16))
    plt.title('Velocity vs Distance Along Path', fontsize=16)
    plt.plot(distance, velocity, linewidth=2, color='green')
    plt.xlabel('Distance along path [mm]', fontsize=14)
    plt.ylabel('Velocity [mm/min]', fontsize=14)
    plt.grid(True, alpha=0.3)
    
    # Отмечаем максимальную скорость
    max_vel_idx = np.argmax(velocity)
    plt.plot(distance[max_vel_idx], velocity[max_vel_idx], 'ro', 
             label=f'Max velocity: {velocity[max_vel_idx]:.1f} mm/min at {distance[max_vel_idx]:.2f}mm')
    plt.legend(fontsize=12)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'velocity_vs_distance.png'), dpi=150)
    plt.close()
    print(f"✅ Velocity vs distance saved to {output_dir}/velocity_vs_distance.png")

def print_statistics(motion_data):
    """
    Выводит статистику движения
    """
    time = motion_data['T_ms'] / 1000.0
    time_rel = time - time[0]
    velocity = motion_data['vel_mm_min']
    
    # Вычисляем расстояния
    pos_x = motion_data['pos_x_mm']
    pos_y = motion_data['pos_y_mm']
    start_pos = np.array([pos_x[0], pos_y[0]])
    end_pos = np.array([pos_x[-1], pos_y[-1]])
    total_distance = np.linalg.norm(end_pos - start_pos)
    
    # Вычисляем путь (интегрируем скорость)
    path_length = 0
    for i in range(1, len(pos_x)):
        dx = pos_x[i] - pos_x[i-1]
        dy = pos_y[i] - pos_y[i-1]
        path_length += np.sqrt(dx**2 + dy**2)
    
    print("\n" + "="*60)
    print("📊 MOVEMENT STATISTICS")
    print("="*60)
    print(f"Start position: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) mm")
    print(f"End position:   ({end_pos[0]:.2f}, {end_pos[1]:.2f}) mm")
    print(f"Total displacement: {total_distance:.2f} mm")
    print(f"Path length:        {path_length:.2f} mm")
    print(f"Total time:         {time_rel[-1]:.2f} s")
    print(f"Max velocity:       {np.max(velocity):.2f} mm/min")
    print(f"Min velocity:       {np.min(velocity):.2f} mm/min")
    print(f"Average velocity:   {np.mean(velocity):.2f} mm/min")
    print(f"Number of samples:  {len(motion_data)}")
    print("="*60)

def parse_legacy_motion_log(log_content, tag):
    """
    Сохранение совместимости со старым скриптом
    """
    # Ищем строки с данными движения
    pattern = re.compile(rf'\[{tag}\]:\s*([\d\.\-]+):([\d\.\-]+):([\d\.\-]+):(\d+):(\d+)')
    matches = pattern.findall(log_content)
    
    if not matches:
        return None
    
    data = []
    for match in matches:
        values = [float(v) for v in match]
        data.append(values)
    
    data = np.array(data)
    
    dtype = [
        ('pos_x_mm', 'float64'),
        ('pos_y_mm', 'float64'),
        ('vel_mm_min', 'float64'),
        ('T_ms', 'float64'),
        ('dt_ms', 'float64')
    ]
    
    return np.array([tuple(row) for row in data], dtype=dtype)

def main():
    # Создаем директорию для графиков
    output_dir = 'movement_plots'
    os.makedirs(output_dir, exist_ok=True)
    
    # Читаем лог-файл
    log_file = 'cnc_server.log'  # Или замените на путь к вашему файлу
    
    try:
        with open(log_file, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"❌ File '{log_file}' not found!")
        print("Please specify the correct path to your log file.")
        return
    
    # Парсим логи движения
    print("🔍 Parsing motion logs...")
    motion_data = parse_motion_log(content)
    
    if motion_data is None:
        print("❌ No motion data found in the log file!")
        print("Looking for data lines in format: 'pos_x:pos_y:velocity:T_ms:dt_ms'")
        return
    
    print(f"✅ Found {len(motion_data)} data points")
    
    # Выводим статистику
    print_statistics(motion_data)
    
    # Строим графики
    print("\n📈 Generating plots...")
    plot_velocity_vs_time(motion_data, output_dir)
    plot_xy_trajectory(motion_data, output_dir)
    plot_velocity_vs_position(motion_data, output_dir)
    
    print(f"\n✅ All plots saved to '{output_dir}/'")
    print(f"   - velocity_profile.png (S-curve velocity vs time)")
    print(f"   - xy_trajectory.png (XY path)")
    print(f"   - velocity_vs_distance.png (velocity along path)")

if __name__ == "__main__":
    main()