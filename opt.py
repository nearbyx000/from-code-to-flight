import math
import itertools

def calculate_distance(p1, p2):
    """Рассчитывает 2D-Евклидово расстояние между двумя кортежами (x, y)."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# 1. Задаем координаты всех точек
coordinates = {
    '0': (0.0, 0.0),  # H (База)
    'A': (2.5, 1.0),
    'B': (6.5, 1.0),
    '1': (0.5, 3.0),
    '2': (7.5, 9.0),
    '3': (0.5, 8.0),
    '4': (7.5, 4.5),
    '5': (4.5, 5.5)
}

# 2. Определяем 5 обязательных "поручений" (доставки)
# [cite_start]Грузы A -> 3 и 4 [cite: 49]
# [cite_start]Грузы B -> 1, 2 и 5 [cite: 50]
tasks = [
    ('A', '3'),
    ('A', '4'),
    ('B', '1'),
    ('B', '2'),
    ('B', '5')
]

min_total_distance = float('inf')
optimal_route = []

# 3. Генерируем все 120 (5!) вариантов порядка
task_permutations = list(itertools.permutations(tasks))

print(f"Начинаю расчет 120 возможных маршрутов...")

# 4. Перебираем каждый вариант
for task_order in task_permutations:
    current_distance = 0
    current_route = ['0']
    last_point_name = '0'
    
    # 5. Считаем полный путь для этого порядка
    for task in task_order:
        pickup, dropoff = task
        
        # Дистанция: Прошлая точка -> Точка захвата
        current_distance += calculate_distance(coordinates[last_point_name], coordinates[pickup])
        # Дистанция: Точка захвата -> Точка выгрузки
        current_distance += calculate_distance(coordinates[pickup], coordinates[dropoff])
        
        current_route.append(pickup)
        current_route.append(dropoff)
        last_point_name = dropoff # Обновляем, где мы сейчас
        
    # [cite_start]6. Добавляем путь домой (Последняя точка -> База) [cite: 71]
    current_distance += calculate_distance(coordinates[last_point_name], coordinates['0'])
    current_route.append('0')
    
    # 7. Сравниваем и сохраняем, если это лучший
    if current_distance < min_total_distance:
        min_total_distance = current_distance
        optimal_route = current_route

# --- 8. Расчет времени для лучшего маршрута ---

# [cite_start]Время зависания = 5 захватов * 3 сек + 5 разгрузок * 3 сек = 30 сек [cite: 24, 26, 56]
hover_time = (5 * 3.0) + (5 * 3.0) 
# [cite_start]Время полета = Дистанция / 1.0 м/с [cite: 16]
flight_time = min_total_distance / 1.0 
total_time = flight_time + hover_time

# --- 9. Вывод результатов ---

print("\n--- ОПТИМИЗАЦИЯ ЗАВЕРШЕНА ---")
print(f"Найден оптимальный маршрут:")
print(f"  { ' -> '.join(optimal_route) }")
print(f"Общая дистанция: {min_total_distance:.4f} м")

print("\n--- РАСЧЕТ ВРЕМЕНИ ---")
print(f"  Время полета (Дистанция / 1.0 м/с): {flight_time:.4f} сек")
print(f"  Время зависаний (10 * 3 сек):       {hover_time:.1f} сек")
print(f"  Общее время миссии:                 {total_time:.4f} сек")

print("\n--- ГОТОВЫЙ ФАЙЛ route.txt ---")
# [cite_start]Формат: (0,A,3,B,1,...;81.7) [cite: 70, 78]
route_str = ",".join(optimal_route)
time_str = f"{total_time:.1f}" # Округление до десятых
print(f"({route_str};{time_str})")