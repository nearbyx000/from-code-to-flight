#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# --- Ваши точки (X=вперед, Y=влево) ---
points_dict = {
    'Н': (0.0, 0.0),
    'А': (2.5, 1.0),
    'В': (6.5, 1.0),
    '1': (0.5, 3.0),
    '2': (7.5, 9.0),
    '3': (0.5, 8.0),
    '4': (7.5, 4.5),
    '5': (4.5, 5.5)
}

# --- Ваш новый маршрут ---
# (Точка 'Н' - это старт, поэтому мы ее не включаем в список целей)
route_keys = ['1', 'В', '5', 'В', '2', 'В', '4', 'А', '3', 'А']

# --- Настройки полета ---
FLYING_ALTITUDE = 5.0 # (метры) - высота полета

# 1. Собираем список координат (X, Y, Z) для Gazebo
gazebo_points = []
for key in route_keys:
    point_xy = points_dict.get(key)
    if point_xy:
        gazebo_points.append((point_xy[0], point_xy[1], FLYING_ALTITUDE))

# 2. Конвертируем Gazebo (X, Y, Z) в MAVSDK NED (North, East, Down)
# N = X, E = -Y, D = -Z
ned_points = [
    PositionNedYaw(p[0], -p[1], -p[2], 0.0) for p in gazebo_points
]


async def run_mission():
    """
    Выполняет полет по точкам в режиме Offboard.
    """
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Ждем подключения к дрону...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Дрон подключен!")
            break

    print("Ждем получения GPS... (нужно для arm)")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Позиция GPS установлена.")
            break
    
    # --- ВАЖНО: ПРОВЕРКА ОШИБОК ARM ---
    print("-- Арминг (взвод моторов)")
    try:
        await drone.action.arm()
    except ActionError as e:
        print(f"!!! ОШИБКА ARM: {e}")
        print("!!! Завершение скрипта.")
        return # Выходим из миссии

    await asyncio.sleep(1) 
    target_altitude_ned = ned_points[0].down_m # Берем высоту из первой точки

    # --- ВАЖНО: ПРОВЕРКА ОШИБОК TAKEOFF ---
    print(f"-- Взлет на {-target_altitude_ned} м")
    try:
        await drone.action.set_takeoff_altitude(-target_altitude_ned)
        await drone.action.takeoff()
    except ActionError as e:
        print(f"!!! ОШИБКА TAKEOFF: {e}")
        print("!!! Завершение скрипта.")
        return # Выходим из миссии

    # Даем дрону время набрать высоту
    await asyncio.sleep(10) 

    print("-- Запуск режима Offboard")
    try:
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, target_altitude_ned, 0.0))
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Не удалось запустить Offboard: {error}")
        await drone.action.land()
        return

    # --- Начинаем полет по точкам ---
    for i, point in enumerate(ned_points):
        print(f"-- Летим в точку {i+1}/{len(ned_points)} (Маршрут: {route_keys[i]})")
        print(f"   Координаты: N:{point.north_m} E:{point.east_m} D:{point.down_m}")
        
        await drone.offboard.set_position_ned(point)
        
        # Ждем 4 секунды. Это МЕНЬШЕ 5-секундного таймаута Offboard.
        # Дрон может не успеть долететь до точки, но он будет 
        # непрерывно двигаться к СЛЕДУЮЩЕЙ точке.
        await asyncio.sleep(4) 

    print("-- Миссия завершена, остановка Offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Не удалось остановить Offboard: {error}")

    print("-- Посадка")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run_mission())