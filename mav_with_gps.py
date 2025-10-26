#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

async def run_gps_mission():
    """
    Загружает и выполняет миссию по GPS-точкам.
    """

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Ждем подключения к дрону...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Дрон подключен!")
            break

    # Точки миссии (Широта, Долгота, Относительная высота, Угол)
    # Эти координаты нужно брать из симулятора (например, из QGroundControl)
    # Это примерные координаты для стандартной точки старта в Gazebo
    mission_items = [
        MissionItem(47.397742,   # Широта
                    8.545594,    # Долгота
                    10,          # Относительная высота (метры)
                    10,          # Скорость (м/с)
                    True,        # Пролетать сквозь точку (не останавливаться)
                    float('nan'), float('nan'), float('nan'), # Параметры камеры
                    float('nan'), float('nan'), float('nan'), # Параметры gimbal
                    float('nan')), # Угол рыскания (NaN = смотрит на след. точку)
        
        MissionItem(47.397926,
                    8.545557,
                    10, 10, True,
                    float('nan'), float('nan'), float('nan'),
                    float('nan'), float('nan'), float('nan'),
                    float('nan')),
        
        MissionItem(47.397963,
                    8.545737,
                    10, 10, True,
                    float('nan'), float('nan'), float('nan'),
                    float('nan'), float('nan'), float('nan'),
                    float('nan'))
    ]

    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Загрузка миссии на дрон...")
    await drone.mission.upload_mission(mission_plan)
    print("Миссия загружена.")

    print("-- Арминг")
    await drone.action.arm()

    print("-- Запуск миссии")
    await drone.mission.start_mission()

    # Ждем завершения миссии
    async for mission_progress in drone.mission.mission_progress():
        print(f"Миссия: {mission_progress.current}/{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("Миссия завершена")
            break
    
    # Дрон должен автоматически вернуться и сесть (RTL),
    # так как мы установили set_return_to_launch_after_mission(True)

if __name__ == "__main__":
    asyncio.run(run_gps_mission())