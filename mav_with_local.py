import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# Ваши точки из Gazebo (X, Y, Z)
# X=вперед, Y=влево, Z=вверх
gazebo_points = [
    (10.0, 0.0, 5.0),   # 10м вперед, 0м вбок, 5м вверх
    (10.0, 10.0, 5.0),  # 10м вперед, 10м влево, 5м вверх
    (0.0, 10.0, 5.0)    # 0м вперед, 10м влево, 5м вверх
]

# Конвертируем Gazebo (X, Y, Z) в MAVSDK NED (North, East, Down)
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
            
    print("-- Арминг (взвод моторов)")
    await drone.action.arm()
    await asyncio.sleep(1) # Даем время на арминг

    # Взлет на целевую высоту (в нашем случае, первая точка)
    target_altitude = -ned_points[0].down_m  # D = -Z, поэтому -D = Z
    print(f"-- Взлет на {target_altitude} м")
    await drone.action.set_takeoff_altitude(target_altitude)
    await drone.action.takeoff()
    await asyncio.sleep(10) # Даем время на взлет

    print("-- Запуск режима Offboard")
    # Устанавливаем начальную точку для Offboard
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Не удалось запустить Offboard: {error}")
        await drone.action.land()
        return

    # --- Начинаем полет по точкам ---
    for i, point in enumerate(ned_points):
        print(f"-- Летим в точку {i+1}: N:{point.north_m} E:{point.east_m} D:{point.down_m}")
        await drone.offboard.set_position_ned(point)
        await asyncio.sleep(8) # Время на полет до точки

    print("-- Миссия завершена, остановка Offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Не удалось остановить Offboard: {error}")

    print("-- Посадка")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run_mission())