#!/usr/-bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
import math

# ПРЕДПОЛАГАЕТСЯ, что сервис для светодиодов - это стандартный SetBool
# Замените 'set_led' на имя реального сервиса из документации олимпиады
from std_srvs.srv import SetBool 

class DroneController:
    def __init__(self):
        rospy.init_node('fly_robotics_controller')

        # --- 1. Координаты и маршрут (из вашего запроса) ---
        self.coordinates = {
            '0': (0.0, 0.0, 1.0),  # H (База) - всегда на Z=1 для полета
            'A': (2.5, 1.0, 1.0),
            'B': (6.5, 1.0, 1.0),
            '1': (0.5, 3.0, 1.0),
            '2': (7.5, 9.0, 1.0),
            '3': (0.5, 8.0, 1.0),
            '4': (7.5, 4.5, 1.0),
            '5': (4.5, 5.5, 1.0)
        }
        
        # Исправленный, корректный маршрут
        self.route_plan = ['B', '1', 'B', '5', 'B', '2', 'A', '4', 'A', '3']
        
        self.pickup_points = ['A', 'B']
        self.dropoff_points = ['1', '2', '3', '4', '5']
        
        # --- 2. Настройка ROS ---
        self.current_state = None
        self.current_pose = None
        self.rate = rospy.Rate(20.0)

        # Подписчики
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Публикаторы
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Клиенты сервисов
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.wait_for_service('/mavros/cmd/land')
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        # Сервис светодиодов (!! ЗАМЕНИТЬ НА РЕАЛЬНЫЙ СЕРВИС !!)
        try:
            rospy.wait_for_service('set_led', timeout=2.0)
            self.set_led_service = rospy.ServiceProxy('set_led', SetBool)
        except rospy.ROSException:
            rospy.logwarn("Сервис 'set_led' не найден. Имитация работы.")
            self.set_led_service = None

        # --- 3. Подготовка к полету ---
        self.prepare_flight()

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def set_led(self, state):
        """Управляет светодиодной лентой."""
        if self.set_led_service is None:
            rospy.loginfo(f"[ИМИТАЦИЯ] LED: {'ЗЕЛЕНЫЙ' if state else 'ВЫКЛ'}")
            return
            
        try:
            # True = зеленый [cite: 23], False = выкл [cite: 25]
            self.set_led_service(state) 
        except rospy.ServiceException as e:
            rospy.logerr(f"Ошибка вызова сервиса LED: {e}")

    def is_at_position(self, target_coords, tolerance=0.3):
        """Проверяет, достиг ли дрон цели с допуском."""
        if self.current_pose is None:
            return False
        
        pos = self.current_pose.pose.position
        dist = math.sqrt(
            (pos.x - target_coords[0])**2 +
            (pos.y - target_coords[1])**2 +
            (pos.z - target_coords[2])**2
        )
        return dist < tolerance

    def go_to_point(self, point_name):
        """Летит к точке и ждет ее достижения."""
        target_coords = self.coordinates[point_name]
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = target_coords[0]
        pose.pose.position.y = target_coords[1]
        pose.pose.position.z = target_coords[2]
        
        rospy.loginfo(f"Движение к {point_name} {target_coords}...")
        
        while not self.is_at_position(target_coords) and not rospy.is_shutdown():
            if (self.current_state and self.current_state.mode != "OFFBOARD"):
                rospy.logwarn("Режим OFFBOARD потерян! Попытка восстановления...")
                self.set_mode_service(custom_mode="OFFBOARD")
                
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
        
        rospy.loginfo(f"Прибытие в {point_name}.")

    def prepare_flight(self):
        """Взлет и переход в OFFBOARD."""
        rospy.loginfo("Подготовка к полету...")
        
        # Ждем подключения к FCU
        while not self.current_state and not rospy.is_shutdown():
            self.rate.sleep()
            
        # Отправляем несколько setpoints перед переключением режима
        pose = PoseStamped()
        pose.pose.position.z = 1.0 # Стартовая высота [cite: 19]
        for _ in range(100):
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
            
        # Включаем OFFBOARD и Arm
        self.set_mode_service(custom_mode="OFFBOARD")
        self.arm_service(True)
        
        # Ждем стабилизации на (0,0,1)
        self.go_to_point('0')
        rospy.loginfo("Готов к миссии. (Время отсчета пошло) [cite: 19]")

    def run_mission(self):
        """Выполняет полет по заданному маршруту."""
        
        for point_name in self.route_plan:
            # Летим к следующей точке
            self.go_to_point(point_name)
            
            # Выполняем действие в точке
            if point_name in self.pickup_points:
                # "Захват" 
                rospy.loginfo(f"Захват груза в {point_name}...")
                self.set_led(True) # Включить зеленый [cite: 23]
                rospy.sleep(3.0) # Зависание 3 сек 
            
            elif point_name in self.dropoff_points:
                # "Разгрузка" 
                rospy.loginfo(f"Разгрузка груза в {point_name}...")
                self.set_led(False) # Выключить [cite: 25]
                rospy.sleep(3.0) # Зависание 3 сек 
        
        # --- 4. Завершение миссии ---
        rospy.loginfo("Миссия выполнена. Возвращение на базу...")
        self.go_to_point('0') # Возврат в (0,0,1) [cite: 20]
        
        rospy.loginfo("Стабилизация на базе... (Окончание времени) [cite: 21]")
        # (Можно добавить проверку стабилизации скоростей)
        
        rospy.loginfo("Выполнение посадки...")
        self.land_service(altitude=0, latitude=0, longitude=0, min_pitch=0) # Посадка в (0,0,0) [cite: 22]
        rospy.sleep(5) # Даем время на посадку
        rospy.loginfo("Миссия завершена.")


if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run_mission()
    except rospy.ROSInterruptException:
        pass