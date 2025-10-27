#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet
from mavros_msgs.msg import State, ParamValue
from clover.srv import SetLEDEffect

class DroneController:
    def __init__(self):
        """Инициализирует узел, переменные, подписчики, издатели и сервисы."""
        rospy.init_node('fly_robotics_final_controller')

        # --- 1. Координаты и маршрут ---
        self.coordinates = {
            '0': (0.0, 0.0, 1.0),  # H (База) - рабочая высота 1м
            'A': (2.5, 1.0, 1.0),
            'B': (6.5, 1.0, 1.0),
            '1': (0.5, 3.0, 1.0),
            '2': (7.5, 9.0, 1.0),
            '3': (0.5, 8.0, 1.0),
            '4': (7.5, 4.5, 1.0),
            '5': (4.5, 5.5, 1.0)
        }
        
        # Корректный маршрут, выполняющий все 5 доставок
        self.route_plan = [
            'B', '1',  # Доставка 1 (B -> 1)
            'B', '5',  # Доставка 2 (B -> 5)
            'B', '2',  # Доставка 3 (B -> 2)
            'A', '4',  # Доставка 4 (A -> 4)
            'A', '3'   # Доставка 5 (A -> 3)
        ]
        
        self.pickup_points = ['A', 'B']
        self.dropoff_points = ['1', '2', '3', '4', '5']
        
        # --- 2. Настройка ROS ---
        self.current_state = None
        self.current_pose = None
        self.rate = rospy.Rate(20.0) # 20 Гц

        # Подписчики
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Издатель
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # --- 3. Клиенты сервисов ---
        rospy.loginfo("Waiting for MAVROS services...")
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.wait_for_service('/mavros/cmd/land')
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        rospy.loginfo("MAVROS services connected.")

        rospy.loginfo("Waiting for Clover LED service...")
        rospy.wait_for_service('/clover/set_led_effect')
        self.set_led_service = rospy.ServiceProxy('/clover/set_led_effect', SetLEDEffect)
        rospy.loginfo("Clover LED service connected.")

        rospy.loginfo("Waiting for MAVROS param set service...")
        rospy.wait_for_service('/mavros/param/set')
        self.set_param_service = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        rospy.loginfo("MAVROS param service connected.")

        # --- 4. Установка параметров ---
        self.set_parameters()

        # --- 5. Подготовка к полету (блокирующая функция) ---
        self.prepare_flight()

    def set_parameters(self):
        """Устанавливает параметры PX4 (скорость) и начальное состояние LED."""
        rospy.loginfo("Setting flight parameters...")
        try:
            # Устанавливаем макс. горизонтальную скорость 1.0 м/с
            pv = ParamValue(integer=0, real=1.0)
            self.set_param_service(param_id='MPC_XY_VEL_MAX', value=pv)
            rospy.loginfo("Set MPC_XY_VEL_MAX to 1.0 m/s.")
            
            # Устанавливаем начальное состояние LED (Выкл)
            self.set_led_service(effect='fill', r=0, g=0, b=0)
            rospy.loginfo("Set initial LED to OFF.")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to set parameters: {e}")

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

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
        """Летит к точке, ждет прибытия и стабилизируется."""
        target_coords = self.coordinates[point_name]
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = target_coords[0]
        pose.pose.position.y = target_coords[1]
        pose.pose.position.z = target_coords[2]
        
        rospy.loginfo(f"Moving to {point_name} {target_coords}...")
        
        # Цикл полета к точке
        while not rospy.is_shutdown():
            if self.is_at_position(target_coords):
                break # Точка достигнута
                
            if (self.current_state and self.current_state.mode != "OFFBOARD"):
                rospy.logwarn("OFFBOARD mode lost! Attempting to recover...")
                # Запрос будет повторен в self.prepare_flight, но мы должны продолжать публиковать
                
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
        
        rospy.loginfo(f"Arrived at {point_name}. Stabilizing...")

        # Цикл стабилизации (чтобы погасить инерцию и не пролететь)
        # 10 циклов * 1/20 Гц = 0.5 сек
        for _ in range(10): 
            if rospy.is_shutdown():
                break
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
        
        rospy.loginfo(f"Stabilized at {point_name}.")

    def prepare_flight(self):
        """Надежный цикл для взлета и перехода в OFFBOARD."""
        rospy.loginfo("Preparing for flight...")
        
        # Ждем подключения к MAVROS
        while not self.current_state and not rospy.is_shutdown():
            rospy.logwarn("Waiting for MAVROS connection...")
            self.rate.sleep()
            
        # "Прогреваем" поток setpoints
        pose = PoseStamped()
        pose.pose.position.z = 1.0 # Стартовая высота
        for _ in range(100):
            if rospy.is_shutdown():
                return
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
            
        rospy.loginfo("Attempting to enable OFFBOARD mode...")
        last_request_time = rospy.Time.now()

        # Цикл, пока не включится OFFBOARD
        while not rospy.is_shutdown() and self.current_state.mode != "OFFBOARD":
            if rospy.Time.now() - last_request_time > rospy.Duration(5.0):
                rospy.loginfo("Requesting OFFBOARD mode...")
                self.set_mode_service(custom_mode="OFFBOARD")
                last_request_time = rospy.Time.now()
            
            self.setpoint_pub.publish(pose)
            self.rate.sleep()

        rospy.loginfo("OFFBOARD mode enabled.")

        # Цикл, пока дрон не заармится
        rospy.loginfo("Attempting to arm...")
        while not rospy.is_shutdown() and not self.current_state.armed:
            rospy.loginfo("Sending arm command...")
            self.arm_service(True)
            self.rate.sleep()
            
        rospy.loginfo("Vehicle armed.")
        
        # Летим на стартовую точку (0,0,1)
        self.go_to_point('0')
        rospy.loginfo("Ready for mission. (Timer starts now)")

    def run_mission(self):
        """Выполняет полетный план по точкам."""
        rospy.loginfo("Starting mission run...")
        
        for point_name in self.route_plan:
            if rospy.is_shutdown():
                break
                
            self.go_to_point(point_name)
            
            try:
                if point_name in self.pickup_points:
                    # "Захват"
                    rospy.loginfo(f"Simulating pickup at {point_name} (3 sec)...")
                    self.set_led_service(effect='fill', r=0, g=255, b=0) # ЗЕЛЕНЫЙ
                    rospy.sleep(3.0)
                
                elif point_name in self.dropoff_points:
                    # "Разгрузка"
                    rospy.loginfo(f"Simulating dropoff at {point_name} (3 sec)...")
                    self.set_led_service(effect='fill', r=0, g=0, b=0) # ВЫКЛ
                    rospy.sleep(3.0)

            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to set LED: {e}")
            except rospy.ROSInterruptException:
                rospy.loginfo("Shutdown requested during sleep.")
                break

        # --- Завершение миссии ---
        if not rospy.is_shutdown():
            rospy.loginfo("Mission complete. Returning to base...")
            self.go_to_point('0')
            
            try:
                self.set_led_service(effect='fill', r=0, g=0, b=0) # Финальное ВЫКЛ
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to set final LED state: {e}")

            rospy.loginfo("Stabilizing at base... (Timer stops)")
            
            rospy.loginfo("Executing land...")
            self.land_service(altitude=0, latitude=0, longitude=0, min_pitch=0)
            rospy.sleep(5)
            
        rospy.loginfo("Mission finished.")


if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run_mission()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received. Shutting down.")
    except Exception as e:
        rospy.logerr(f"An unhandled exception occurred: {e}")