#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
import math

class DroneController:
    def __init__(self):
        rospy.init_node('fly_robotics_mvp_controller')

        # --- 1. Coordinates and Route (from our discussion) ---
        self.coordinates = {
            '0': (0.0, 0.0, 1.0),  # H (Base) - working altitude 1m
            'A': (2.5, 1.0, 1.0),
            'B': (6.5, 1.0, 1.0),
            '1': (0.5, 3.0, 1.0),
            '2': (7.5, 9.0, 1.0),
            '3': (0.5, 8.0, 1.0),
            '4': (7.5, 4.5, 1.0),
            '5': (4.5, 5.5, 1.0)
        }
        
        # Correct route, performs all 5 deliveries
        self.route_plan = [
            'B', '1',  # Delivery 1 (B -> 1)
            'B', '5',  # Delivery 2 (B -> 5)
            'B', '2',  # Delivery 3 (B -> 2)
            'A', '4',  # Delivery 4 (A -> 4)
            'A', '3'   # Delivery 5 (A -> 3)
        ]
        
        self.pickup_points = ['A', 'B']
        self.dropoff_points = ['1', '2', '3', '4', '5']
        
        # --- 2. ROS Setup ---
        self.current_state = None
        self.current_pose = None
        self.rate = rospy.Rate(20.0) # 20 Hz for MAVROS

        # Subscribers
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Publisher
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # MAVROS Service Clients
        rospy.loginfo("Waiting for MAVROS services...")
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.wait_for_service('/mavros/cmd/land')
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        rospy.loginfo("MAVROS services connected.")

        # MVP: LED service is skipped

        # --- 3. Prepare for flight ---
        self.prepare_flight()

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def is_at_position(self, target_coords, tolerance=0.3):
        """Checks if the drone has reached the target with a tolerance."""
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
        """Flies to a point by its name and waits for arrival."""
        target_coords = self.coordinates[point_name]
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = target_coords[0]
        pose.pose.position.y = target_coords[1]
        pose.pose.position.z = target_coords[2]
        
        rospy.loginfo(f"Moving to {point_name} {target_coords}...")
        
        while not rospy.is_shutdown():
            if self.is_at_position(target_coords):
                break
                
            if (self.current_state and self.current_state.mode != "OFFBOARD"):
                rospy.logwarn("OFFBOARD mode lost! Attempting to recover...")
                
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
        
        rospy.loginfo(f"Arrived at {point_name}.")

    def prepare_flight(self):
        """Takeoff and switch to OFFBOARD."""
        rospy.loginfo("Preparing for flight...")
        
        # Wait for connection to FCU
        while not self.current_state and not rospy.is_shutdown():
            rospy.logwarn("Waiting for MAVROS connection...")
            self.rate.sleep()
            
        # "Warm up" the setpoint stream, this is a MAVROS requirement
        pose = PoseStamped()
        pose.pose.position.z = 1.0 # Start altitude
        for _ in range(100):
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
            
        # Enable OFFBOARD
        rospy.loginfo("Enabling OFFBOARD mode...")
        self.set_mode_service(custom_mode="OFFBOARD")
        
        # Arm the drone
        rospy.loginfo("Arming...")
        self.arm_service(True)
        
        # Wait for stabilization at (0,0,1)
        self.go_to_point('0')
        rospy.loginfo("Ready for mission. (Timer starts now)")

    def run_mission(self):
        """Executes the flight plan."""
        
        for point_name in self.route_plan:
            # Fly to the next point
            self.go_to_point(point_name)
            
            # Perform the mandatory action at the point
            if point_name in self.pickup_points:
                # "Pickup"
                rospy.loginfo(f"Simulating pickup at {point_name} (3 sec)...")
                rospy.sleep(3.0) # Mandatory hover
            
            elif point_name in self.dropoff_points:
                # "Dropoff"
                rospy.loginfo(f"Simulating dropoff at {point_name} (3 sec)...")
                rospy.sleep(3.0) # Mandatory hover
        
        # --- Mission Completion ---
        rospy.loginfo("Mission complete. Returning to base...")
        self.go_to_point('0') # Return to (0,0,1)
        
        rospy.loginfo("Stabilizing at base... (Timer stops)")
        
        rospy.loginfo("Executing land...")
        self.land_service(altitude=0, latitude=0, longitude=0, min_pitch=0) # Land
        rospy.sleep(5) # Give it time to land
        rospy.loginfo("Mission finished.")


if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run_mission()
    except rospy.ROSInterruptException:
        pass