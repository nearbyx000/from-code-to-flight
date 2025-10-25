# Примерный код узла (WaypointsCommander.py)
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import time

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_commander')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.waypoints = [
            (2.0, 0.0, 0.0),  # x, y, yaw (в радианах)
            (2.0, 2.0, 1.57),
            (0.0, 2.0, 3.14)
        ]

    def send_waypoints(self):
        self.get_logger().info("Ожидание сервера действий 'follow_waypoints'...")
        self._action_client.wait_for_server()

        goal_msg = FollowWaypoints.Goal()
        
        for x, y, yaw in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            # Для простоты ориентация (yaw) задается напрямую
            # В реальном коде нужно конвертировать yaw в кватернион
            pose.pose.orientation.w = 1.0 
            goal_msg.poses.append(pose)

        self.get_logger().info("Начало миссии. Замеряем время.")
        self.time_start = time.time()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Цель отклонена сервером!')
            return

        self.get_logger().info('Цель принята. Ожидание результата...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.time_end = time.time()
        
        if result.missed_waypoints:
            self.get_logger().warn('Миссия завершена, но некоторые точки пропущены.')
        else:
            self.get_logger().info('Миссия успешно завершена!')

        total_time = self.time_end - self.time_start
        self.get_logger().info(f"Общее время миссии: {total_time:.2f} секунд.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    commander = WaypointFollower()
    commander.send_waypoints()
    rclpy.spin(commander)

if __name__ == '__main__':
    main()