#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
import math

class SmartCartBoxFollower(Node):
    def __init__(self):
        super().__init__("smart_cart_follower")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_subscription = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.gazebo_subscription = self.create_subscription(ModelStates, "/gazebo/model_states", self.gazebo_callback, 10)
        self.target_dist = 1.2
        self.emergency_dist = 0.5
        self.lidar_emergency = False
        self.current_lidar_dist = 99.0
        self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0
        self.box_x, self.box_y = 1.5, 0.0
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Box Tracking Ready.")

    def lidar_callback(self, msg):
        front_angles = list(range(0, 45)) + list(range(315, 360))
        valid_ranges = [msg.ranges[a] for a in front_angles if msg.range_min < msg.ranges[a] < msg.range_max]
        if valid_ranges: self.current_lidar_dist = min(valid_ranges); self.lidar_emergency = True if self.current_lidar_dist <= self.emergency_dist else False
        else: self.current_lidar_dist = 99.0; self.lidar_emergency = False

    def gazebo_callback(self, msg):
        target_name = "box" if "box" in msg.name else ("person" if "person" in msg.name else None)
        if "burger" in msg.name and target_name:
            r_idx = msg.name.index("burger")
            t_idx = msg.name.index(target_name)
            self.robot_x = msg.pose[r_idx].position.x
            self.robot_y = msg.pose[r_idx].position.y
            q = msg.pose[r_idx].orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y); cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.box_x = msg.pose[t_idx].position.x
            self.box_y = msg.pose[t_idx].position.y

    def control_loop(self):
        twist = Twist(); dx = self.box_x - self.robot_x; dy = self.box_y - self.robot_y
        rel_dist = math.sqrt(dx*dx + dy*dy); target_angle = math.atan2(dy, dx); angle_error = target_angle - self.robot_yaw
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi
        virtual_marker_x_offset = angle_error / (math.pi / 4.0)
        if self.lidar_emergency:
            twist.linear.x = 0.0; twist.angular.z = 0.0
            self.get_logger().warn(f"[SAFETY] EMERGENCY STOP: {self.current_lidar_dist:.2f}m")
        else:
            error_dist = rel_dist - self.target_dist
            twist.linear.x = min(error_dist * 0.5, 0.18) if error_dist > 0.1 else (max(error_dist * 0.5, -0.12) if error_dist < -0.1 else 0.0)
            twist.angular.z = max(min(virtual_marker_x_offset * 1.2, 0.6), -0.6) if abs(virtual_marker_x_offset) > 0.05 else 0.0
            self.get_logger().info(f"[VISION-SIM] Target Dist: {rel_dist:.2f}m, Offset: {virtual_marker_x_offset:.2f}")
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args); node = SmartCartBoxFollower()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.publisher_.publish(Twist()); node.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
