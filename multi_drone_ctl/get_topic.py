#!/usr/bin/env python3
import math
import random
import threading
import time
import yaml

import numpy as np
from scipy.optimize import least_squares

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from px4_msgs.msg import FailsafeFlags

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class TopicSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('topic_subscriber')
        self.topic_name = topic_name
        self.subscriber = self.create_subscription(
            VehicleLocalPosition,
            topic_name,
            self.listener_callback,
            qos_profile
        )

        self.timer = self.create_timer(1/50, self.timer_callback) 
        self.data = None
        self.position = [0.0, 0.0, 0.0] 

    def listener_callback(self, msg):
        self.data = msg  # 可用于其他用途

        # 拆解消息字段
        # self.get_logger().info("Received VehicleLocalPosition message:")
        # self.get_logger().info(f"  timestamp: {msg.timestamp}")
        # self.get_logger().info(f"  timestamp_sample: {msg.timestamp_sample}")
        # self.get_logger().info(f"  position (x, y, z): ({msg.x}, {msg.y}, {msg.z})")
        self.position = [msg.x, msg.y, msg.z]  # 更新位置
        # self.get_logger().info(f"  velocity (vx, vy, vz): ({msg.vx}, {msg.vy}, {msg.vz})")
        # self.get_logger().info(f"  acceleration (ax, ay, az): ({msg.ax}, {msg.ay}, {msg.az})")
        # self.get_logger().info(f"  heading: {msg.heading} (var: {msg.heading_var})")
        # self.get_logger().info(f"  global flags - xy_global: {msg.xy_global}, z_global: {msg.z_global}")
        # self.get_logger().info(f"  eph: {msg.eph}, epv: {msg.epv}, evh: {msg.evh}, evv: {msg.evv}")
        # self.get_logger().info(f"  dist_bottom: {msg.dist_bottom} (valid: {msg.dist_bottom_valid})")
        # self.get_logger().info(f"  dead_reckoning: {msg.dead_reckoning}")
        # self.get_logger().info(f"  vxy_max: {msg.vxy_max}, vz_max: {msg.vz_max}")
        # self.get_logger().info(f"  hagl_min: {msg.hagl_min}, hagl_max: {msg.hagl_max}")

        # 如果你想展开数组字段
        # delta_xy = msg.delta_xy.tolist()
        # delta_vxy = msg.delta_vxy.tolist()

        # self.get_logger().info(f"  delta_xy: {delta_xy}, reset_counter: {msg.xy_reset_counter}")
        # self.get_logger().info(f"  delta_vxy: {delta_vxy}, reset_counter: {msg.vxy_reset_counter}")
        # self.get_logger().info(f"  delta_z: {msg.delta_z}, reset_counter: {msg.z_reset_counter}")
        # self.get_logger().info(f"  delta_vz: {msg.delta_vz}, reset_counter: {msg.vz_reset_counter}")
        # self.get_logger().info(f"  delta_heading: {msg.delta_heading}, reset_counter: {msg.heading_reset_counter}")

    def timer_callback(self):
        self.get_logger().info(f"Current position: {self.position}")


def get_topic_data(topic_name):
    rclpy.init()
    node = TopicSubscriber(topic_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    return node.data

if __name__ == '__main__':
    rclpy.init()
    print("Starting topic subscriber...")

    topic_name = '/fmu/out/vehicle_local_position'
    node = TopicSubscriber(topic_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    while rclpy.ok():
        if node.data is not None:
            print(f"Received data: {node.data}")
        else:
            print("No data received yet.")

    rclpy.shutdown()

# /fmu/out/failsafe_flags
# FailsafeFlags
# [INFO] [1753346223.848001520] [topic_subscriber]: Received data from /fmu/out/failsafe_flags: 
# px4_msgs.msg.FailsafeFlags(
# timestamp=1129144970, 
# mode_req_angular_velocity=8311934,
# mode_req_attitude=8310910, 
# mode_req_local_alt=8257662, 
# mode_req_local_position=7995448, 
# mode_req_local_position_relaxed=262212, 
# mode_req_global_position=56, 
# mode_req_global_position_relaxed=0, 
# mode_req_mission=8, 
# mode_req_offboard_signal=16384, 
# mode_req_home_position=32, 
# mode_req_wind_and_flight_time_compliance=2621464, 
# mode_req_prevent_arming=3944480, 
# mode_req_manual_control=33863, 
# mode_req_other=2139095040, 
# angular_velocity_invalid=False, 
# attitude_invalid=False, 
# local_altitude_invalid=False, 
# local_position_invalid=True, 
# local_position_invalid_relaxed=True, 
# local_velocity_invalid=True, 
# global_position_invalid=True, 
# global_position_invalid_relaxed=True, 
# auto_mission_missing=True, 
# offboard_control_signal_lost=True, 
# home_position_invalid=True, 
# manual_control_signal_lost=True, 
# gcs_connection_lost=True, 
# battery_warning=0, 
# battery_low_remaining_time=False, 
# battery_unhealthy=False, 
# geofence_breached=False, 
# mission_failure=False, 
# vtol_fixed_wing_system_failure=False, 
# wind_limit_exceeded=False, 
# flight_time_limit_exceeded=False, 
# position_accuracy_low=False, 
# navigator_failure=False, 
# fd_critical_failure=False, 
# fd_esc_arming_failure=False, 
# fd_imbalanced_prop=False, 
# fd_motor_failure=False)
