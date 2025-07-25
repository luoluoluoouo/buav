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
            FailsafeFlags,
            topic_name,
            self.listener_callback,
            qos_profile
        )
        self.data = None

    def listener_callback(self, msg):
        self.data = msg
        if self.data is None:
            self.get_logger().warn(f'No data received from {self.topic_name}')
        else:
            # self.get_logger().info(f'Received data from {self.topic_name}: {msg}')
            offboard_control_signal_lost = self.data.offboard_control_signal_lost
            self.get_logger().info(f"Offboard Control Signal Lost: {offboard_control_signal_lost}")

def get_topic_data(topic_name):
    rclpy.init()
    node = TopicSubscriber(topic_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    return node.data

if __name__ == '__main__':
    try:
        topic_name = '/fmu/out/failsafe_flags'
        data = get_topic_data(topic_name)
        if data:
            print(f"Data received from {topic_name}: {data}")
        else:
            print(f"No data received from {topic_name}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
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
