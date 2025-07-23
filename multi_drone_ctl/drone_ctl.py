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

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, name: str, prefix: str, target_system: int, gazebo_position: tuple) -> None:
        super().__init__(f'offboard_control_{name}')

        self.prefix = prefix
        self.target_system = target_system
        self.gazebo_x, self.gazebo_y, self.gazebo_z = gazebo_position
        self.get_logger().warn(f'{prefix} + {target_system}')

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{prefix}/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{prefix}/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{prefix}/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

        self.timer = self.create_timer(0.05, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        # self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        # self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        # self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        # if self.vehicle_local_position.z > self.takeoff_height \
        #   and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        # elif self.vehicle_local_position.z <= self.takeoff_height:
        #     self.land()

        self.x = self.vehicle_local_position.x
        self.y = self.vehicle_local_position.y
        self.z = self.vehicle_local_position.z

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def set_position(self, target_position: tuple):
        target_x, target_y, target_z = target_position
        x = target_y - self.gazebo_y
        y = target_x - self.gazebo_x
        z = self.gazebo_z - target_z
        self.publish_position_setpoint(x, y, z)

    def get_position(self):
        self.abs_x = self.gazebo_x + self.vehicle_local_position.y
        self.abs_y = self.gazebo_y + self.vehicle_local_position.x
        self.abs_z = self.gazebo_z - self.vehicle_local_position.z
        return (self.abs_x, self.abs_y, self.abs_z)

class BLEbeacon(Node):
    def __init__(self, node_name: str, position: tuple, rssi_settings: dict) -> None:
        super().__init__(f'{node_name}')

        self.position = position

        # 模擬參數
        self.rssi0 = rssi_settings['rssi0'] + random.uniform(-5, 5)  # 基準 RSSI 值，隨機偏移
        self.path_loss_n = rssi_settings['path_loss_n'] + random.uniform(-0.1, 0.1)  # 路徑損失指數，隨機偏移
        self.noise_stddev = rssi_settings['noise_stddev']  # dBm 隨機雜訊

    def get_rssi(self, drone_position: tuple):
        drone_x, drone_y, drone_z = drone_position

        # 計算與 beacon 的距離
        dx = drone_x - self.position[0]
        dy = drone_y - self.position[1]
        dz = drone_z - self.position[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        distance = max(distance, 0.1)  # 避免 log(0)

        # 使用 log-distance 模型模擬 RSSI
        rssi = self.rssi0 - 10 * self.path_loss_n * math.log10(distance)

        # 加上隨機 noise
        # noise = random.gauss(0, self.noise_stddev)
        # rssi += noise

        return rssi

class Controller():
    def __init__(self):
        self.drones = []
        drone_1 = OffboardControl(name = 'drone_1', \
                                    prefix= '/px4_1', \
                                    target_system=2, \
                                    gazebo_position=(0.0, 3.0, 0.0))
        self.drones.append(drone_1)
        drone_2 = OffboardControl(name = 'drone_2', \
                                    prefix= '/px4_2', \
                                    target_system=3, \
                                    gazebo_position=(0.0, 6.0, 0.0))
        self.drones.append(drone_2)

        self.beacons = []
        self.RSSI_SETTINGS = {'rssi0': -50, 'path_loss_n': 2.0, 'noise_stddev': 2.0}
        beacon_1 = BLEbeacon(node_name='beacon1', \
                                position=(0.0, 0.0, 0.0), \
                                rssi_settings=self.RSSI_SETTINGS)
        self.beacons.append(beacon_1)
        beacon_2 = BLEbeacon(node_name='beacon2', \
                                position=(5.0, 0.0, 0.0), \
                                rssi_settings=self.RSSI_SETTINGS)
        self.beacons.append(beacon_2)
        beacon_3 = BLEbeacon(node_name='beacon3', \
                                position=(0.0, 5.0, 0.0), \
                                rssi_settings=self.RSSI_SETTINGS)
        self.beacons.append(beacon_3)
        beacon_4 = BLEbeacon(node_name='beacon4', \
                                position=(5.0, 5.0, 0.0), \
                                rssi_settings=self.RSSI_SETTINGS)
        self.beacons.append(beacon_4)

        executor = MultiThreadedExecutor()
        for drone in self.drones:
            executor.add_node(drone)
        for beacon in self.beacons:
            executor.add_node(beacon)

        self.executor_thread = threading.Thread(target=self._run_executor, args=(executor,))
        self.executor_thread.start()
        
    def _run_executor(self, executor):
        executor.spin()

    def position_setpoint(self, drone_id = 0, position: tuple = (0.0, 0.0, -5.0)):
        drone = self.drones[drone_id]
        drone.set_position(position)

    def estimate_position(self, drone_id = 0):
        drone = self.drones[drone_id]
        position = drone.get_position()

        rssi_values = [beacon.get_rssi(position) for beacon in self.beacons]

        def residuals(x):
            return [rssi - (self.RSSI_SETTINGS['rssi0'] - 10 * self.RSSI_SETTINGS['path_loss_n'] * math.log10(
                math.sqrt((x[0] - beacon.position[0])**2 + (x[1] - beacon.position[1])**2 + (x[2] - beacon.position[2])**2)))
                    for rssi, beacon in zip(rssi_values, self.beacons)]

        initial_guess = [position[0], position[1], position[2]]
        result = least_squares(residuals, initial_guess)

        estimated_position = result.x
        print(f"Estimated Position for Drone {drone_id}: {estimated_position}")
        


def main(args=None) -> None:
    rclpy.init(args=args)

    controller = Controller()


    cmd_dict = {
        'set': lambda z: controller.position_setpoint(z),
        'disarm': lambda: [drone.disarm() for drone in controller.drones],
        'land': lambda: [drone.land() for drone in controller.drones],
        'est': lambda: [controller.estimate_position(drone_id) for drone_id in range(len(controller.drones))]
    }

    while rclpy.ok():
        try:
            drone_id = input("Enter drone ID (0 or 1): ").strip()
            if drone_id not in ['0', '1']:
                print("Invalid drone ID. Please enter 0 or 1.")
                continue
            drone_id = int(drone_id)
            cmd = input("Enter command (set, disarm, land, est): ").strip().lower()
            if cmd in cmd_dict:
                if cmd == 'set':
                    x = float(input("Enter x position: ").strip())
                    y = float(input("Enter y position: ").strip())
                    z = float(input("Enter z position: ").strip())
                    controller.position_setpoint(drone_id, (x, y, z))
                elif cmd == 'est':
                    controller.estimate_position(drone_id)
                else:
                    cmd_dict[cmd]()
            else:
                print("Unknown command. Please enter 'setpoint', 'disarm', or 'land'.")
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
