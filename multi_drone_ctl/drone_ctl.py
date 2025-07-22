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

    def __init__(self, prefix: str, target_system: int) -> None:
        super().__init__(f'offboard_control_drone_{target_system}')

        self.prefix = prefix
        self.target_system = target_system
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
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

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

class BLEbeacon(Node):
    def __init__(self, position = (0.0, 0.0, 0.0), \
                    prefix: str = '', \
                    target_system: int = 1, 
                    name: str = '',
                    rssi_settings: dict = None) -> None:
        super().__init__(f'ble_beacon_{name}_drone_{target_system}')

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            f'{prefix}/fmu/out/vehicle_local_position',
            self.listener_callback,
            qos_profile
        )

        self.rssi_publisher = self.create_publisher(Float32, f'/ble{prefix}/beacon_{name}/rssi', qos_profile)

        self.position = position

        # 模擬參數
        self.rssi0 = rssi_settings['rssi0']  # 在 1 米處的 RSSI
        self.path_loss_n = rssi_settings['path_loss_n']  # 路徑損失指數
        self.noise_stddev = 2.0  # dBm 隨機雜訊

        self.rssi = 0.0

    def listener_callback(self, msg: VehicleLocalPosition):
        drone_x = msg.x
        drone_y = msg.y
        drone_z = msg.z

        # 計算與 beacon 的水平距離
        dx = drone_x - self.position[0]
        dy = drone_y - self.position[1]
        dz = drone_z - self.position[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        distance = max(distance, 0.1)  # 避免 log(0)

        # 使用 log-distance 模型模擬 RSSI
        rssi = self.rssi0 - 10 * self.path_loss_n * math.log10(distance)

        # 加上隨機 noise
        noise = random.gauss(0, self.noise_stddev)
        rssi += noise

        # 發布
        msg_out = Float32()
        msg_out.data = float(rssi)
        self.rssi_publisher.publish(msg_out)

        self.rssi = rssi

class Controller():
    def __init__(self, drones, beacons):
        self.drones = drones
        self.beacons = beacons

        executor = MultiThreadedExecutor()
        for drone in drones:
            executor.add_node(drone)

        for beacon in beacons:
            executor.add_node(beacon)

        self.executor_thread = threading.Thread(target=self._run_executor, args=(executor,))
        self.executor_thread.start()
        
    def _run_executor(self, executor):
        executor.spin()

    def _run_position_setpoint(self, drone_ID = 0,x: float = 0.0, y: float = 0.0, z: float = -5.0):
        drone = self.drones[drone_ID]
        drone.publish_position_setpoint(x, y, z)

    def position_setpoint(self, drone_ID=0, x: float = 0.0, y: float = 0.0, z: float = -5.0):
        self.position_setpoint_thread = threading.Thread(
            target=self._run_position_setpoint, args=(drone_ID, x, y, z))
        self.position_setpoint_thread.start()

    def estimate_position(self):
        for i in range(4):
            print(self.beacons[i].rssi)

        rssi0 = self.beacons[0].rssi0
        path_loss_n = self.beacons[0].path_loss_n

        positions = []
        distances = []

        for beacon in self.beacons[:4]:
            if beacon.rssi == 0.0:  # 初始值尚未更新
                continue
            pos = beacon.position
            rssi = beacon.rssi
            distance = 10 ** ((rssi0 - rssi) / (10 * path_loss_n))
            positions.append(pos)
            distances.append(distance)

        if len(positions) < 3:
            print("[WARN] Not enough beacons for trilateration")
            return None

        def residuals(xy):
            x, y = xy
            return [np.linalg.norm([x - px, y - py]) - d for (px, py, _), d in zip(positions, distances)]

        result = least_squares(residuals, x0=[0.0, 0.0])
        print(f"Estimated position: {result.x}")
        # return result.x  # [x_est, y_est]


        

def main(args=None) -> None:
    rclpy.init(args=args)

    with open('/home/ada/luoluo/px4_ros2_ws/src/multi_drone_ctl/multi_drone_ctl/config.yaml', 'r') as f:
        config = yaml.safe_load(f)

    drones = []
    ble_beacons = []
    for drone in config['drones']:
        drone_setting = config['drones'][drone]
        drone = OffboardControl(prefix=drone_setting['prefix'], \
                                target_system=drone_setting['target_system'])
        drones.append(drone)

        for beacon in config['ble_beacons']:
            beacon_setting = config['ble_beacons'][beacon]
            blebeacon = BLEbeacon(position=beacon_setting['position'], \
                                  prefix=drone_setting['prefix'], \
                                  target_system=drone_setting['target_system'], \
                                  name=beacon_setting['name'], \
                                  rssi_settings=config['rssi_settings'])
            ble_beacons.append(blebeacon)

    controller = Controller(drones, ble_beacons)


    cmd_dict = {
        'set': lambda z: controller.position_setpoint(z),
        'disarm': lambda: [drone.disarm() for drone in drones],
        'land': lambda: [drone.land() for drone in drones],
        'estimate': controller.estimate_position
    }

    while rclpy.ok():
        try:
            cmd = input("Enter command (set, disarm, land): ").strip().lower()
            if cmd in cmd_dict:
                if cmd == 'set':
                    drone_id = int(input("Enter drone ID (0 or 1): ").strip())
                    x = float(input("Enter x position: ").strip())
                    y = float(input("Enter y position: ").strip())
                    z = float(input("Enter z position: ").strip())
                    controller.position_setpoint(drone_id, x, y, z)
                    # pass
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
