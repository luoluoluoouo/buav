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
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

from publisher.offboard_control_mode_publisher import OffboardControlModePublisher
from publisher.trajectory_setpoint_publisher import TrajectorySetpointPublisher
from publisher.vehicle_command_publisher import VehicleCommandPublisher, VehicleCommandConsts

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
        self.x = self.gazebo_x
        self.y = self.gazebo_y
        self.z = self.gazebo_z

        self.get_logger().warn(f'{prefix} + {target_system}')

        # Create publishers using modular approach
        self._offboard_control_mode_publisher = OffboardControlModePublisher(self, qos_profile, prefix)
        self._trajectory_setpoint_publisher = TrajectorySetpointPublisher(self, qos_profile, prefix)
        self._vehicle_command_publisher = VehicleCommandPublisher(self, qos_profile, prefix)

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
        if msg is not None:
            self.vehicle_local_position = msg
            self.x = self.vehicle_local_position.x
            self.y = self.vehicle_local_position.y
            self.z = self.vehicle_local_position.z
        else:
            self.get_logger().warn(f'No data received from {self.prefix}/fmu/out/vehicle_local_position')


    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self._vehicle_command_publisher.publish_command(
            command_id=VehicleCommandConsts.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=VehicleCommandConsts.ARMING_ACTION_ARM,
            target_system=self.target_system,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )

    def disarm(self):
        self._vehicle_command_publisher.publish_command(
            command_id=VehicleCommandConsts.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=VehicleCommandConsts.ARMING_ACTION_DISARM,
            target_system=self.target_system,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )

    def engage_offboard_mode(self):
        self._vehicle_command_publisher.publish_command(
            command_id=VehicleCommandConsts.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
            target_system=self.target_system,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )

    def land(self):
        self._vehicle_command_publisher.publish_command(
            command_id=VehicleCommandConsts.VEHICLE_CMD_NAV_LAND,
            target_system=self.target_system,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )

    def publish_offboard_control_heartbeat_signal(self):
        self._offboard_control_mode_publisher.publish_command(position=True)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        self._trajectory_setpoint_publisher.publish_command(
            position=(x, y, z),
            yaw=1.57079
        )

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
    
    def get_offboard_control_mode_publisher(self):
        return self._offboard_control_mode_publisher
    
    def get_trajectory_setpoint_publisher(self):
        return self._trajectory_setpoint_publisher
    
    def get_vehicle_command_publisher(self):
        return self._vehicle_command_publisher

class BLEbeacon(Node):
    def __init__(self, node_name: str, position: tuple, rssi_settings: dict) -> None:
        super().__init__(f'{node_name}')

        self.position = position

        # 模擬參數
        self.rssi0 = rssi_settings['rssi0'] #+ random.uniform(-5, 5)  # 基準 RSSI 值，隨機偏移
        self.path_loss_n = rssi_settings['path_loss_n'] #+ random.uniform(-0.1, 0.1)  # 路徑損失指數，隨機偏移
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
        noise = random.gauss(0, self.noise_stddev)
        rssi += noise

        return rssi

class MultiDroneController():
    def __init__(self):
        self.drones = []
        # test_drone = OffboardControl(name = 'test_drone', \
        #                             prefix= '', \
        #                             target_system=0, \
        #                             gazebo_position=(0.0, 0.0, 0.0))
        # self.drones.append(test_drone)

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
        self.RSSI_SETTINGS = {'rssi0': -50, 'path_loss_n': 2.0, 'noise_stddev': 1.0}
        beacon_1 = BLEbeacon(node_name='beacon1', \
                                position=(0.0, 0.0, 0.0), \
                                rssi_settings=self.RSSI_SETTINGS)
        self.beacons.append(beacon_1)
        beacon_2 = BLEbeacon(node_name='beacon2', \
                                position=(10.0, 0.0, 0.0), \
                                rssi_settings=self.RSSI_SETTINGS)
        self.beacons.append(beacon_2)
        beacon_3 = BLEbeacon(node_name='beacon3', \
                                position=(0.0, 10.0, 0.0), \
                                rssi_settings=self.RSSI_SETTINGS)
        self.beacons.append(beacon_3)
        beacon_4 = BLEbeacon(node_name='beacon4', \
                                position=(10.0, 10.0, 0.0), \
                                rssi_settings=self.RSSI_SETTINGS)
        self.beacons.append(beacon_4)

        # Drone1 作為移動 BLE 信標（協同定位用）
        self.drone1_beacon = BLEbeacon(node_name='drone1_beacon', \
                                      position=(0.0, 0.0, 0.0), \
                                      rssi_settings=self.RSSI_SETTINGS)

        executor = MultiThreadedExecutor()
        for drone in self.drones:
            executor.add_node(drone)
        for beacon in self.beacons:
            executor.add_node(beacon)
        executor.add_node(self.drone1_beacon)

        self.executor_thread = threading.Thread(target=self._run_executor, args=(executor,))
        self.executor_thread.start()
        
        # 保存 executor 以便優雅關閉
        self.executor = executor
        
    def _run_executor(self, executor):
        try:
            executor.spin()
        except Exception as e:
            print(f"Executor error: {e}")
    
    def shutdown(self):
        """優雅關閉所有節點和執行緒"""
        try:
            # 先讓所有無人機降落並解除武裝
            for drone in self.drones:
                drone.land()
                time.sleep(0.5)
                drone.disarm()
            
            # 停止 executor
            if hasattr(self, 'executor'):
                self.executor.shutdown()
            
            # 等待執行緒結束
            if hasattr(self, 'executor_thread') and self.executor_thread.is_alive():
                self.executor_thread.join(timeout=5.0)
                
        except Exception as e:
            print(f"Shutdown error: {e}")

    def position_setpoint(self, drone_id = 0, position: tuple = (0.0, 0.0, -5.0)):
        drone = self.drones[drone_id]
        drone.set_position(position)

    def update_drone1_beacon_position(self):
        """動態更新 drone1 作為移動信標的位置"""
        if len(self.drones) > 0:
            drone1_position = self.drones[0].get_position()
            self.drone1_beacon.position = tuple(drone1_position)
            return drone1_position
        return None

    def estimate_position(self, drone_id = 0):
        drone = self.drones[drone_id]

        position = drone.get_position()
        print(f"Current Position for Drone {drone_id}: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

        rssi_values = [beacon.get_rssi(position) for beacon in self.beacons]

        def residuals(x):
            return [rssi - (self.RSSI_SETTINGS['rssi0'] - 10 * self.RSSI_SETTINGS['path_loss_n'] * math.log10(
                math.sqrt((x[0] - beacon.position[0])**2 + (x[1] - beacon.position[1])**2 + (x[2] - beacon.position[2])**2))
            ) for rssi, beacon in zip(rssi_values, self.beacons)]

        initial_guess = [position[0], position[1], position[2]]
        result = least_squares(residuals, initial_guess)

        estimated_position = result.x
        print(f"Estimated Position for Drone {drone_id}: x={estimated_position[0]:.2f}, y={estimated_position[1]:.2f}, z={estimated_position[2]:.2f}")
        
        # 計算估計誤差
        error_x = abs(estimated_position[0] - position[0])
        error_y = abs(estimated_position[1] - position[1]) 
        error_z = abs(estimated_position[2] - position[2])
        error_total = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        print(f"Position Error for Drone {drone_id}: x={error_x:.2f}m, y={error_y:.2f}m, z={error_z:.2f}m, total={error_total:.2f}m")
        
        return estimated_position

    def estimate_positions_coop(self):
        """協同定位：先估測 drone1，再以 drone1 作為移動信標估測 drone2"""
        print("=== 開始協同定位 ===")
        
        # 步驟1：使用靜態信標估測 drone1 位置
        print("\n步驟1：估測 Drone1 位置（使用靜態 BLE 信標）")
        drone1_estimated = self.estimate_position(0)
        
        # 步驟2：更新 drone1_beacon 的位置（讓 drone1 成為移動信標）
        self.drone1_beacon.position = tuple(drone1_estimated)
        print(f"更新 Drone1 作為移動信標位置: {drone1_estimated}")
        
        # 步驟3：使用靜態信標 + drone1 信標來估測 drone2 位置
        print("\n步驟2：估測 Drone2 位置（使用靜態信標 + Drone1 移動信標）")
        drone2 = self.drones[1]
        drone2_actual_position = drone2.get_position()
        print(f"Drone2 實際位置: x={drone2_actual_position[0]:.2f}, y={drone2_actual_position[1]:.2f}, z={drone2_actual_position[2]:.2f}")
        
        # 獲取所有信標（靜態 + drone1）的 RSSI 值
        all_beacons = self.beacons + [self.drone1_beacon]
        rssi_values = [beacon.get_rssi(drone2_actual_position) for beacon in all_beacons]
        
        def residuals_coop(x):
            return [rssi - (self.RSSI_SETTINGS['rssi0'] - 10 * self.RSSI_SETTINGS['path_loss_n'] * math.log10(
                max(0.1, math.sqrt((x[0] - beacon.position[0])**2 + (x[1] - beacon.position[1])**2 + (x[2] - beacon.position[2])**2)))
            ) for rssi, beacon in zip(rssi_values, all_beacons)]
        
        initial_guess = [drone2_actual_position[0], drone2_actual_position[1], drone2_actual_position[2]]
        result = least_squares(residuals_coop, initial_guess)
        
        drone2_estimated = result.x
        print(f"Drone2 估測位置（協同定位）: x={drone2_estimated[0]:.2f}, y={drone2_estimated[1]:.2f}, z={drone2_estimated[2]:.2f}")
        
        # 計算協同定位的精度改善
        # 先計算只用靜態信標時的估測誤差
        rssi_values_static = [beacon.get_rssi(drone2_actual_position) for beacon in self.beacons]
        
        def residuals_static(x):
            return [rssi - (self.RSSI_SETTINGS['rssi0'] - 10 * self.RSSI_SETTINGS['path_loss_n'] * math.log10(
                max(0.1, math.sqrt((x[0] - beacon.position[0])**2 + (x[1] - beacon.position[1])**2 + (x[2] - beacon.position[2])**2)))
            ) for rssi, beacon in zip(rssi_values_static, self.beacons)]
        
        result_static = least_squares(residuals_static, initial_guess)
        drone2_static_estimated = result_static.x
        print(f"Drone2 估測位置（僅靜態信標）: x={drone2_static_estimated[0]:.2f}, y={drone2_static_estimated[1]:.2f}, z={drone2_static_estimated[2]:.2f}")
        
        # 計算兩種方法的誤差
        error_static = math.sqrt(sum([(drone2_static_estimated[i] - drone2_actual_position[i])**2 for i in range(3)]))
        error_coop = math.sqrt(sum([(drone2_estimated[i] - drone2_actual_position[i])**2 for i in range(3)]))
        
        print(f"\n=== 定位精度比較 ===")
        print(f"僅靜態信標誤差: {error_static:.2f}m")
        print(f"協同定位誤差: {error_coop:.2f}m")
        print(f"精度改善: {((error_static - error_coop)/error_static*100):.1f}%")
        
        return {
            'drone1_estimated': drone1_estimated,
            'drone2_actual': drone2_actual_position,
            'drone2_static_estimated': drone2_static_estimated,
            'drone2_coop_estimated': drone2_estimated,
            'error_static': error_static,
            'error_coop': error_coop
        }
    
def main(args=None) -> None:
    rclpy.init(args=args)

    controller = MultiDroneController()

    cmd_dict = {
        'arm': lambda: [drone.arm() for drone in controller.drones],
        'set': lambda z: controller.position_setpoint(z),
        'disarm': lambda: [drone.disarm() for drone in controller.drones],
        'land': lambda: [drone.land() for drone in controller.drones],
        'est': lambda: [controller.estimate_position(drone_id) for drone_id in range(len(controller.drones))],
        'coop': lambda: controller.estimate_positions_coop()
    }

    try:
        print("=== Multi-Drone BLE Localization Controller ===")
        print("Available commands:")
        print("  arm  - Arm all drones")
        print("  set  - Set position for a specific drone")
        print("  est  - Estimate position using BLE beacons")
        print("  coop - Cooperative localization (Drone1 as moving beacon)")
        print("  land - Land all drones")
        print("  disarm - Disarm all drones")
        print("  quit - Exit program")
        print()

        while rclpy.ok():
            try:
                cmd = input("Enter command: ").strip().lower()
                if cmd in ['quit', 'exit', 'q']:
                    break
                elif cmd in cmd_dict:
                    if cmd == 'set':
                        drone_id = input("Enter drone ID (0 or 1): ").strip()
                        if drone_id not in ['0', '1']:
                            print("Invalid drone ID. Please enter 0 or 1.")
                            continue
                        drone_id = int(drone_id)
                        x = float(input("Enter x position: ").strip())
                        y = float(input("Enter y position: ").strip())
                        z = float(input("Enter z position: ").strip())
                        controller.position_setpoint(drone_id, (x, y, z))
                    elif cmd == 'est':
                        drone_id = input("Enter drone ID (0 or 1, or 'all' for both): ").strip()
                        if drone_id == 'all':
                            for i in range(len(controller.drones)):
                                controller.estimate_position(i)
                        elif drone_id in ['0', '1']:
                            controller.estimate_position(int(drone_id))
                        else:
                            print("Invalid drone ID. Please enter 0, 1, or 'all'.")
                    elif cmd == 'coop':
                        controller.estimate_positions_coop()
                    else:
                        cmd_dict[cmd]()
                else:
                    print("Unknown command. Available commands: arm, set, disarm, land, est, coop, quit")
            except KeyboardInterrupt:
                break
            except ValueError as e:
                print(f"Input error: {e}")
            except Exception as e:
                print(f"Error executing command: {e}")
    
    finally:
        print("Shutting down...")
        controller.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)