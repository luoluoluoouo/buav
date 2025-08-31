#!/usr/bin/env python3
import math
import numpy as np

from threading import Thread

from scipy.optimize import least_squares

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from .node_models.offboard_control import OffboardControl
from .node_models.ble_beacon import BLEbeacon

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class MultiDroneController():
    def __init__(self):    
        drone = OffboardControl(
            qos_profile = qos_profile,
            name = 'drone',
            prefix = '',
            target_system = 1,
            gazebo_pos = np.array([0.0, 0.0, 0.0, 0.0]),  # x, y, z, yaw
            is_gazebo = False
        )
        self.drones = [ drone ]

        self.RSSI_SETTINGS = {'rssi0': -50.0, 'path_loss_n': 2.0, 'noise_stddev': 1.0}
        
        beacon_1 = BLEbeacon(
            node_name='beacon1',
            position=(0.0, 0.0, 0.0),
            rssi_settings=self.RSSI_SETTINGS)
        
        beacon_2 = BLEbeacon(
            node_name='beacon2',
            position=(10.0, 0.0, 0.0),
            rssi_settings=self.RSSI_SETTINGS)
        
        beacon_3 = BLEbeacon(
            node_name='beacon3',
            position=(0.0, 10.0, 0.0), \
            rssi_settings=self.RSSI_SETTINGS)
        
        beacon_4 = BLEbeacon(node_name='beacon4',
            position=(10.0, 10.0, 0.0),
            rssi_settings=self.RSSI_SETTINGS)

        self.beacons = [ beacon_1, beacon_2, beacon_3, beacon_4 ]

        # Drone1 作為移動 BLE 信標（協同定位用）
        self.drone1_beacon = BLEbeacon(
            node_name='drone1_beacon',
            position=(0.0, 0.0, 0.0),
            rssi_settings=self.RSSI_SETTINGS)

        self.executor = MultiThreadedExecutor()
        for drone in self.drones:
            self.executor.add_node(drone)
        for beacon in self.beacons:
            self.executor.add_node(beacon)
        self.executor.add_node(self.drone1_beacon)

        self.executor_thread = Thread(target=self._run_executor, args=(self.executor,))
        self.executor_thread.start()

    def set_absolute_position_setpoint(self, drone_id = 0, pos: np.ndarray = None): 
        self.drones[drone_id].set_absolute_position(pos)

    def set_incremental_position_setpoint(self, drone_id = 0, inc_pos: np.ndarray = None):
        self.drones[drone_id].set_incremental_position(inc_pos)

    def update_drone1_beacon_position(self) -> None:
        """動態更新 drone1 作為移動信標的位置"""
        try:
            self.drone1_beacon.position = self.drones[0].get_position()
        except Exception:
            pass

    def _run_executor(self, executor: MultiThreadedExecutor):
        try:
            executor.spin()
        except Exception as e:
            print(f"Executor error: {e}")

def cmd_arm(controller: MultiDroneController) -> None:
    """Arm all drones"""
    for drone in controller.drones:
        if not drone.is_armed():
            drone.arm()

def cmd_disarm(controller: MultiDroneController) -> None:
    """Disarm all drones"""
    for drone in controller.drones:
        if drone.is_armed():
            drone.disarm()

def cmd_land(controller: MultiDroneController) -> None:
    """Land all drones"""
    for drone in controller.drones:
        drone.land()

def cmd_abs(controller: MultiDroneController) -> None:
    """
    Set absolute position for a specific drone
    In NEU (North, East, Up) coordinates
    """
    drone_id = 0
    target_x = float(input("Enter x position (m): ").strip())
    target_y = float(input("Enter y position (m): ").strip())
    target_z = float(input("Enter z position (m): ").strip())
    target_yaw = math.radians(float(input("Enter target yaw (degrees): ").strip()))
    abs_pos = (target_x, target_y, target_z, target_yaw)

    controller.set_absolute_position_setpoint(drone_id, abs_pos)

def cmd_inc(controller: MultiDroneController) -> None:
    """
    Set incremental position for a specific drone
    In NEU (North, East, Up) coordinates
    """
    drone_id = 0
    increment_x = float(input("Enter x position increment (m): ").strip())
    increment_y = float(input("Enter y position increment (m): ").strip())
    increment_z = float(input("Enter z position increment (m): ").strip())
    increment_yaw = math.radians(float(input("Enter target yaw increment (degrees): ").strip()))
    inc_pos = np.array([increment_x, increment_y, increment_z, increment_yaw])

    controller.set_incremental_position_setpoint(drone_id, inc_pos)

def cmd_offboard(controller: MultiDroneController) -> None:
    """Force enable offboard mode"""
    for drone in controller.drones:
        drone.force_offboard_mode()
    print("Offboard mode forced for all drones")

def cmd_status(controller: MultiDroneController) -> None:
    """Check current control status"""
    for i, drone in enumerate(controller.drones):
        status = drone.get_control_status()
        print(f"\n=== Drone {i} Status ===")
        print(f"Navigation State: {status['nav_state']} ({'Offboard' if status['offboard_mode'] else 'Manual' if status['manual_mode'] else 'Other'})")
        print(f"Armed: {status['is_armed']}")
        print(f"Flying: {status['is_flying']}")
        print(f"Safe for Offboard: {drone.is_safe_for_offboard()}")
        
        if status['manual_mode']:
            print("⚠️  WARNING: Vehicle is in MANUAL mode - offboard commands will be rejected!")
        elif status['offboard_mode']:
            print("✅ Vehicle is in OFFBOARD mode - ready for commands")

def main(args=None) -> None:
    rclpy.init(args=args)

    controller = MultiDroneController()

    cmd_dict = {
        'arm': cmd_arm,
        'abs': cmd_abs,
        'inc': cmd_inc,
        'offboard': cmd_offboard,
        'status': cmd_status,
        'disarm': cmd_disarm,
        'land': cmd_land,
    }
    
    print("=== Multi-Drone BLE Localization Controller ===")
    print("Available commands:")
    print("  arm     - Arm all drones")
    print("  abs     - Set absolute position for a specific drone")
    print("  inc     - Set incremental position for a specific drone")
    print("  offboard- Force enable offboard mode")
    print("  status  - Check current control status")
    print("  land    - Land all drones")
    print("  disarm  - Disarm all drones")
    print("  quit    - Exit program")
    print()
    print("⚠️  SAFETY: Before using 'abs' or 'inc', ensure RC is not in manual mode!")
    print("   Use 'status' command to check current mode.")
    print()

    while rclpy.ok():
        try:
            cmd = input("Enter command: ").strip().lower()
            
            if cmd in ['quit', 'exit', 'q']:
                break
            
            if cmd not in cmd_dict:
                print("Unknown command. Available commands: arm, abs, inc, offboard, status, disarm, land, quit")
                continue
            
            cmd_dict[cmd](controller)
        except ValueError as e:
            print(f"Input error: {e}")
        except Exception as e:
            print(f"Error executing command: {e}")
        except:
            break
    
    print("Shutting down...")
    shutdown_everything(controller)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

def shutdown_everything(controller: MultiDroneController) -> None:
    try:
        controller.executor.shutdown()
    except:
        pass
    for drone in controller.drones:
        try:
            drone.destroy_node()
        except:
            pass
    for beacon in controller.beacons:
        try:
            beacon.destroy_node()
        except:
            pass
    try:
        controller.drone1_beacon.destroy_node()
    except:
        pass
    try:
        rclpy.shutdown()
    except:
        pass