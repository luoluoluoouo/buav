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
    def __init__(self, is_gazebo = False, drones = list[OffboardControl] = None):
        self.drones = drones if drones is not None else []
        self.is_gazebo = is_gazebo

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
    drone_id = int(input("Enter drone ID: ").strip())
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
    drone_id = int(input("Enter drone ID: ").strip())
    increment_x = float(input("Enter x position increment (m): ").strip())
    increment_y = float(input("Enter y position increment (m): ").strip())
    increment_z = float(input("Enter z position increment (m): ").strip())
    increment_yaw = math.radians(float(input("Enter target yaw increment (degrees): ").strip()))
    inc_pos = np.array([increment_x, increment_y, increment_z, increment_yaw])

    controller.set_incremental_position_setpoint(drone_id, inc_pos)

def main(args=None) -> None:
    rclpy.init(args=args)

    controller = MultiDroneController()

    cmd_dict = {
        'arm': cmd_arm,
        'abs': cmd_abs,
        'inc': cmd_inc,
        'disarm': cmd_disarm,
        'land': cmd_land,
    }
    
    print("=== Multi-Drone BLE Localization Controller ===")
    print("Available commands:")
    print("  arm  - Arm all drones")
    print("  abs  - Set absolute position for a specific drone")
    print("  inc  - Set incremental position for a specific drone")
    print("  land - Land all drones")
    print("  disarm - Disarm all drones")
    print("  quit - Exit program")
    print()

    while rclpy.ok():
        try:
            cmd = input("Enter command: ").strip().lower()
            
            if cmd in ['quit', 'exit', 'q']:
                break
            
            if cmd not in cmd_dict:
                print("Unknown command. Available commands: arm, abs, inc, disarm, land, quit")
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

def sim(args=None) -> None:
    rclpy.init(args=args)

    drone_1 = OffboardControl(
        qos_profile = qos_profile,
        name = 'drone_1',
        prefix = '/px4_1',
        target_system = 2,
        gazebo_pos = np.array([0.0, 0.0, 0.0, 0.0]),  # x, y, z, yaw
        is_gazebo = False
    )
    drone_2 = OffboardControl(
        qos_profile = qos_profile,
        name = 'drone_2',
        prefix = '/px4_2',
        target_system = 3,
        gazebo_pos = np.array([0.0, 3.0, 0.0, 0.0]),  # x, y, z, yaw
        is_gazebo = False
    )
    controller = MultiDroneController(is_gazebo=True, drones=[drone_1, drone_2])

    cmd_dict = {
        'arm': cmd_arm,
        'abs': cmd_abs,
        'inc': cmd_inc,
        'disarm': cmd_disarm,
        'land': cmd_land,
    }
    
    print("=== Multi-Drone BLE Localization Controller ===")
    print("Available commands:")
    print("  arm  - Arm all drones")
    print("  abs  - Set absolute position for a specific drone")
    print("  inc  - Set incremental position for a specific drone")
    print("  land - Land all drones")
    print("  disarm - Disarm all drones")
    print("  quit - Exit program")
    print()

    while rclpy.ok():
        try:
            cmd = input("Enter command: ").strip().lower()
            
            if cmd in ['quit', 'exit', 'q']:
                break
            
            if cmd not in cmd_dict:
                print("Unknown command. Available commands: arm, abs, inc, disarm, land, quit")
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
        # sim()
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