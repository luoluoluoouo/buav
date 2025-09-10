#!/usr/bin/env python3
import math
from more_itertools import one
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
    def __init__(self, is_gazebo: bool = False, drones: list[OffboardControl] = None):
        self.drones = drones if drones is not None else []
        self.is_gazebo = is_gazebo

        self.executor = MultiThreadedExecutor()
        for drone in self.drones:
            self.executor.add_node(drone)

        self.executor_thread = Thread(target=self._run_executor, args=(self.executor,))
        self.executor_thread.start()

    def _run_executor(self, executor: MultiThreadedExecutor):
        try:
            executor.spin()
        except Exception as e:
            print(f"Executor error: {e}")

    def _get_drone_id(self) -> int:
        if len(self.drones) == 1:
            return 0
        else:
            drone_id = int(input(f"Enter drone ID (0 to {len(self.drones)-1}): ").strip())
            return drone_id

    def cmd_arm(self) -> None:
        """Arm all drones"""
        for drone in self.drones:
            if not drone.is_armed():
                drone.arm()

    def cmd_disarm(self) -> None:
        """Disarm all drones"""
        for drone in self.drones:
            if drone.is_armed():
                drone.disarm()

    def cmd_land(self) -> None:
        """Land all drones"""
        for drone in self.drones:
            drone.land()

    def cmd_abs(self) -> None:
        """
        Set absolute position for a specific drone
        In NEU (North, East, Up) coordinates
        """
        drone_id = self._get_drone_id()
        target_x = float(input("Enter x position (m): ").strip())
        target_y = float(input("Enter y position (m): ").strip())
        target_z = float(input("Enter z position (m): ").strip())
        target_yaw = math.radians(float(input("Enter target yaw (degrees): ").strip()))
        abs_pos = np.array([target_x, target_y, target_z, target_yaw])

        self.drones[drone_id].set_absolute_position(abs_pos)

    def cmd_inc(self) -> None:
        """
        Set incremental position for a specific drone
        In NEU (North, East, Up) coordinates
        """
        drone_id = self._get_drone_id()
        increment_x = float(input("Enter x position increment (m): ").strip())
        increment_y = float(input("Enter y position increment (m): ").strip())
        increment_z = float(input("Enter z position increment (m): ").strip())
        increment_yaw = math.radians(float(input("Enter target yaw increment (degrees): ").strip()))
        inc_pos = np.array([increment_x, increment_y, increment_z, increment_yaw])

        self.drones[drone_id].set_incremental_position(inc_pos)

    def cmd_get_position(self) -> None:
        drone_id = self._get_drone_id()
        pos = self.drones[drone_id].get_position()
        print(f"Drone {drone_id} position: x={pos[0]}, y={pos[1]}, z={pos[2]}")

    def cmd_circular_scan(self) -> None:
        drone_id = self._get_drone_id()

        center_x = 0
        center_y = 0
        center_z = 2.5
        radius = 2
        yaw = math.pi/2
        _step = math.radians(60)
        _theta = math.radians(0)

        while _theta < 2*math.pi:
            scan_pos = np.array([
                        center_x + radius * np.cos(_theta), \
                        center_y + radius * np.sin(_theta), \
                        center_z, \
                        _theta + yaw])
            self.drones[drone_id].set_absolute_position(drone_id, scan_pos)
            print(f"Set drone {drone_id} position to: {scan_pos}")
            input("Press Enter to continue to next point...")
            _theta += _step

        _theta = math.radians(0)
        scan_pos = np.array([
                    center_x + radius * np.cos(_theta), \
                    center_y + radius * np.sin(_theta), \
                    center_z, \
                    _theta + yaw])
        self.drones[drone_id].set_absolute_position(drone_id, scan_pos)

def real_drone(args=None):
    rclpy.init(args=args)

    drone_0 = OffboardControl(
        qos_profile = qos_profile,
        name = 'drone_0',
        prefix = '',
        target_system = 1,
        gazebo_enu_pos = np.array([0.0, 0.0, 0.0, 0.0]),  # x, y, z, yaw
        is_gazebo = False
    )
    controller = MultiDroneController(is_gazebo=True, drones=[drone_0])

    main(controller)

def sim_one_drone(args=None):
    rclpy.init(args=args)

    drone_0 = OffboardControl(
        qos_profile = qos_profile,
        name = 'drone_1',
        prefix = '/px4_1',
        target_system = 2,
        gazebo_enu_pos = np.array([3.0, 0.0, 0.0, 0.0]),  # x, y, z, yaw
        is_gazebo = True
    )
    controller = MultiDroneController(is_gazebo=True, drones=[drone_0])

    main(controller)

def sim_two_drones(args=None):
    rclpy.init(args=args)

    drone_1 = OffboardControl(
        qos_profile = qos_profile,
        name = 'drone_1',
        prefix = '/px4_1',
        target_system = 2,
        gazebo_enu_pos = np.array([0.0, 3.0, 0.0, 0.0]),  # x, y, z, yaw
        is_gazebo = True
    )
    drone_2 = OffboardControl(
        qos_profile = qos_profile,
        name = 'drone_2',
        prefix = '/px4_2',
        target_system = 3,
        gazebo_enu_pos = np.array([0.0, -3.0, 0.0, 0.0]),  # x, y, z, yaw
        is_gazebo = True
    )
    controller = MultiDroneController(is_gazebo=True, drones=[drone_1, drone_2])

    main(controller)

def main(controller: MultiDroneController) -> None:
    cmd_dict = {
        'arm': controller.cmd_arm,
        'abs': controller.cmd_abs,
        'inc': controller.cmd_inc,
        'disarm': controller.cmd_disarm,
        'land': controller.cmd_land,
        'pos': controller.cmd_get_position,
        'scan': controller.cmd_circular_scan
    }
    
    print("=== Multi-Drone BLE Localization Controller ===")
    print("Available commands:")
    print("  arm  - Arm all drones")
    print("  abs  - Set absolute position for a specific drone")
    print("  inc  - Set incremental position for a specific drone")
    print("  land - Land all drones")
    print("  pos  - Get current position of a specific drone")
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

            cmd_dict[cmd]()
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
        rclpy.init(args=None)

        drone_0 = OffboardControl(
            qos_profile = qos_profile,
            name = 'drone_1',
            prefix = '/px4_1',
            target_system = 2,
            gazebo_pos = np.array([3.0, 0.0, 0.0, 0.0]),  # x, y, z, yaw
            is_gazebo = True
        )
        controller = MultiDroneController(is_gazebo=True, drones=[drone_0])

        main(controller)
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
    # for beacon in controller.beacons:
    #     try:
    #         beacon.destroy_node()
    #     except:
    #         pass
    try:
        controller.drone1_beacon.destroy_node()
    except:
        pass
    try:
        rclpy.shutdown()
    except:
        pass