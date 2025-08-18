#!/usr/bin/env python3
import math

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
        drone_1 = OffboardControl(
            qos_profile = qos_profile,
            name = 'drone_1',
            prefix = '/px4_1',
            target_system = 2,
            gazebo_position = (0.0, 3.0, 0.0))
        
        drone_2 = OffboardControl(
            qos_profile=qos_profile,
            name = 'drone_2',
            prefix= '/px4_2',
            target_system = 3,
            gazebo_position=(0.0, 6.0, 0.0))

        self.drones = [ drone_1, drone_2 ]
        
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

    def position_setpoint(self, drone_id = 0, position: tuple = (0.0, 0.0, -5.0)):
        drone = self.drones[drone_id]
        drone.set_position(position)

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
        drone.arm()

def cmd_disarm(controller: MultiDroneController) -> None:
    """Disarm all drones"""
    for drone in controller.drones:
        drone.disarm()

def cmd_land(controller: MultiDroneController) -> None:
    """Land all drones"""
    for drone in controller.drones:
        drone.land()

def cmd_set(controller: MultiDroneController) -> None:
    drone_id = int(input("Enter drone ID (0 or 1): ").strip())
    x = float(input("Enter x position: ").strip())
    y = float(input("Enter y position: ").strip())
    z = float(input("Enter z position: ").strip())
    
    controller.position_setpoint(drone_id, (x, y, z))


def main(args=None) -> None:
    rclpy.init(args=args)

    controller = MultiDroneController()

    cmd_dict = {
        'arm': cmd_arm,
        'set': cmd_set,
        'disarm': cmd_disarm,
        'land': cmd_land,
    }
    
    print("=== Multi-Drone BLE Localization Controller ===")
    print("Available commands:")
    print("  arm  - Arm all drones")
    print("  set  - Set position for a specific drone")
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
                print("Unknown command. Available commands: arm, set, disarm, land, est, coop, quit")
                continue
            
            cmd_dict[cmd](controller)
        except ValueError as e:
            print(f"Input error: {e}")
        except Exception as e:
            print(f"Error executing command: {e}")
        except:
            break
    
    print("Shutting down...")
    
    for drone in controller.drones:
        drone.land()
        drone.disarm()
    
    controller.executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)