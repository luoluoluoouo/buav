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
        """設定 drone 的位置目標點"""
        self.drones[drone_id].set_position(*position)

    def update_drone1_beacon_position(self) -> None:
        """動態更新 drone1 作為移動信標的位置"""
        try:
            self.drone1_beacon.position = self.drones[0].get_position()
        except Exception:
            pass

    # TODO: CLEAN CODE, THIS IS DIRTY CODE
    # def estimate_position(self, drone_id = 0):
    #     """使用靜態 BLE 信標估測 drone 的位置"""
    #     drone = self.drones[drone_id]

    #     position = drone.get_position()
    #     print(f"Current Position for Drone {drone_id}: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

    #     rssi_values = [beacon.get_rssi(position) for beacon in self.beacons]

    #     def residuals(x):
    #         return [rssi - (self.RSSI_SETTINGS['rssi0'] - 10 * self.RSSI_SETTINGS['path_loss_n'] * math.log10(
    #             math.sqrt((x[0] - beacon.position[0])**2 + (x[1] - beacon.position[1])**2 + (x[2] - beacon.position[2])**2))
    #         ) for rssi, beacon in zip(rssi_values, self.beacons)]

    #     initial_guess = [position[0], position[1], position[2]]
    #     result = least_squares(residuals, initial_guess)

    #     estimated_position = result.x
    #     print(f"Estimated Position for Drone {drone_id}: x={estimated_position[0]:.2f}, y={estimated_position[1]:.2f}, z={estimated_position[2]:.2f}")
        
    #     # 計算估計誤差
    #     error_x = abs(estimated_position[0] - position[0])
    #     error_y = abs(estimated_position[1] - position[1]) 
    #     error_z = abs(estimated_position[2] - position[2])
    #     error_total = math.sqrt(error_x**2 + error_y**2 + error_z**2)
    #     print(f"Position Error for Drone {drone_id}: x={error_x:.2f}m, y={error_y:.2f}m, z={error_z:.2f}m, total={error_total:.2f}m")
        
    #     return estimated_position

    # def estimate_positions_coop(self):
    #     """協同定位：先估測 drone1，再以 drone1 作為移動信標估測 drone2"""
    #     print("=== 開始協同定位 ===")
        
    #     # 步驟1：使用靜態信標估測 drone1 位置
    #     print("\n步驟1：估測 Drone1 位置（使用靜態 BLE 信標）")
    #     drone1_estimated = self.estimate_position(0)
        
    #     # 步驟2：更新 drone1_beacon 的位置（讓 drone1 成為移動信標）
    #     self.drone1_beacon.position = tuple(drone1_estimated)
    #     print(f"更新 Drone1 作為移動信標位置: {drone1_estimated}")
        
    #     # 步驟3：使用靜態信標 + drone1 信標來估測 drone2 位置
    #     print("\n步驟2：估測 Drone2 位置（使用靜態信標 + Drone1 移動信標）")
    #     drone2 = self.drones[1]
    #     drone2_actual_position = drone2.get_position()
    #     print(f"Drone2 實際位置: x={drone2_actual_position[0]:.2f}, y={drone2_actual_position[1]:.2f}, z={drone2_actual_position[2]:.2f}")
        
    #     # 獲取所有信標（靜態 + drone1）的 RSSI 值
    #     all_beacons = self.beacons + [self.drone1_beacon]
    #     rssi_values = [beacon.get_rssi(drone2_actual_position) for beacon in all_beacons]
        
    #     def residuals_coop(x):
    #         return [rssi - (self.RSSI_SETTINGS['rssi0'] - 10 * self.RSSI_SETTINGS['path_loss_n'] * math.log10(
    #             max(0.1, math.sqrt((x[0] - beacon.position[0])**2 + (x[1] - beacon.position[1])**2 + (x[2] - beacon.position[2])**2)))
    #         ) for rssi, beacon in zip(rssi_values, all_beacons)]
        
    #     initial_guess = [drone2_actual_position[0], drone2_actual_position[1], drone2_actual_position[2]]
    #     result = least_squares(residuals_coop, initial_guess)
        
    #     drone2_estimated = result.x
    #     print(f"Drone2 估測位置（協同定位）: x={drone2_estimated[0]:.2f}, y={drone2_estimated[1]:.2f}, z={drone2_estimated[2]:.2f}")
        
    #     # 計算協同定位的精度改善
    #     # 先計算只用靜態信標時的估測誤差
    #     rssi_values_static = [beacon.get_rssi(drone2_actual_position) for beacon in self.beacons]
        
    #     def residuals_static(x):
    #         return [rssi - (self.RSSI_SETTINGS['rssi0'] - 10 * self.RSSI_SETTINGS['path_loss_n'] * math.log10(
    #             max(0.1, math.sqrt((x[0] - beacon.position[0])**2 + (x[1] - beacon.position[1])**2 + (x[2] - beacon.position[2])**2)))
    #         ) for rssi, beacon in zip(rssi_values_static, self.beacons)]
        
    #     result_static = least_squares(residuals_static, initial_guess)
    #     drone2_static_estimated = result_static.x
    #     print(f"Drone2 估測位置（僅靜態信標）: x={drone2_static_estimated[0]:.2f}, y={drone2_static_estimated[1]:.2f}, z={drone2_static_estimated[2]:.2f}")
        
    #     # 計算兩種方法的誤差
    #     error_static = math.sqrt(sum([(drone2_static_estimated[i] - drone2_actual_position[i])**2 for i in range(3)]))
    #     error_coop = math.sqrt(sum([(drone2_estimated[i] - drone2_actual_position[i])**2 for i in range(3)]))
        
    #     print(f"\n=== 定位精度比較 ===")
    #     print(f"僅靜態信標誤差: {error_static:.2f}m")
    #     print(f"協同定位誤差: {error_coop:.2f}m")
    #     print(f"精度改善: {((error_static - error_coop)/error_static*100):.1f}%")
        
    #     return {
    #         'drone1_estimated': drone1_estimated,
    #         'drone2_actual': drone2_actual_position,
    #         'drone2_static_estimated': drone2_static_estimated,
    #         'drone2_coop_estimated': drone2_estimated,
    #         'error_static': error_static,
    #         'error_coop': error_coop
    #     }
    
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

def cmd_set(controller: MultiDroneController) -> None:
    drone_id = int(input("Enter drone ID (0 or 1): ").strip())
    x = float(input("Enter x position: ").strip())
    y = float(input("Enter y position: ").strip())
    z = float(input("Enter z position: ").strip())
    
    controller.position_setpoint(drone_id, (x, y, z))

# def cmd_est(controller: MultiDroneController) -> None:
#     """Estimate position using BLE beacons"""
#     drone_id = input("Enter drone ID (0 or 1, or 'all' for both): ").strip()
    
#     if drone_id == 'all':
#         for i in range(len(controller.drones)):
#             controller.estimate_position(i)
#     elif drone_id.isdigit() and drone_id in [0, 1]:
#         controller.estimate_position(drone_id)
#     else:
#         print("Invalid drone ID. Please enter 0, 1, or 'all'.")

# def cmd_coop(controller: MultiDroneController) -> None:
#     """Cooperative localization (Drone1 as moving beacon)"""
#     controller.estimate_positions_coop()

def main(args=None) -> None:
    rclpy.init(args=args)

    controller = MultiDroneController()

    cmd_dict = {
        'arm': cmd_arm,
        'set': cmd_set,
        'disarm': cmd_disarm,
        'land': cmd_land,
        
        # TODO: DIRTY!!
        # 'est': lambda: [controller.estimate_position(drone_id) for drone_id in range(len(controller.drones))],
        # 'coop': lambda: controller.estimate_positions_coop()
    }
    
    print("=== Multi-Drone BLE Localization Controller ===")
    print("Available commands:")
    print("  arm  - Manual Arm all drones")
    print("  set  - Set position for a specific drone (will arm if not armed)")
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