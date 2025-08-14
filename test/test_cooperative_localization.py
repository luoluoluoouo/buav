#!/usr/bin/env python3
"""
測試協同定位功能的腳本
模擬不同場景下的定位精度比較
"""

import math
import random
from scipy.optimize import least_squares

class BLEbeacon:
    """簡化的 BLE 信標類別，用於測試"""
    def __init__(self, node_name: str, position: tuple, rssi_settings: dict) -> None:
        self.node_name = node_name
        self.position = position
        self.rssi0 = rssi_settings['rssi0']
        self.path_loss_n = rssi_settings['path_loss_n']
        self.noise_stddev = rssi_settings['noise_stddev']

    def get_rssi(self, drone_position: tuple):
        drone_x, drone_y, drone_z = drone_position
        dx = drone_x - self.position[0]
        dy = drone_y - self.position[1]
        dz = drone_z - self.position[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        distance = max(distance, 0.1)
        
        rssi = self.rssi0 - 10 * self.path_loss_n * math.log10(distance)
        noise = random.gauss(0, self.noise_stddev)
        rssi += noise
        
        return rssi

class CooperativeLocalizationTest:
    def __init__(self):
        self.RSSI_SETTINGS = {'rssi0': -50, 'path_loss_n': 2.0, 'noise_stddev': 1.0}
        
        # 設置靜態信標位置
        self.static_beacons = [
            BLEbeacon('beacon1', (0.0, 0.0, 0.0), self.RSSI_SETTINGS),
            BLEbeacon('beacon2', (10.0, 0.0, 0.0), self.RSSI_SETTINGS),
            BLEbeacon('beacon3', (0.0, 10.0, 0.0), self.RSSI_SETTINGS),
            BLEbeacon('beacon4', (10.0, 10.0, 0.0), self.RSSI_SETTINGS)
        ]
    
    def simulate_localization(self, target_position, beacons, label=""):
        """模擬定位過程"""
        # 獲取 RSSI 值
        rssi_values = [beacon.get_rssi(target_position) for beacon in beacons]
        
        def residuals(x):
            return [rssi - (self.RSSI_SETTINGS['rssi0'] - 10 * self.RSSI_SETTINGS['path_loss_n'] * math.log10(
                max(0.1, math.sqrt((x[0] - beacon.position[0])**2 + (x[1] - beacon.position[1])**2 + (x[2] - beacon.position[2])**2)))
            ) for rssi, beacon in zip(rssi_values, beacons)]
        
        # 使用目標位置附近的點作為初始猜測
        initial_guess = [target_position[0] + 1, target_position[1] + 1, target_position[2]]
        result = least_squares(residuals, initial_guess)
        
        estimated_position = result.x
        error = math.sqrt(sum([(estimated_position[i] - target_position[i])**2 for i in range(3)]))
        
        print(f"{label}")
        print(f"  實際位置: ({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f})")
        print(f"  估測位置: ({estimated_position[0]:.2f}, {estimated_position[1]:.2f}, {estimated_position[2]:.2f})")
        print(f"  定位誤差: {error:.2f}m")
        
        return estimated_position, error
    
    def test_cooperative_scenarios(self):
        """測試不同協同定位場景"""
        print("=== 協同定位測試 ===\n")
        
        # 場景 1: drone1 在靜態信標範圍內
        drone1_pos = (3.0, 4.0, -2.0)
        drone2_pos = (7.0, 6.0, -3.0)
        
        print("場景 1: 兩架無人機都在靜態信標覆蓋範圍內")
        print("-" * 50)
        
        # 估測 drone1 位置（僅使用靜態信標）
        drone1_est, drone1_error = self.simulate_localization(
            drone1_pos, self.static_beacons, "Drone1 定位（靜態信標）:"
        )
        
        # 創建 drone1 移動信標
        drone1_beacon = BLEbeacon('drone1_beacon', drone1_est, self.RSSI_SETTINGS)
        
        # 估測 drone2 位置（僅使用靜態信標）
        drone2_est_static, drone2_error_static = self.simulate_localization(
            drone2_pos, self.static_beacons, "\nDrone2 定位（僅靜態信標）:"
        )
        
        # 估測 drone2 位置（使用靜態信標 + drone1 移動信標）
        all_beacons = self.static_beacons + [drone1_beacon]
        drone2_est_coop, drone2_error_coop = self.simulate_localization(
            drone2_pos, all_beacons, "\nDrone2 定位（協同定位）:"
        )
        
        improvement = ((drone2_error_static - drone2_error_coop) / drone2_error_static) * 100
        print(f"\n精度改善: {improvement:.1f}%")
        
        # 場景 2: drone2 在靜態信標覆蓋邊緣
        print("\n\n場景 2: Drone2 在靜態信標覆蓋邊緣")
        print("-" * 50)
        
        drone1_pos = (2.0, 3.0, -2.0)
        drone2_pos = (12.0, 8.0, -3.0)  # 距離靜態信標較遠
        
        # 重複測試過程
        drone1_est, _ = self.simulate_localization(
            drone1_pos, self.static_beacons, "Drone1 定位（靜態信標）:"
        )
        
        drone1_beacon.position = drone1_est
        
        drone2_est_static, drone2_error_static = self.simulate_localization(
            drone2_pos, self.static_beacons, "\nDrone2 定位（僅靜態信標）:"
        )
        
        all_beacons = self.static_beacons + [drone1_beacon]
        drone2_est_coop, drone2_error_coop = self.simulate_localization(
            drone2_pos, all_beacons, "\nDrone2 定位（協同定位）:"
        )
        
        improvement = ((drone2_error_static - drone2_error_coop) / drone2_error_static) * 100
        print(f"\n精度改善: {improvement:.1f}%")
        
        # 場景 3: 多次運行統計分析
        print("\n\n場景 3: 統計分析（運行 50 次）")
        print("-" * 50)
        
        errors_static = []
        errors_coop = []
        
        for i in range(50):
            # 隨機位置
            drone1_pos = (2 + i*0.1, 3 + i*0.05, -2.0)
            drone2_pos = (7 + i*0.08, 6 + i*0.06, -3.0)
            
            # 靜態信標定位
            _, drone1_error = self.simulate_localization(drone1_pos, self.static_beacons, "")
            drone1_beacon.position = drone1_pos  # 使用真實位置作為移動信標
            
            _, error_static = self.simulate_localization(drone2_pos, self.static_beacons, "")
            _, error_coop = self.simulate_localization(drone2_pos, all_beacons, "")
            
            errors_static.append(error_static)
            errors_coop.append(error_coop)
        
        avg_error_static = sum(errors_static) / len(errors_static)
        avg_error_coop = sum(errors_coop) / len(errors_coop)
        avg_improvement = ((avg_error_static - avg_error_coop) / avg_error_static) * 100
        
        print(f"平均靜態定位誤差: {avg_error_static:.2f}m")
        print(f"平均協同定位誤差: {avg_error_coop:.2f}m")
        print(f"平均精度改善: {avg_improvement:.1f}%")

def main():
    test = CooperativeLocalizationTest()
    test.test_cooperative_scenarios()

if __name__ == '__main__':
    main()
