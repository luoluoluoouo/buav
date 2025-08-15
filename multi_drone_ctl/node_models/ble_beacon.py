import math
import random

from rclpy.node import Node

class BLEbeacon(Node):
    def __init__(self, node_name: str, position: tuple[float, float, float], rssi_settings: dict) -> None:
        super().__init__(f'{node_name}')

        self.position = position

        # 模擬參數
        self.rssi0: float = rssi_settings['rssi0'] #+ random.uniform(-5, 5)  # 基準 RSSI 值，隨機偏移
        self.path_loss_n: float = rssi_settings['path_loss_n'] #+ random.uniform(-0.1, 0.1)  # 路徑損失指數，隨機偏移
        self.noise_stddev: float = rssi_settings['noise_stddev']  # dBm 隨機雜訊

    def get_rssi(self, drone_position: tuple[float, float, float]):
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