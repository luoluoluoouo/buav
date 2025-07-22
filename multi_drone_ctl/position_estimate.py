from drone_ctl import *
from geometry_msgs.msg import PointStamped
from collections import defaultdict

class DronePositionEstimator(Node):
    def __init__(self, beacon_info, prefix):
        super().__init__(f'drone_position_estimator_{prefix.strip("/")}')
        self.prefix = prefix
        self.beacon_positions = beacon_info  # {name: (x, y, z)}
        self.rssi_data = {}  # {name: latest RSSI}

        # Create RSSI subscribers
        for beacon_name in self.beacon_positions:
            topic = f'/ble{self.prefix}/beacon_{beacon_name}/rssi'
            self.create_subscription(Float32, topic, self._make_callback(beacon_name), qos_profile)

        # Publish estimated position
        self.position_publisher = self.create_publisher(PointStamped, f'{prefix}/ble_estimated_position', 10)

        # Timer for periodic estimation
        self.timer = self.create_timer(1.0, self.estimate_position)

        # RSSI params
        self.rssi0 = -50.0
        self.path_loss_n = 2.5

    def _make_callback(self, beacon_name):
        def callback(msg: Float32):
            self.rssi_data[beacon_name] = msg.data
        return callback

    def estimate_position(self):
        if len(self.rssi_data) < 3:
            self.get_logger().warn("Not enough RSSI data for estimation")
            return

        positions = []
        distances = []
        for name, rssi in self.rssi_data.items():
            if name in self.beacon_positions:
                pos = self.beacon_positions[name]
                d = 10 ** ((self.rssi0 - rssi) / (10 * self.path_loss_n))
                positions.append(pos)
                distances.append(d)

        # Solve nonlinear least squares
        import numpy as np
        from scipy.optimize import least_squares

        def residuals(xy):
            x, y = xy
            return [np.linalg.norm([x - px, y - py]) - d
                    for (px, py, _), d in zip(positions, distances)]

        try:
            result = least_squares(residuals, x0=[0.0, 0.0])
            x_est, y_est = result.x
            self.get_logger().info(f"Estimated Position: x={x_est:.2f}, y={y_est:.2f}")

            # Publish result
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.point.x = x_est
            msg.point.y = y_est
            msg.point.z = 0.0
            self.position_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Estimation failed: {str(e)}")


if __name__ == '__main__':
    rclpy.init(args=None)
    with open('/home/ada/luoluo/px4_ros2_ws/src/multi_drone_ctl/multi_drone_ctl/config.yaml', 'r') as f:
        config = yaml.safe_load(f)

    # 建立 estimator（針對每台 drone）
    for drone_config in config['drones'].values():
        beacon_info = {beacon['name']: tuple(beacon['position']) for beacon in config['ble_beacons'].values()}
        estimator = DronePositionEstimator(beacon_info=beacon_info, prefix=drone_config['prefix'])
        rclpy.spin_once(estimator, timeout_sec=0.1)  # 讓 node 建立完成