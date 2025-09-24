#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# --- 小工具：px4_msgs v1.16 之後會在話題尾巴加 _v<number>，這裡自動帶版本 ---
def topic(name, T):
    ver = getattr(T, 'MESSAGE_VERSION', 0)
    return f"{name}_v{ver}" if ver else name

# --- 四元數運算 ---
def q_conj(q):  # (w,x,y,z)
    return (q[0], -q[1], -q[2], -q[3])

def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )

class OdomBridge(Node):
    def __init__(self):
        super().__init__('vehicle_odom_bridge')

        # 發 /odom 與 odom->base_link 的 TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 訂閱 PX4 的 VehicleOdometry（uXRCE-DDS → ROS2）
        self.sub = self.create_subscription(
            VehicleOdometry,
            '/px4_1/fmu/out/vehicle_odometry',
            self.cb,
            qos_profile
        )

        # 固定的座標對齊：把 NED 坐標表達改成 ENU（座標軸交換矩陣）
        # 這個對應的等效四元數為：Rz(+90°) * Rx(180°)
        s2 = math.sqrt(0.5)
        self.q_map_ned_to_enu = (0.0, s2, s2, 0.0)  # (w,x,y,z)，單位四元數
        self.q_map_inv = q_conj(self.q_map_ned_to_enu)

    def cb(self, msg: VehicleOdometry):
        # 位置：NED [N,E,D] → ENU [E,N,-D]
        px, py, pz = msg.position  # N, E, D
        x_enu = float(py)
        y_enu = float(px)
        z_enu = float(-pz)

        # 姿態：PX4給的是「FRD(機體) -> 參考座標(通常NED)」的旋轉四元數 q_b2n
        # 我們需要 TF 的方向「父(odom, ENU) -> 子(base_link, ENU)」
        # 步驟：
        # 1) q_n2b = q_b2n 的反向 = 共軛（NED -> 機體）
        # 2) 把 NED 表達換到 ENU： q_enu = q_map * q_n2b * q_map^{-1}
        # 參考：PX4 VehicleOdometry 定義（q: rotation from FRD body frame to reference frame）
        qw, qx, qy, qz = msg.q  # PX4 順序就是 (w,x,y,z)
        q_b2n = (float(qw), float(qx), float(qy), float(qz))
        q_n2b = q_conj(q_b2n)

        q_enu = q_mul(self.q_map_ned_to_enu, q_mul(q_n2b, self.q_map_inv))
        qw_e, qx_e, qy_e, qz_e = q_enu

        # 發 /odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'      # 父座標（世界）
        odom.child_frame_id = 'base_link'  # 子座標（機體）
        odom.pose.pose.position.x = float(x_enu)
        odom.pose.pose.position.y = float(y_enu)
        odom.pose.pose.position.z = float(z_enu)
        odom.pose.pose.orientation.w = float(qw_e)
        odom.pose.pose.orientation.x = float(qx_e)
        odom.pose.pose.orientation.y = float(qy_e)
        odom.pose.pose.orientation.z = float(qz_e)
        self.odom_pub.publish(odom)

        # 發 TF：odom -> base_link
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(x_enu)
        t.transform.translation.y = float(y_enu)
        t.transform.translation.z = float(z_enu)
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(OdomBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
