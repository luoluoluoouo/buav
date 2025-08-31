#!/usr/bin/env python3

from .common_publisher import CommonPublisher

from rclpy.node import Node
from rclpy.qos import QoSProfile

from px4_msgs.msg import VehicleOdometry

from typing import Tuple

class VehicleVisualOdometryPublisher(CommonPublisher):
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str = ''):
        self.node = node
        self.publisher = node.create_publisher(
            VehicleOdometry, f'{prefix}/fmu/in/vehicle_visual_odometry', qos_profile)

    def publish_pose(self, position: Tuple[float, float, float] = (0.0, 0.0, 0.0), 
                     orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> None:
        """
        Publish vehicle visual odometry message.
        position: (x, y, z) in NED frame
        orientation: quaternion (x, y, z, w)
        """
        msg = VehicleOdometry()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = msg.timestamp
        
        # Position
        msg.position = list(position)
        
        # Orientation quaternion
        msg.q = list(orientation)
        
        # Velocity (set to zero for hovering)
        msg.velocity = [0.0, 0.0, 0.0]
        msg.angular_velocity = [0.0, 0.0, 0.0]
        
        # Covariances (small values indicate high confidence)
        msg.position_variance = [0.1, 0.1, 0.1]  # 10cm std dev
        msg.orientation_variance = [0.01, 0.01, 0.01]  # Small rotation variance
        msg.velocity_variance = [0.1, 0.1, 0.1]
        
        # Reset counters
        msg.reset_counter = 0
        msg.quality = 100  # High quality
        
        # Frame IDs (set to appropriate values or remove if not available)
        # msg.local_frame = 1  # NED frame
        
        self.publisher.publish(msg)

    def publish(self, *args, **kwargs) -> None:
        """Default publish method for CommonPublisher compatibility"""
        self.publish_pose()
