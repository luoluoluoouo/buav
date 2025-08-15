from .common_publisher import CommonPublisher

from rclpy.node import Node
from rclpy.qos import QoSProfile

from px4_msgs.msg import TrajectorySetpoint # type: ignore

from typing import Tuple

class TrajectorySetpointPublisher(CommonPublisher):
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.publisher = node.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', qos_profile)

    def publish(
            self,
            position: Tuple[float, float, float] | None = None,
            velocity: Tuple[float, float, float] | None = None,
            acceleration: Tuple[float, float, float] | None = None,
            jerk: Tuple[float, float, float] | None = None,
            yaw: float | None = None,
            yawspeed: float | None = None) -> None:
        
        command = TrajectorySetpoint()
        command.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        
        if position is not None:
            command.position = position
        if velocity is not None:
            command.velocity = velocity
        if acceleration is not None:
            command.acceleration = acceleration
        if jerk is not None:
            command.jerk = jerk
        if yaw is not None:
            command.yaw = yaw
        if yawspeed is not None:
            command.yawspeed = yawspeed

        self.publisher.publish(command)