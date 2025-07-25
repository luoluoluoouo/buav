from .common_publisher import CommonPublisher

from rclpy.node import Node
from rclpy.qos import QoSProfile

from px4_msgs.msg import OffboardControlMode # type: ignore

class OffboardControlModePublisher(CommonPublisher):
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.publisher = node.create_publisher(
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', qos_profile)
    
    def publish_command(
            self,
            position: bool | None = None,
            velocity: bool | None = None,
            acceleration: bool | None = None,
            attitude: bool | None = None,
            body_rate: bool | None = None,
            thrust_and_torque: bool | None = None,
            direct_actuator: bool | None = None) -> None:

        command = OffboardControlMode()

        command.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)

        if position is not None:
            command.position = position
        if velocity is not None:
            command.velocity = velocity
        if acceleration is not None:
            command.acceleration = acceleration
        if attitude is not None:
            command.attitude = attitude
        if body_rate is not None:
            command.body_rate = body_rate
        if thrust_and_torque is not None:
            command.thrust_and_torque = thrust_and_torque
        if direct_actuator is not None:
            command.direct_actuator = direct_actuator

        self.publisher.publish(command)