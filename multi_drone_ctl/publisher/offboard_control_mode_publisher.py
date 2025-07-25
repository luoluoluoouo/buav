from .common_publisher import CommonPublisher

from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import UInt64, Bool
from px4_msgs.msg import OffboardControlMode

class OffboardControlModePublisher(CommonPublisher):
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.publisher = node.create_publisher(
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', qos_profile)
    
    def publish_command(
            self,
            position: Bool = None,
            velocity: Bool = None,
            acceleration: Bool = None,
            attitude: Bool = None,
            body_rate: Bool = None,
            thrust_and_torque: Bool = None,
            direct_actuator: Bool = None) -> None:
        
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