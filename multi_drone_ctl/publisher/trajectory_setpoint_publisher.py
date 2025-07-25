from .common_publisher import CommonPublisher

from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32
from px4_msgs.msg import TrajectorySetpoint

class TrajectorySetpointPublisher(CommonPublisher):
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.publisher = node.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', qos_profile)

    def publish_command(
            self,
            position: list[Float32, Float32, Float32] = None,
            velocity: list[Float32, Float32, Float32] = None,
            acceleration: list[Float32, Float32, Float32] = None,
            jerk: list[Float32, Float32, Float32] = None,
            yaw: Float32=None,
            yawspeed: Float32=None) -> None:
        
        command = TrajectorySetpoint()
        
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