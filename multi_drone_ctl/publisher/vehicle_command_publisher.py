from .common_publisher import CommonPublisher

from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Bool, Float32, UInt32, UInt16, UInt8
from px4_msgs.msg import VehicleCommand

class VehicleCommandConsts:
    VEHICLE_CMD_NAV_LAND = 21
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

    ARMING_ACTION_ARM = 1.0
    ARMING_ACTION_DISARM = 0.0

class VehicleCommandPublisher(CommonPublisher):
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.publisher = node.create_publisher(
            VehicleCommand, f'{prefix}/fmu/in/vehicle_command', qos_profile)

    def publish_command(
            self,
            command_id: UInt32=None,
            param1: Float32=None,
            param2: Float32=None,
            param3: Float32=None,
            param4: Float32=None,
            param5: Float32=None,
            param6: Float32=None,
            param7: Float32=None,
            target_system: UInt8=None,
            target_component: UInt8=None,
            source_system: UInt8=None,
            source_component: UInt16=None,
            confirmation: UInt8=None,
            from_external: Bool=None) -> None:
        
        command = VehicleCommand()

        if command_id is not None:
            command.command = command_id

        if param1 is not None:
            command.param1 = param1
        if param2 is not None:
            command.param2 = param2
        if param3 is not None:
            command.param3 = param3
        if param4 is not None:
            command.param4 = param4
        if param5 is not None:
            command.param5 = param5
        if param6 is not None:
            command.param6 = param6
        if param7 is not None:
            command.param7 = param7
        if target_system is not None:
            command.target_system = target_system
        if target_component is not None:
            command.target_component = target_component
        if source_system is not None:
            command.source_system = source_system
        if source_component is not None:
            command.source_component = source_component
        if confirmation is not None:
            command.confirmation = confirmation
        if from_external is not None:
            command.from_external = from_external

        self.publisher.publish(command)