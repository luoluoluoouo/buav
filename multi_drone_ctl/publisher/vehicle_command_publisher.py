from .common_publisher import CommonPublisher

from rclpy.node import Node
from rclpy.qos import QoSProfile

from px4_msgs.msg import VehicleCommand # type: ignore

class VehicleCommandEnum:
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

    def publish(
            self,
            command_id: int | None = None,
            param1: float | None = None,
            param2: float | None = None,
            param3: float | None = None,
            param4: float | None = None,
            param5: float | None = None,
            param6: float | None = None,
            param7: float | None = None,
            target_system: int | None = None,
            target_component: int | None = None,
            source_system: int | None = None,
            source_component: int | None = None,
            confirmation: int | None = None,
            from_external: bool | None = None) -> None:

        if command_id is None:
            raise ValueError("command_id must be provided")

        msg = VehicleCommand()
        msg.command = command_id
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)

        if param1 is not None:
            msg.param1 = param1
        if param2 is not None:
            msg.param2 = param2
        if param3 is not None:
            msg.param3 = param3
        if param4 is not None:
            msg.param4 = param4
        if param5 is not None:
            msg.param5 = param5
        if param6 is not None:
            msg.param6 = param6
        if param7 is not None:
            msg.param7 = param7
        if target_system is not None:
            msg.target_system = target_system
        if target_component is not None:
            msg.target_component = target_component
        if source_system is not None:
            msg.source_system = source_system
        if source_component is not None:
            msg.source_component = source_component
        if confirmation is not None:
            msg.confirmation = confirmation
        if from_external is not None:
            msg.from_external = from_external

        self.publisher.publish(msg)