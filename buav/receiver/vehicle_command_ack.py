from .common_receiver import CommonReceiver

from rclpy.node import Node
from rclpy.qos import QoSProfile
from typing import Protocol, Callable, List, Any

from px4_msgs.msg import VehicleCommandAck  # type: ignore

class VehicleCommandAckEnum:
    MESSAGE_VERSION = 0
    
    VEHICLE_CMD_RESULT_ACCEPTED = 0
    VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1
    VEHICLE_CMD_RESULT_DENIED = 2
    VEHICLE_CMD_RESULT_UNSUPPORTED = 3
    VEHICLE_CMD_RESULT_FAILED = 4
    VEHICLE_CMD_RESULT_IN_PROGRESS = 5
    VEHICLE_CMD_RESULT_CANCELLED = 6
    
    ARM_AUTH_DENIED_REASON_GENERIC = 0
    ARM_AUTH_DENIED_REASON_NONE = 1
    ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
    ARM_AUTH_DENIED_REASON_TIMEOUT = 3
    ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
    ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5
    
    ORB_QUEUE_LENGTH = 4

class VehicleCommandAckMsg(Protocol):
    timestamp: int
    
    command: int
    result: int
    result_param1: int
    result_param2: int
    target_system: int
    target_component: int
    
    from_external: bool

class VehicleCommandAckReceiver(CommonReceiver):
    """
    A class to receive vehicle local position data.
    """

    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.subscriber = node.create_subscription(
            VehicleCommandAck,
            f'{prefix}/fmu/out/vehicle_command_ack',
            self._listener_callback,
            qos_profile
        )
        self.callbacks: List[Callable[[VehicleCommandAckMsg], Any]] = []
    
    def add_callback(self, callback: Callable[[VehicleCommandAckMsg], Any]) -> None:
        if callable(callback):
            self.callbacks.append(callback)
        else:
            raise TypeError("Callback must be a callable function")

    def _listener_callback(self, msg: VehicleCommandAckMsg) -> None:
        for callback in self.callbacks:
            callback(msg)