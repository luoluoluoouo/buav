from .common_receiver import CommonReceiver

from typing import Protocol, Callable, List, Any

from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

class PointCloud2Msg(Protocol):
    header: Any  # Replace with actual Header type
    height: int
    width: int
    fields: List[Any]  # Replace with actual PointField type
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool

class PointCloudReceiver(CommonReceiver):
    def __init__(self, node: Node):
        self.node = node
        self.subscription = self.node.create_subscription(
            PointCloud2,
            '/camera/points',
            self._listener_callback,
            1
        )
        self.callbacks: List[Callable[[PointCloud2Msg], Any]] = []

    def add_callback(self, callback: Callable[[PointCloud2Msg], Any]) -> None:
        if callable(callback):
            self.callbacks.append(callback)
        else:
            raise TypeError("Callback must be a callable function")

    def _listener_callback(self, msg: PointCloud2Msg) -> None:
        for callback in self.callbacks:
            callback(msg)