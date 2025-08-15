from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.qos import QoSProfile

class CommonPublisher(ABC):
    @abstractmethod
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str='') -> None:
        """
        Initialize the publisher with the given node, QoS profile, and prefix.
        """
        pass
    
    @abstractmethod
    def publish(self) -> None:
        pass
