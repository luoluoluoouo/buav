from .common_receiver import CommonReceiver

from typing import Protocol

from rclpy.node import Node
from rclpy.qos import QoSProfile

from px4_msgs.msg import VehicleLocalPosition # type: ignore

class VehicleLocalPositionEnum:
    MESSAGE_VERSION = 1
    DIST_BOTTOM_SENSOR_NONE = 0
    DIST_BOTTOM_SENSOR_RANGE = 1
    DIST_BOTTOM_SENSOR_FLOW = 2

class VehicleLocalPositionMsg(Protocol):
    timestamp: int
    timestamp_sample: int
    
    xy_vaild: bool
    z_valid: bool
    v_xy_valid: bool
    v_z_valid: bool
    
    x: float
    y: float
    z: float
    
    delta_x: tuple[float, float]
    delta_z: float
    z_reset_counter: int
    
    vx: float
    vy: float
    vz: float
    z_deriv: float
    
    delta_vxy: tuple[float, float]
    delta_vz: float
    vz_reset_counter: int
    
    ax: float
    ay: float
    az: float
    
    heading: float
    heading_var: float
    unaided_heading: float
    delta_heading: float
    heading_reset_counter: int
    heading_good_for_control: bool
    
    tilt_var: float
    
    xy_global: bool
    z_global: bool
    ref_timestamp: int
    ref_lat: float
    ref_lon: float
    ref_alt: float
    
    dist_bottom_vaild: bool
    dist_bottom: float
    dist_bottom_var: float
    
    delta_dist_bottom: float
    dist_bottom_reset_counter: int
    
    dist_bottom_sensor_bitfield: int
    
    eph: float
    epv: float
    evh: float
    evv: float
    
    dead_rockoning: bool
    
    vxy_max: float
    vz_max: float
    hagl_min: float
    hagl_max_z: float
    hagl_max_xy: float
    

class VehicleLocalPositionReceiver(CommonReceiver):
    """
    A class to receive vehicle local position data.
    """

    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.subscriber = node.create_subscription(
            VehicleLocalPosition,
            f'{prefix}/fmu/out/vehicle_local_position',
            self._listener_callback,
            qos_profile
        )
        self.callbacks = []
    
    def add_callback(self, callback) -> None:
        if callable(callback):
            self.callbacks.append(callback)
        else:
            raise TypeError("Callback must be a callable function")

    def _listener_callback(self, msg) -> None:
        for callback in self.callbacks:
            callback(msg)