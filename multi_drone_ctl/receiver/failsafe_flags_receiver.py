from .common_receiver import CommonReceiver

from typing import Protocol

from rclpy.node import Node
from rclpy.qos import QoSProfile

from px4_msgs.msg import FailsafeFlags # type: ignore

class FailsafeFlagsMsg(Protocol):
    timestamp: int

    # Per-mode requirements
    mode_req_angular_velocity: int
    mode_req_attitude: int
    mode_req_local_alt: int
    mode_req_local_position: int
    mode_req_local_position_relaxed: int
    mode_req_global_position: int
    mode_req_global_position_relaxed: int
    mode_req_mission: int
    mode_req_offboard_signal: int
    mode_req_home_position: int
    mode_req_wind_and_flight_time_compliance: int
    mode_req_prevent_arming: int
    mode_req_manual_control: int
    mode_req_other: int

    # Mode requirements
    angular_velocity_invalid: bool
    attitude_invalid: bool
    local_altitude_invalid: bool
    local_position_invalid: bool
    local_position_invalid_relaxed: bool
    global_position_invalid: bool
    global_position_invalid_relaxed: bool
    auto_mission_missing: bool
    offboard_control_signal_lost: bool
    home_position_invalid: bool

    # Control links
    manual_control_signal_lost: bool
    gcs_connection_lost: bool

    # Battery
    battery_warning: int
    battery_low_remaining_time: bool
    battery_unhealthy: bool

    # Other
    geofence_breached: bool
    mission_failure: bool
    vtol_fixed_wing_system_failure: bool
    wind_limit_exceeded: bool
    flight_time_limit_exceeded: bool
    position_accuracy_low: bool
    navigator_failure: bool

    # Failure detector
    fd_critical_failure: bool
    fd_esc_arming_failure: bool
    fd_imbalanced_prop: bool
    fd_motor_failure: bool

class FailsafeFlagsReceiver(CommonReceiver):
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.subscriber = node.create_subscription(
            FailsafeFlags,
            f'{prefix}/fmu/out/failsafe_flags',
            self.listener_callback,
            qos_profile
        )
        self.callbacks = []
    
    def add_callback(self, callback):
        if callable(callback):
            self.callbacks.append(callback)
        else:
            raise TypeError("Callback must be a callable function")
    
    def listener_callback(self, msg) -> None:
        self.data = msg
        
        for callback in self.callbacks:
            callback(data=self.data)