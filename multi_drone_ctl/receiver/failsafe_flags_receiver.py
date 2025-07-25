from .common_receiver import CommonReceiver

from typing import Protocol

from std_msgs.msg import UInt64, UInt32, UInt8, Bool
from rclpy.node import Node
from rclpy.qos import QoSProfile

from px4_msgs.msg import FailsafeFlags

class FailsafeFlagsMsg(Protocol):
    timestamp: UInt64

    # Per-mode requirements
    mode_req_angular_velocity: UInt32
    mode_req_attitude: UInt32
    mode_req_local_alt: UInt32
    mode_req_local_position: UInt32
    mode_req_local_position_relaxed: UInt32
    mode_req_global_position: UInt32
    mode_req_global_position_relaxed: UInt32
    mode_req_mission: UInt32
    mode_req_offboard_signal: UInt32
    mode_req_home_position: UInt32
    mode_req_wind_and_flight_time_compliance: UInt32
    mode_req_prevent_arming: UInt32
    mode_req_manual_control: UInt32
    mode_req_other: UInt32

    # Mode requirements
    angular_velocity_invalid: Bool
    attitude_invalid: Bool
    local_altitude_invalid: Bool
    local_position_invalid: Bool
    local_position_invalid_relaxed: Bool
    global_position_invalid: Bool
    global_position_invalid_relaxed: Bool
    auto_mission_missing: Bool
    offboard_control_signal_lost: Bool
    home_position_invalid: Bool

    # Control links
    manual_control_signal_lost: Bool
    gcs_connection_lost: Bool

    # Battery
    battery_warning: UInt8
    battery_low_remaining_time: Bool
    battery_unhealthy: Bool

    # Other
    geofence_breached: Bool
    mission_failure: Bool
    vtol_fixed_wing_system_failure: Bool
    wind_limit_exceeded: Bool
    flight_time_limit_exceeded: Bool
    position_accuracy_low: Bool
    navigator_failure: Bool

    # Failure detector
    fd_critical_failure: Bool
    fd_esc_arming_failure: Bool
    fd_imbalanced_prop: Bool
    fd_motor_failure: Bool

class FailsafeFlagsReceiver(CommonReceiver):
    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.subscriber = node.create_subscription(
            FailsafeFlags, f'{prefix}/fmu/out/failsafe_flags', self.listener_callback, qos_profile)
        self.callbacks = []
    
    def add_callback(self, callback):
        if callable(callback):
            self.callbacks.append(callback)
        else:
            raise TypeError("Callback must be a callable function")
    
    def listener_callback(self, msg: FailsafeFlagsMsg) -> None:
        self.data = msg
        
        for callback in self.callbacks:
            callback(data=self.data)