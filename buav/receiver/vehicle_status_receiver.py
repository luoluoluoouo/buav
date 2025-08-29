from .common_receiver import CommonReceiver

from rclpy.node import Node
from rclpy.qos import QoSProfile
from typing import Protocol, Callable, List, Any

from px4_msgs.msg import VehicleStatus  # type: ignore

class VehicleStatusEnum:
    MESSAGE_VERSION = 1
    
    ARMING_STATE_DISARMED = 1
    ARMING_STATE_ARMED    = 2
    
    ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0
    ARM_DISARM_REASON_STICK_GESTURE = 1
    ARM_DISARM_REASON_RC_SWITCH = 2
    ARM_DISARM_REASON_COMMAND_INTERNAL = 3
    ARM_DISARM_REASON_COMMAND_EXTERNAL = 4
    ARM_DISARM_REASON_MISSION_START = 5
    ARM_DISARM_REASON_SAFETY_BUTTON = 6
    ARM_DISARM_REASON_AUTO_DISARM_LAND = 7
    ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8
    ARM_DISARM_REASON_KILL_SWITCH = 9
    ARM_DISARM_REASON_LOCKDOWN = 10
    ARM_DISARM_REASON_FAILURE_DETECTOR = 11
    ARM_DISARM_REASON_SHUTDOWN = 12
    ARM_DISARM_REASON_UNIT_TEST = 13
    
    NAV_STATE_MANUAL = 0
    NAV_STATE_ALTCTL = 1
    NAV_STATE_POSCTL = 2
    NAV_STATE_AUTO_MISSION = 3
    NAV_STATE_AUTO_LOITER = 4
    NAV_STATE_AUTO_RTL = 5
    NAVIGATION_STATE_POSITION_SLOW = 6
    NAVIGATION_STATE_FREE5 = 7
    NAVIGATION_STATE_FREE4 = 8
    NAVIGATION_STATE_FREE3 = 9
    NAVIGATION_STATE_ACRO = 10
    NAVIGATION_STATE_FREE2 = 11
    NAVIGATION_STATE_DESCEND = 12
    NAVIGATION_STATE_TERMINATION = 13
    NAVIGATION_STATE_OFFBOARD = 14
    NAVIGATION_STATE_STAB = 15
    NAVIGATION_STATE_FREE1 = 16
    NAVIGATION_STATE_AUTO_TAKEOFF = 17
    NAVIGATION_STATE_AUTO_LAND = 18
    NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19
    NAVIGATION_STATE_AUTO_PRECLAND = 20
    NAVIGATION_STATE_ORBIT = 21
    NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22
    NAVIGATION_STATE_EXTERNAL1 = 23
    NAVIGATION_STATE_EXTERNAL2 = 24
    NAVIGATION_STATE_EXTERNAL3 = 25
    NAVIGATION_STATE_EXTERNAL4 = 26
    NAVIGATION_STATE_EXTERNAL5 = 27
    NAVIGATION_STATE_EXTERNAL6 = 28
    NAVIGATION_STATE_EXTERNAL7 = 29
    NAVIGATION_STATE_EXTERNAL8 = 30
    NAVIGATION_STATE_MAX = 31
    
    FAILURE_NONE = 0
    FAILURE_ROLL = 1
    FAILURE_PITCH = 2
    FAILURE_ALT = 3
    FAILURE_EXT = 8
    FAILURE_ARM_ESC = 16
    FAILURE_BATTERY = 32
    FAILURE_IMBALANCED_PROP = 64
    FAILURE_MOTOR = 128
    
    HIL_STATE_OFF = 0
    HIL_STATE_ON = 1
    
    VEHICLE_TYPE_UNSPECIFIED = 0
    VEHICLE_TYPE_ROTARY_WING = 1
    VEHICLE_TYPE_FIXED_WING = 2
    VEHICLE_TYPE_ROVER = 3
    
    FAILSAFE_DEFER_STATE_DISABLED = 0
    FAILSAFE_DEFER_STATE_ENABLED = 1
    FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2

class VehicleStatusMsg(Protocol):
    timestamp: int
    
    armed_time: int
    takeoff_time: int
    
    arming_state: int
    
    latest_arming_reason: int
    latest_disarming_reason: int
    
    nav_state_timestamp: int
    
    nav_state_user_intention: int
    
    nav_state: int
    executor_in_charge: int
    
    vaild_nav_states_mask: int
    can_set_nav_states_mask: int
    
    failure_detector_status: int
    
    hil_state: int
    
    vehicle_type: int
    
    failsafe: bool
    failsafe_and_user_took_over: bool
    failsafe_defer_state: int
    
    gcs_connection_lost: bool
    gcs_connection_lost_counter: int
    high_latency_data_link_lost: bool
    
    is_vtol: bool
    is_vtol_tailsitter: bool
    in_transition_mode: bool
    in_transition_to_fw: bool
    
    system_type: int
    system_id: int
    component_id: int
    
    safety_button_available: bool
    safety_off: bool
    
    power_input_valid: bool
    usb_connected: bool
    
    open_drone_id_system_present: bool
    open_drone_id_system_healthy: bool
    
    parachute_system_present: bool
    parachute_system_healthy: bool
    
    rc_calibration_in_progress: bool
    calibration_enabled: bool
    
    pre_flight_checks_pass: bool

class VehicleStatusReceiver(CommonReceiver):
    """
    A class to receive vehicle local position data.
    """

    def __init__(self, node: Node, qos_profile: QoSProfile, prefix: str=''):
        self.node = node
        self.subscriber = node.create_subscription(
            VehicleStatus,
            f'{prefix}/fmu/out/vehicle_status_v1',
            self._listener_callback,
            qos_profile
        )
        self.callbacks: List[Callable[[VehicleStatusMsg], Any]] = []
    
    def add_callback(self, callback: Callable[[VehicleStatusMsg], Any]) -> None:
        if callable(callback):
            self.callbacks.append(callback)
        else:
            raise TypeError("Callback must be a callable function")

    def _listener_callback(self, msg: VehicleStatusMsg) -> None:
        for callback in self.callbacks:
            callback(msg)
    
    def get_simple_msg(self) -> VehicleStatusMsg:
        return VehicleStatus()