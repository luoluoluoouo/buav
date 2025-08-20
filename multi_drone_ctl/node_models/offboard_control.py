import math

from rclpy.node import Node
from rclpy.qos import QoSProfile

from ..publisher.trajectory_setpoint_publisher import TrajectorySetpointPublisher
from ..publisher.offboard_control_mode_publisher import OffboardControlModePublisher
from ..publisher.vehicle_command_publisher import VehicleCommandPublisher, VehicleCommandEnum

from ..receiver.vehicle_status_receiver import VehicleStatusEnum, VehicleStatusReceiver, VehicleStatusMsg
from ..receiver.vehicle_local_position_receiver import VehicleLocalPositionReceiver, VehicleLocalPositionMsg
from ..receiver.vehicle_command_ack import VehicleCommandAckReceiver, VehicleCommandAckMsg

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, qos_profile: QoSProfile, name: str, prefix: str, target_system: int, gazebo_position: tuple[float, float, float]) -> None:
        super().__init__(f'offboard_control_{name}')

        self.prefix = prefix
        self.target_system = target_system
        self.gazebo_x, self.gazebo_y, self.gazebo_z = gazebo_position
        
        self.x = self.gazebo_x
        self.y = self.gazebo_y
        self.z = self.gazebo_z
        
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 0.0
        self._target_yaw = 0.0
        
        self._offboard_control_mode_publisher = OffboardControlModePublisher(self, qos_profile, prefix)
        self._trajectory_setpoint_publisher = TrajectorySetpointPublisher(self, qos_profile, prefix)
        self._vehicle_command_publisher = VehicleCommandPublisher(self, qos_profile, prefix)

        self._vehicle_local_position_receiver = VehicleLocalPositionReceiver(self, qos_profile, prefix)
        # self._vehicle_local_position_receiver.add_callback(self._vehicle_local_position_callback)
        
        self._vehicle_status_receiver = VehicleStatusReceiver(self, qos_profile, prefix)
        self._vehicle_status_receiver.add_callback(self._vehicle_status_callback)

        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = self._vehicle_local_position_receiver.get_simple_msg()
        self.vehicle_status = self._vehicle_status_receiver.get_simple_msg()

        self.timer = self.create_timer(0.1, self._timer_callback)
        
        self.flying = False
        
        self.get_logger().warn(f'{prefix} + {target_system}')

    def arm(self) -> None:
        self._vehicle_command_publisher.publish(
            command_id=VehicleCommandEnum.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=VehicleCommandEnum.ARMING_ACTION_ARM,
            target_system=self.target_system,
        )

    def disarm(self) -> None:
        self._vehicle_command_publisher.publish(
            command_id=VehicleCommandEnum.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=VehicleCommandEnum.ARMING_ACTION_DISARM,
            target_system=self.target_system,
        )

    def engage_offboard_mode(self) -> None:
        self._vehicle_command_publisher.publish(
            command_id=VehicleCommandEnum.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
            target_system=self.target_system,
        )

    def land(self) -> None:
        self._vehicle_command_publisher.publish(
            command_id=VehicleCommandEnum.VEHICLE_CMD_NAV_LAND,
            target_system=self.target_system,
        )
        
        self.flying = False

    def set_absolute_position(self, is_gazebo: bool = False, target_x: float = 0.0, target_y: float = 0.0, target_z: float = 0.0, target_yaw: float = 0.0) -> None:
        if target_z < 2.0:
            self.get_logger().error("Target Z position must be at least 2.0 meters above the ground.")
            raise ValueError("Target Z position must be at least 2.0 meters above the ground.")
        if abs(target_x) > 5.0 or abs(target_y) > 5.0:
            self.get_logger().error("Target X and Y positions must be within -5.0 to 5.0 meters.")
            raise ValueError("Target X and Y positions must be within -5.0 to 5.0 meters.")
        if target_yaw < 0.0 or target_yaw > 2*math.pi:
            self.get_logger().error("Target yaw must be between 0 and 360 degrees.")
            raise ValueError("Target yaw must be between 0 and 360 degrees.")
        
        if is_gazebo:
            self._target_x = target_y - self.gazebo_y
            self._target_y = target_x - self.gazebo_x
            self._target_z = self.gazebo_z - target_z
        else:
            self._target_x = target_y
            self._target_y = target_x
            self._target_z = - target_z

        self._target_yaw = - target_yaw

        self.flying = True

    def get_position(self):
        self.abs_x = self.gazebo_x + self.vehicle_local_position.y
        self.abs_y = self.gazebo_y + self.vehicle_local_position.x
        self.abs_z = self.gazebo_z - self.vehicle_local_position.z
        return (self.abs_x, self.abs_y, self.abs_z)

    def is_armed(self) -> bool:
        return self.vehicle_status.arming_state == VehicleStatusEnum.ARMING_STATE_ARMED
    
    def _vehicle_local_position_callback(self, msg: VehicleLocalPositionMsg) -> None:
        self.vehicle_local_position = msg
        self.x = self.vehicle_local_position.x
        self.y = self.vehicle_local_position.y
        self.z = self.vehicle_local_position.z
        
        if self.flying:
            print(msg.x, msg.y, msg.z)

    def _vehicle_status_callback(self, msg: VehicleStatusMsg) -> None:
        self.vehicle_status = msg
    
    def _vehicle_command_ack_callback(self, msg: VehicleCommandAckMsg) -> None:
        pass
    
    def _timer_callback(self) -> None:
        """Callback function for the timer."""
        self._offboard_control_mode_publisher.heartbeat()
        
        if self.offboard_setpoint_counter == 15:
            self.engage_offboard_mode()
            
        if self.flying:
            self._keep_status()
            self._trajectory_setpoint_publisher.publish(
                position=(self._target_x, self._target_y, self._target_z),
                yaw=self._target_yaw
            )
            
        if self.offboard_setpoint_counter < 16:
            self.offboard_setpoint_counter += 1
    
    def _keep_status(self) -> None:
        """Ensure the vehicle is in offboard mode and armed."""
        if self.vehicle_status.nav_state != VehicleStatusEnum.NAVIGATION_STATE_OFFBOARD:
            self.engage_offboard_mode()
        
        if not self.is_armed():
            self.arm()

    def get_offboard_control_mode_publisher(self) -> OffboardControlModePublisher:
        return self._offboard_control_mode_publisher
    
    def get_trajectory_setpoint_publisher(self) -> TrajectorySetpointPublisher:
        return self._trajectory_setpoint_publisher
    
    def get_vehicle_command_publisher(self) -> VehicleCommandPublisher:
        return self._vehicle_command_publisher
    
    def get_vehicle_local_position_receiver(self) -> VehicleLocalPositionReceiver:
        return self._vehicle_local_position_receiver
    
    def get_vehicle_status_receiver(self) -> VehicleStatusReceiver:
        return self._vehicle_status_receiver