import math
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile

from ..publisher.trajectory_setpoint_publisher import TrajectorySetpointPublisher
from ..publisher.offboard_control_mode_publisher import OffboardControlModePublisher
from ..publisher.vehicle_command_publisher import VehicleCommandPublisher, VehicleCommandEnum
from ..publisher.vehicle_visual_odometry_publisher import VehicleVisualOdometryPublisher

from ..receiver.vehicle_status_receiver import VehicleStatusEnum, VehicleStatusReceiver, VehicleStatusMsg
from ..receiver.vehicle_local_position_receiver import VehicleLocalPositionReceiver, VehicleLocalPositionMsg
from ..receiver.vehicle_command_ack import VehicleCommandAckReceiver, VehicleCommandAckMsg

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, qos_profile: QoSProfile, name: str, prefix: str, target_system: int, gazebo_pos: np.ndarray, is_gazebo = False) -> None:
        super().__init__(f'offboard_control_{name}')

        self.prefix = prefix
        self.target_system = target_system

        self.gazebo_pos = self._ENU2NED(gazebo_pos)
        self.x = self.gazebo_pos[0]
        self.y = self.gazebo_pos[1]
        self.z = self.gazebo_pos[2]
        self.yaw = self.gazebo_pos[3]
        self.is_gazebo = is_gazebo

        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 0.0
        self._target_yaw = 0.0
        
        self._offboard_control_mode_publisher = OffboardControlModePublisher(self, qos_profile, prefix)
        self._trajectory_setpoint_publisher = TrajectorySetpointPublisher(self, qos_profile, prefix)
        self._vehicle_command_publisher = VehicleCommandPublisher(self, qos_profile, prefix)
        self._vehicle_visual_odometry_publisher = VehicleVisualOdometryPublisher(self, qos_profile, prefix)

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

    def force_offboard_mode(self) -> None:
        """Force the vehicle into offboard mode with proper sequence."""
        self.get_logger().info("Forcing offboard mode activation...")
        
        # Set a safe hover position
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = -3.0  # 3 meters up in NED
        self._target_yaw = 0.0
        self.flying = True
        
        # Force arm and offboard mode
        self.arm()
        self.engage_offboard_mode()

    def is_safe_for_offboard(self) -> bool:
        """Check if it's safe to engage offboard mode"""
        # Check if manual control is active
        if self.vehicle_status.nav_state == 1:  # Manual mode
            return False
        
        # Add other safety checks here if needed
        return True

    def get_control_status(self) -> dict:
        """Get current control status information"""
        return {
            'nav_state': self.vehicle_status.nav_state,
            'is_armed': self.is_armed(),
            'is_flying': self.flying,
            'offboard_mode': self.vehicle_status.nav_state == VehicleStatusEnum.NAVIGATION_STATE_OFFBOARD,
            'manual_mode': self.vehicle_status.nav_state == 1
        }

    def _ENU2NED(self, ENU_pos: np.ndarray) -> np.ndarray:
        """
        Convert ENU (East, North, Up) coordinates to NED (North, East, Down) coordinates.
        Input is a numpy array of shape (4,) representing (East, North, Up, Yaw) in ENU frame.
        Output is a numpy array of shape (4,) representing (North, East, Down, Yaw) in NED frame.
        """
        NED_pos = np.array([ENU_pos[1], ENU_pos[0], -ENU_pos[2], -ENU_pos[3]], dtype=float)
        return NED_pos

    def _NED2ENU(self, NED_pos: np.ndarray) -> np.ndarray:
        """
        Convert NED (North, East, Down) coordinates to ENU (East, North, Up) coordinates.
        Input is a numpy array of shape (4,) representing (North, East, Down, Yaw) in NED frame.
        Output is a numpy array of shape (4,) representing (East, North, Up, Yaw) in ENU frame.
        """
        ENU_pos = np.array([NED_pos[1], NED_pos[0], -NED_pos[2], -NED_pos[3]], dtype=float)
        return ENU_pos

    def _ENU_pos_check(self, pos: np.ndarray) -> bool:
        """
        Check if the given ENU position is valid.
        """
        if pos[2] < 2.0 or pos[2] > 6.0:
            print("Target Z position must be between 2.0 and 6.0 meters above the ground.")
            raise ValueError("Target Z position must be between 2.0 and 6.0 meters above the ground.")
        if abs(pos[0]) > 5.0 or abs(pos[1]) > 5.0:
            print("Target X and Y positions must be within -5.0 to 5.0 meters.")
            raise ValueError("Target X and Y positions must be within -5.0 to 5.0 meters.")
        if abs(pos[3]) > 2*math.pi:
            print("Target yaw must be between -360 and 360 degrees.")
            raise ValueError("Target yaw must be between -360 and 360 degrees.")
        
        return True

    def set_absolute_position(self, abs_pos: np.ndarray) -> None:
        """
        Set the absolute position of the drone.
        abs_pos: numpy array of shape (4,) representing in ENU frame (East, North, Up, Yaw).
        """
        # Safety check before accepting new position
        if not self.is_safe_for_offboard():
            self.get_logger().error("Cannot set position: Vehicle is in manual mode or not safe for offboard control!")
            self.get_logger().warn("Please ensure vehicle is not in manual mode before sending position commands.")
            return
        
        if not self._ENU_pos_check(abs_pos):
            return

        abs_pos = self._ENU2NED(abs_pos)

        self._target_x = abs_pos[0]
        self._target_y = abs_pos[1]
        self._target_z = abs_pos[2]
        self._target_yaw = abs_pos[3]

        self.flying = True
        self.get_logger().info(f"New position target set: x={abs_pos[0]:.2f}, y={abs_pos[1]:.2f}, z={abs_pos[2]:.2f}, yaw={abs_pos[3]:.2f}")

    def set_incremental_position(self, inc_pos: np.ndarray) -> None:
        """
        Set the incremental position of the drone.
        increment: numpy array of shape (4,) representing (dEast, dNorth, dUp, dYaw) in ENU frame.
        """
        inc_pos = self._ENU2NED(inc_pos)

        abs_pos = np.array([
            self._target_x + inc_pos[0],
            self._target_y + inc_pos[1],
            self._target_z + inc_pos[2],
            self._target_yaw + inc_pos[3]
        ])

        if not self._ENU_pos_check(self._NED2ENU(abs_pos)):
            return

        self._target_x = abs_pos[0]
        self._target_y = abs_pos[1]
        self._target_z = abs_pos[2]
        self._target_yaw = abs_pos[3]

        self.flying = True

        pos = self._NED2ENU(abs_pos)
        self.get_logger().info(f'Drone has set to x: {pos[0]}, y: {pos[1]}, z: {pos[2]}, yaw: {math.degrees(pos[3])}')

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
        
        # Publish visual odometry to provide position estimate
        if not self.is_gazebo:
            # Publish a mock visual odometry position at origin
            self._vehicle_visual_odometry_publisher.publish_pose(
                position=(0.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            )
        
        # Always publish trajectory setpoint to maintain offboard control signal
        if self.flying:
            self._keep_status()

            if self.is_gazebo:
                self._trajectory_setpoint_publisher.publish(
                    position=(self._target_x + self.gazebo_x,
                              self._target_y + self.gazebo_y,
                              self._target_z + self.gazebo_z),
                    yaw=self._target_yaw + self.gazebo_yaw
                )
            else:
                self._trajectory_setpoint_publisher.publish(
                    position=(self._target_x,
                              self._target_y,
                              self._target_z),
                    yaw=self._target_yaw
                )
        else:
            # Publish a safe hover position to maintain offboard signal
            if self.is_gazebo:
                self._trajectory_setpoint_publisher.publish(
                    position=(self.gazebo_x, self.gazebo_y, self.gazebo_z),
                    yaw=self.gazebo_yaw
                )
            else:
                # Publish a safe default position (current altitude or safe hover height)
                current_pos = self._vehicle_local_position_receiver.get_simple_msg()
                safe_z = current_pos.z if current_pos.z_valid else -3.0
                self._trajectory_setpoint_publisher.publish(
                    position=(0.0, 0.0, safe_z),
                    yaw=0.0
                )

        # Try to engage offboard mode after sending enough setpoints
        if self.offboard_setpoint_counter == 15:
            self.get_logger().info("Attempting to engage offboard mode...")
            self.engage_offboard_mode()
        
        # Continue trying to engage offboard mode if not successful
        if self.offboard_setpoint_counter > 15:
            if self.vehicle_status.nav_state != VehicleStatusEnum.NAVIGATION_STATE_OFFBOARD:
                if self.offboard_setpoint_counter % 10 == 0:  # Try every 1 second
                    self.get_logger().warn(f"Offboard mode not active (nav_state: {self.vehicle_status.nav_state}), retrying...")
                    self.engage_offboard_mode()

        if self.offboard_setpoint_counter < 100:  # Prevent overflow
            self.offboard_setpoint_counter += 1
    
    def _keep_status(self) -> None:
        """Ensure the vehicle is in offboard mode and armed."""
        # Check for manual control interference
        if self.vehicle_status.nav_state != VehicleStatusEnum.NAVIGATION_STATE_OFFBOARD:
            if self.vehicle_status.nav_state == 1:  # Manual mode
                self.get_logger().warn("Vehicle switched to manual mode! Offboard control interrupted.")
                self.get_logger().warn("Please switch back to offboard mode using RC or use 'offboard' command.")
                self.flying = False  # Stop flying to prevent conflicts
                return
            else:
                self.get_logger().info(f"Vehicle not in offboard mode (nav_state: {self.vehicle_status.nav_state}), engaging offboard mode...")
                self.engage_offboard_mode()
        
        # First, make sure vehicle is armed
        if not self.is_armed():
            self.get_logger().info("Vehicle not armed, attempting to arm...")
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