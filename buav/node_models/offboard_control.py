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

    def __init__(self, qos_profile: QoSProfile, \
                       name: str, 
                       prefix: str, 
                       target_system: int, 
                       is_gazebo = False,
                       gazebo_enu_pos: list = None,
                       scale: float = 1.0
                       ) -> None:
        super().__init__(f'offboard_control_{name}')

        self.prefix = prefix
        self.target_system = target_system

        self.is_gazebo = is_gazebo
        if is_gazebo:
            self.gazebo_enu_pos = gazebo_enu_pos
            self.gazebo_ned_pos = self._ENU2NED(gazebo_enu_pos)
            self._target_x = self.gazebo_ned_pos[0]
            self._target_y = self.gazebo_ned_pos[1]
            self._target_z = self.gazebo_ned_pos[2]
            self._target_yaw = self.gazebo_ned_pos[3]
        else:
            self.gazebo_enu_pos = [0.0, 0.0, 0.0, 0.0]
            self.gazebo_ned_pos = [0.0, 0.0, 0.0, 0.0]
            self._target_x = 0.0
            self._target_y = 0.0
            self._target_z = 0.0
            self._target_yaw = 0.0
        self.scale = scale

        self._offboard_control_mode_publisher = OffboardControlModePublisher(self, qos_profile, prefix)
        self._trajectory_setpoint_publisher = TrajectorySetpointPublisher(self, qos_profile, prefix)
        self._vehicle_command_publisher = VehicleCommandPublisher(self, qos_profile, prefix)

        self._vehicle_local_position_receiver = VehicleLocalPositionReceiver(self, qos_profile, prefix)
        self._vehicle_local_position_receiver.add_callback(self._vehicle_local_position_callback)
        
        self._vehicle_status_receiver = VehicleStatusReceiver(self, qos_profile, prefix)
        self._vehicle_status_receiver.add_callback(self._vehicle_status_callback)

        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = self._vehicle_local_position_receiver.get_simple_msg()
        self.vehicle_local_enu_pos = [0.0, 0.0, 0.0, 0.0]
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

    def _ENU2NED(self, ENU_pos: list) -> list:
        """
        Convert ENU (East, North, Up) coordinates to NED (North, East, Down) coordinates.
        Input is a list of shape (4,) representing (East, North, Up, Yaw) in ENU frame.
        Output is a list of shape (4,) representing (North, East, Down, Yaw) in NED frame.
        """
        NED_pos = [ENU_pos[1], ENU_pos[0], -ENU_pos[2], -ENU_pos[3]]
        return NED_pos

    def _NED2ENU(self, NED_pos: list) -> list:
        """
        Convert NED (North, East, Down) coordinates to ENU (East, North, Up) coordinates.
        Input is a list of shape (4,) representing (North, East, Down, Yaw) in NED frame.
        Output is a list of shape (4,) representing (East, North, Up, Yaw) in ENU frame.
        """
        ENU_pos = [NED_pos[1], NED_pos[0], -NED_pos[2], -NED_pos[3]]
        return ENU_pos

    def _ENU_pos_check(self, enu_pos: list) -> bool:
        """
        Check if the given ENU position is valid.
        """
        if enu_pos[2] < 2.0 or enu_pos[2] > 5.0:
            print("Target Z position must be between 2.0 and 5.0 meters above the ground.")
            raise ValueError("Target Z position must be between 2.0 and 5.0 meters above the ground.")
        if abs(enu_pos[0]) > 5.0 or abs(enu_pos[1]) > 5.0:
            print("Target X and Y positions must be within -5.0 to 5.0 meters.")
            raise ValueError("Target X and Y positions must be within -5.0 to 5.0 meters.")
        if enu_pos[3] > 2*math.pi:
            enu_pos[3] = enu_pos[3] % (2*math.pi)
        if enu_pos[3] < -2*math.pi:
            enu_pos[3] = enu_pos[3] % (-2*math.pi)

        return True

    def set_absolute_position(self, abs_pos: list) -> None:
        """
        Set the absolute position of the drone.
        abs_pos: list of shape (4,) representing in ENU frame (East, North, Up, Yaw).
        """
        if not self._ENU_pos_check(abs_pos):
            return

        abs_pos = self._ENU2NED(abs_pos)

        self._target_x = abs_pos[0]
        self._target_y = abs_pos[1]
        self._target_z = abs_pos[2]
        self._target_yaw = abs_pos[3]

        self.flying = True

    def set_incremental_position(self, inc_pos: list) -> None:
        """
        Set the incremental position of the drone.
        increment: list of shape (4,) representing (dEast, dNorth, dUp, dYaw) in ENU frame.
        """
        inc_pos = self._ENU2NED(inc_pos)

        abs_pos = [
            self._target_x + inc_pos[0],
            self._target_y + inc_pos[1],
            self._target_z + inc_pos[2],
            self._target_yaw + inc_pos[3]
        ]

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
        self.abs_x = self.gazebo_enu_pos[0] + self.vehicle_local_enu_pos[0]
        self.abs_y = self.gazebo_enu_pos[1] + self.vehicle_local_enu_pos[1]
        self.abs_z = self.gazebo_enu_pos[2] + self.vehicle_local_enu_pos[2]

        return (self.abs_x, self.abs_y, self.abs_z)

    def is_armed(self) -> bool:
        return self.vehicle_status.arming_state == VehicleStatusEnum.ARMING_STATE_ARMED
    
    def _vehicle_local_position_callback(self, msg: VehicleLocalPositionMsg) -> None:
        self.vehicle_local_position = msg
        vehicle_local_ned_pos = [
            self.vehicle_local_position.x,
            self.vehicle_local_position.y,
            self.vehicle_local_position.z,
            -1
        ]

        self.vehicle_local_enu_pos = self._NED2ENU(vehicle_local_ned_pos)

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

            if self.is_gazebo:
                x = self.scale * (self._target_x - self.gazebo_ned_pos[0])
                y = self.scale * (self._target_y - self.gazebo_ned_pos[1])
                z = self._target_z - self.gazebo_ned_pos[2]
                self._trajectory_setpoint_publisher.publish(
                    position=(x, y, z),
                    yaw=self._target_yaw - self.gazebo_ned_pos[3]
                )
            else:
                x = self.scale * self._target_x
                y = self.scale * self._target_y
                z = self._target_z
                self._trajectory_setpoint_publisher.publish(
                    position=(x, y, z),
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