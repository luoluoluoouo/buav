from rclpy.node import Node
from rclpy.qos import QoSProfile

from ..publisher.offboard_control_mode_publisher import OffboardControlModePublisher
from ..publisher.trajectory_setpoint_publisher import TrajectorySetpointPublisher
from ..publisher.vehicle_command_publisher import VehicleCommandPublisher, VehicleCommandEnum

from ..receiver.vehicle_local_position_receiver import VehicleLocalPositionReceiver, VehicleLocalPositionMsg
from ..receiver.vehicle_status_receiver import VehicleStatusReceiver, VehicleStatusMsg

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
        
        self._offboard_control_mode_publisher = OffboardControlModePublisher(self, qos_profile, prefix)
        self._trajectory_setpoint_publisher = TrajectorySetpointPublisher(self, qos_profile, prefix)
        self._vehicle_command_publisher = VehicleCommandPublisher(self, qos_profile, prefix)

        self._vehicle_local_position_receiver = VehicleLocalPositionReceiver(self, qos_profile, prefix)
        self._vehicle_local_position_receiver.add_callback(self.vehicle_local_position_callback)
        
        self._vehicle_status_receiver = VehicleStatusReceiver(self, qos_profile, prefix)
        self._vehicle_status_receiver.add_callback(self.vehicle_status_callback)

        self.offboard_setpoint_counter = 0
        self.vehicle_status = self._vehicle_status_receiver.get_simple_msg()
        self.takeoff_height = -5.0

        self.timer = self.create_timer(0.05, self._timer_callback)
        
        self.get_logger().warn(f'{prefix} + {target_system}')

    def vehicle_local_position_callback(self, msg: VehicleLocalPositionMsg):
        if msg is not None:
            self.vehicle_local_position = msg
            self.x = self.vehicle_local_position.x
            self.y = self.vehicle_local_position.y
            self.z = self.vehicle_local_position.z
        else:
            self.get_logger().warn(f'No data received from {self.prefix}/fmu/out/vehicle_local_position')


    def vehicle_status_callback(self, msg: VehicleStatusMsg):
        self.vehicle_status = msg

    def arm(self):
        self._vehicle_command_publisher.publish(
            command_id=VehicleCommandEnum.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=VehicleCommandEnum.ARMING_ACTION_ARM,
            target_system=self.target_system,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )

    def disarm(self):
        self._vehicle_command_publisher.publish(
            command_id=VehicleCommandEnum.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=VehicleCommandEnum.ARMING_ACTION_DISARM,
            target_system=self.target_system,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )

    def engage_offboard_mode(self):
        self._vehicle_command_publisher.publish(
            command_id=VehicleCommandEnum.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
            target_system=self.target_system,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )

    def land(self):
        self._vehicle_command_publisher.publish(
            command_id=VehicleCommandEnum.VEHICLE_CMD_NAV_LAND,
            target_system=self.target_system,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )

    def set_position(self, target_position: tuple[float, float, float]):
        target_x, target_y, target_z = target_position
        x = target_y - self.gazebo_y
        y = target_x - self.gazebo_x
        z = self.gazebo_z - target_z
        
        self._trajectory_setpoint_publisher.publish(
            position=(x, y, z),
            yaw=0.0,
        )

    def get_position(self):
        self.abs_x = self.gazebo_x + self.vehicle_local_position.y
        self.abs_y = self.gazebo_y + self.vehicle_local_position.x
        self.abs_z = self.gazebo_z - self.vehicle_local_position.z
        return (self.abs_x, self.abs_y, self.abs_z)
    
    def heartbeat(self):
        self._offboard_control_mode_publisher.publish(position=True)
    
    def _timer_callback(self):
        self.heartbeat()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
    
    def get_offboard_control_mode_publisher(self):
        return self._offboard_control_mode_publisher
    
    def get_trajectory_setpoint_publisher(self):
        return self._trajectory_setpoint_publisher
    
    def get_vehicle_command_publisher(self):
        return self._vehicle_command_publisher
    
    def get_vehicle_local_position_receiver(self):
        return self._vehicle_local_position_receiver
    
    def get_vehicle_status_receiver(self):
        return self._vehicle_status_receiver