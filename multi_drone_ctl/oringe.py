import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from publisher.offboard_control_mode_publisher import OffboardControlModePublisher
from publisher.trajectory_setpoint_publisher import TrajectorySetpointPublisher
from publisher.vehicle_command_publisher import VehicleCommandPublisher, VehicleCommandConsts

from receiver.failsafe_flags_receiver import FailsafeFlagsReceiver, FailsafeFlagsMsg

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class Controller(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, name: str, prefix: str, target_system: int, gazebo_position: tuple) -> None:
        super().__init__(f'controller_{name}')

        self.prefix = prefix
        self.target_system = target_system
        self.gazebo_x, self.gazebo_y, self.gazebo_z = gazebo_position
        self.get_logger().warn(f'{prefix} + {target_system}')

        self._offboard_control_mode_publisher = OffboardControlModePublisher(self, qos_profile)
        self._trajectory_setpoint_publisher = TrajectorySetpointPublisher(self, qos_profile)
        self._vehicle_command_publisher = VehicleCommandPublisher(self, qos_profile)

        self._vehicle_command_publisher.publish_command(
            command_id=VehicleCommandConsts.VEHICLE_CMD_COMPONENT_ARM_DISARM
        )

        self._failsafe_flags_receiver = FailsafeFlagsReceiver(self, qos_profile)
        self._offboard_control_mode_publisher.publish_command()
    
    def arm(self):
        self._vehicle_command_publisher.publish_command(
            command_id=VehicleCommandConsts.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=VehicleCommandConsts.ARMING_ACTION_ARM,
            target_system=1,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True
        )
    
    def get_offboard_control_mode_publisher(self):
        return self._offboard_control_mode_publisher
    
    def get_trajectory_setpoint_publisher(self):
        return self._trajectory_setpoint_publisher
    
    def get_vehicle_command_publisher(self):
        return self._vehicle_command_publisher
    
    def get_failsafe_flags_receiver(self):
        return self._failsafe_flags_receiver

def failsafe_flags_callback(data: FailsafeFlagsMsg):
    offboard_control_signal_lost = data.offboard_control_signal_lost
    print(f"Offboard Control Signal Lost: {offboard_control_signal_lost}")

def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    controller = Controller(name = 'test_drone', \
                            prefix= '', \
                            target_system=0, \
                            gazebo_position=(0.0, 0.0, 0.0))

    controller.get_failsafe_flags_receiver().add_callback(failsafe_flags_callback)

    executor.add_node(controller)
    executor.spin()

    while rclpy.ok():
        try:
            controller.get_offboard_control_mode_publisher().publish_command(position=True)

            controller.get_vehicle_command_publisher().publish_command(
                command_id=VehicleCommandConsts.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=VehicleCommandConsts.ARMING_ACTION_ARM
            )

            controller.get_trajectory_setpoint_publisher().publish_command(
                position=(0.0, 0.0, -5.0),
                yaw=1.57079
            )
        except KeyboardInterrupt:
            break
    
    while not rclpy.ok():
        pass

    print('trying to arming...')
    controller.arm()
    print('arming done.')

    while rclpy.ok():
        pass

if __name__ == '__main__':
    main()