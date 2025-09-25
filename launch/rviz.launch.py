from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_share = FindPackageShare('buav').find('buav')
    rviz_config = os.path.join(pkg_share, 'rviz', 'point_image.rviz')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='default',
        description='Name of the Gazebo world'
    )
    
    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='x500_depth_0',
        description='Name of the drone model'
    )

    world_name = LaunchConfiguration('world_name')
    drone_name = LaunchConfiguration('drone_name')

    return LaunchDescription([
        world_name_arg,
        drone_name_arg,
        
        # RGB Image
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[["/world/", world_name, "/model/", drone_name, "/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image"]],
            remappings=[(["/world/", world_name, "/model/", drone_name, "/link/camera_link/sensor/IMX214/image"], "/camera/image")]
        ),
        # CameraInfo
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[["/world/", world_name, "/model/", drone_name, "/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"]],
            remappings=[(["/world/", world_name, "/model/", drone_name, "/link/camera_link/sensor/IMX214/camera_info"], "/camera/camera_info")]
        ),
        # PointCloud
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/depth_camera/points@"
                       "sensor_msgs/msg/PointCloud2@"
                       "gz.msgs.PointCloudPacked"],
            remappings=[("/depth_camera/points", "/camera/points")],
            output="screen"
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        Node(
            package="buav",
            executable="vehicle_odom_bridge",
            name="vehicle_odom_bridge",
            output="screen"
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", [drone_name, "/camera_link/StereoOV7251"]]
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
