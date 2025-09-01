from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('buav').find('buav')
    rviz_config = os.path.join(pkg_share, 'rviz', 'point_image.rviz')
    drone_id = "x500_depth_0"

    return LaunchDescription([
        # RGB Image
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"/world/walls/model/{drone_id}/link/camera_link/sensor/IMX214/image"
                       "@sensor_msgs/msg/Image@gz.msgs.Image"],
            remappings=[(f"/world/walls/model/{drone_id}/link/camera_link/sensor/IMX214/image",
                         "/camera/image")]
        ),
        # CameraInfo
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"/world/walls/model/{drone_id}/link/camera_link/sensor/IMX214/camera_info"
                       "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
            remappings=[(f"/world/walls/model/{drone_id}/link/camera_link/sensor/IMX214/camera_info",
                         "/camera/camera_info")]
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
        # Static TF
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", f"{drone_id}/camera_link/StereoOV7251"]
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
