#!/usr/bin/env bash
set -e

# # 1) RGB 影像 -> /camera/image
# ros2 run ros_gz_bridge parameter_bridge \
#   /world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image \
#   --ros-args -r /world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/image &

# # 2) CameraInfo -> /camera/camera_info
# ros2 run ros_gz_bridge parameter_bridge \
#   /world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
#   --ros-args -r /world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info:=/camera/camera_info &

# # 3) GZ 點雲 -> /camera/points
# ros2 run ros_gz_bridge parameter_bridge \
#   /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
#   --ros-args -r /depth_camera/points:=/camera/points &

# 1) RGB 影像 → /camera/image
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image:=/camera/image &

# 2) CameraInfo → /camera/camera_info
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args -r /world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info:=/camera/camera_info &

# 3)（已經有）GZ 點雲 → /camera/points ；這條不用改
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  --ros-args -r /depth_camera/points:=/camera/points &

# 4) 建議把模擬時鐘也橋來（RViz 用）
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  --ros-args -r /world/default/clock:=/clock &

wait
