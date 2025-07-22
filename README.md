colcon build --packages-select multi_drone_ctl

source install/setup.bash

ros2 run multi_drone_ctl drone_ctl 
