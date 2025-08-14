!#/bin/bash

# Setting AutoPilot path
export PX4_AUTOPILOT_PATH=/home/ada/luoluo/PX4-Autopilot

$PX4_AUTOPILOT_PATH/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 2 -w ../../../../../../../../../../../../../../../../../../../../../../../$(pwd)/ble