#!/bin/bash

kill -9 $(lsof -t -i :11345)
kill -9 $(lsof -t -i :8888)

MicroXRCEAgent udp4 -p 8888 &

export PX4_AUTOPILOT_PATH=/home/ada/luoluo/PX4-Autopilot

$PX4_AUTOPILOT_PATH/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 2 -w ../../../../../../../../../../../../../../../../../../../../../../../$(pwd)/ble