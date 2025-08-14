#!/bin/sh

MicroXRCEAgent udp4 -p 8888 &

kill -9 $(lsof -t -i :11345)

# 取得腳本所在資料夾的絕對路徑
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Setting AutoPilot path
export PX4_AUTOPILOT_PATH=/home/ubuntu/PX4-Autopilot

$PX4_AUTOPILOT_PATH/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 2 -w ../../../../../../../../../../../../../../../../../../../../../../../$SCRIPT_DIR/ble