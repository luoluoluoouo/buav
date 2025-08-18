#!/bin/sh

get_root_back() {
    path="$1"
    # 去掉開頭的 "/" 避免第一個空元素
    clean=$(echo "$path" | sed 's:^/::')
    # 用 "/" 切開計算有幾層
    depth=$(echo "$clean" | awk -F/ '{print NF}')

    out="/"
    i=0
    while [ "$i" -lt "$depth" ]; do
        out="../$out"
        i=$((i + 1))
    done

    echo "$out"
}

kill -9 $(lsof -t -i :11345)
kill -9 $(lsof -t -i :8888)

MicroXRCEAgent udp4 -p 8888 &

kill -9 $(lsof -t -i :11345)

# 取得腳本所在資料夾的絕對路徑
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Setting AutoPilot path
export PX4_AUTOPILOT_PATH=/home/ada/luoluo/PX4-Autopilot

$PX4_AUTOPILOT_PATH/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 2 -w $(get_root_back "$PX4_AUTOPILOT_PATH/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds")$SCRIPT_DIR/ble -s iris:2 -t px4_sitl_default

kill -9 $(lsof -t -i :11345)
kill -9 $(lsof -t -i :8888)