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

# 清理函數
cleanup() {
    echo "正在清理進程..."
    kill -9 $(lsof -t -i :11345) 2>/dev/null
    kill -9 $(lsof -t -i :8888) 2>/dev/null
    pkill -f 'gz sim|ign gazebo|gazebo' 2>/dev/null
    # 殺掉所有背景進程
    jobs -p | xargs -r kill -9
    exit 0
}


# pkill -f 'gz sim|ign gazebo|gazebo' 2>/dev/null

# 捕獲 SIGINT (Ctrl+C) 和 SIGTERM
trap cleanup INT TERM

# kill -9 $(lsof -t -i :11345) 2>/dev/null
kill -9 $(lsof -t -i :8888) 2>/dev/null

# 取得腳本所在資料夾的絕對路徑
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

cd "$SCRIPT_DIR"
export $(grep -v '^#' .env | xargs) #PX4_AUTOPILOT_PATH=/your/path/to/PX4-Autopilot

MicroXRCEAgent udp4 -p 8888 &

python3 send_heartbeat.py &

sleep 1

# PX4_GZ_WORLD=../../../../../px4_ros2_ws/src/buav/gazebo/ble \
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE="0,3" \
$PX4_AUTOPILOT_PATH/build/px4_sitl_default/bin/px4 -i 1 &
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,-3" \
$PX4_AUTOPILOT_PATH/build/px4_sitl_default/bin/px4 -i 2 &

ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0 map 'x500_depth_1/camera_link/StereoOV7251' &

# PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,0" \
# $PX4_AUTOPILOT_PATH/build/px4_sitl_default/bin/px4 -i 0 &

# 等待信號
echo "按 Ctrl+C 停止..."
wait