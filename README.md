## BUAV（BLE-based UAV Localization and Scanning）

ROS 2 + PX4 多無人機 Offboard 控制與簡易 BLE 信標模擬。內建 Gazebo（GZ）SITL 腳本，可啟動兩台 PX4 實例（`/px4_1`, `/px4_2`）。

## 環境清單

- Ubuntu：22.04
- ROS 2：humble
- PX4-Autopilot：v1.16
- px4_msgs：release/1.16

## 快速開始

- ROS 2 工具鏈：ros2, colcon
- PX4-Autopilot（已完成 SITL 編譯）
- Micro-XRCE-DDS-Agent（可直接執行 `MicroXRCEAgent`）
- px4_msgs（請放在工作區 `src/` 內並建置）
- Python 依賴：numpy、scipy

安裝 PX4-Autopilot（若尚未完成建制）

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.16.0
make px4_sitl gz_x500
```

安裝 px4_msgs（若尚未加入工作區）

```bash
cd your_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.16
cd ../.. && colcon build --packages-up-to px4_msgs
source install/setup.bash
```

建置本套件

```bash
cd your_ws
colcon build --packages-select buav --symlink-install
source install/setup.bash
```

設定與啟動模擬（Gazebo + 兩架 PX4）

1) 編輯 `buav/gazebo/run.sh` 將 `PX4_AUTOPILOT_PATH` 改成你的 PX4-Autopilot 路徑。
2) 執行：

```bash
cd your_ws/src/buav/gazebo
bash run.sh
```

腳本會啟動 Micro XRCE Agent（UDP 8888）與兩個 PX4 實例。當 `/px4_1`、`/px4_2` 話題出現即可使用。

## 簡單使用（互動命令）
啟動 MicroXRCEAgent

- 模擬
```bash
MicroXRCEAgent udp4 -p 8888
```
- 實機
```bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 115200
```

啟動控制節點
- 單機或實機（/fmu/...）
```bash
ros2 run buav main
```

- 雙機（/px4_1/fmu/..., /px4_2/fmu/...）
```bash
ros2 run buav sim
```

可用命令

- arm：解鎖所有無人機
- abs：對指定無人機設定「絕對位置」
- inc：對指定無人機設定「增量位移」
- land：所有無人機降落
- disarm：上鎖所有無人機
- quit：離開程式

輸入座標說明

- 互動命令的 x, y, z, yaw 以 ENU（East, North, Up, Yaw）輸入；yaw 以「度」輸入，程式會轉為弧度。
- 內部會自動轉換為 PX4 NED（North, East, Down）來發送軌跡設定點。
- 安全限制（於 `OffboardControl._ENU_pos_check`）
	- z（Up）需介於 2.0–6.0 公尺
	- x、y 需在 ±5.0 公尺內
	- yaw 需在 ±360 度內

## 專案結構與用途

- `buav/main.py`
	- 互動式 CLI 與執行入口（entry points：`main`、`sim`）。
	- 建立 `MultiDroneController`，將各 `OffboardControl` 節點加入多執行緒 executor。
	- 重要指令函數：
		- `cmd_arm/controller`：解鎖所有無人機
		- `cmd_disarm/controller`：上鎖所有無人機
		- `cmd_land/controller`：降落所有無人機
		- `cmd_abs/controller`：設定指定無人機的絕對 ENU 位置（x,y,z,yaw）
		- `cmd_inc/controller`：設定指定無人機的增量 ENU 位移（dx,dy,dz,dyaw）

- `buav/node_models/offboard_control.py`
	- 核心控制節點 `OffboardControl`（每台無人機一個節點）。
	- 重要方法：
		- `arm()` / `disarm()` / `land()` / `engage_offboard_mode()`
		- `set_absolute_position(abs_pos: np.ndarray)`：接受 ENU [E,N,U,yaw] 絕對位置
		- `set_incremental_position(inc_pos: np.ndarray)`：接受 ENU [dE,dN,dU,dyaw] 增量
		- `_timer_callback()`：固定頻率送出 Offboard 心跳與 TrajectorySetpoint
		- `_keep_status()`：確保處於 Offboard 且已上鎖/解鎖狀態正確
		- `_ENU2NED()` / `_NED2ENU()`：座標系轉換
		- `_ENU_pos_check()`：輸入限制檢查
	- 主要話題：
		- 發布：`{prefix}/fmu/in/offboard_control_mode`、`{prefix}/fmu/in/trajectory_setpoint`、`{prefix}/fmu/in/vehicle_command`
		- 訂閱：`{prefix}/fmu/out/vehicle_local_position`、`{prefix}/fmu/out/vehicle_status_v1`

- `buav/node_models/ble_beacon.py`
	- BLE 信標模擬節點 `BLEbeacon`，用 log-distance 模型產生 RSSI。
	- 重要函數：`get_rssi(drone_position)` 回傳含雜訊的 RSSI。

- `buav/publisher/`
	- `OffboardControlModePublisher.heartbeat()`：定期送 position 模式的 offboard 心跳。
	- `TrajectorySetpointPublisher.publish(position, yaw, ...)`：送位置/速度/加速度/jerk/yaw 等設定點。
	- `VehicleCommandPublisher.publish(...)`：送 arm/disarm、set mode、land 等 PX4 VehicleCommand。
	- `VehicleCommandEnum`：常用命令與參數（如 `VEHICLE_CMD_COMPONENT_ARM_DISARM`）。

- `buav/receiver/`
	- `VehicleLocalPositionReceiver`：訂閱 `vehicle_local_position`，支援 `add_callback()`、`get_simple_msg()`。
	- `VehicleStatusReceiver`：訂閱 `vehicle_status_v1`，支援 `add_callback()`。
	- `VehicleCommandAckReceiver`：訂閱命令回覆（可用於除錯）。

- `gazebo/run.sh`
	- 啟動 Micro XRCE Agent（UDP 8888）與兩個 PX4 實例（`gz_x500`/`gz_x500_depth`）。
	- 可依需求修改初始位姿、TF 與橋接設定。

- `launch/rviz.launch.py`
	- 透過 `ros_gz_bridge` 橋接影像/點雲，載入 `rviz/point_image.rviz`。。

## 常用話題一覽

- 發布
	- `{prefix}/fmu/in/offboard_control_mode`（px4_msgs/OffboardControlMode）
	- `{prefix}/fmu/in/trajectory_setpoint`（px4_msgs/TrajectorySetpoint）
	- `{prefix}/fmu/in/vehicle_command`（px4_msgs/VehicleCommand）
- 訂閱
	- `{prefix}/fmu/out/vehicle_local_position`（px4_msgs/VehicleLocalPosition）
	- `{prefix}/fmu/out/vehicle_status_v1`（px4_msgs/VehicleStatus）
	- `{prefix}/fmu/out/vehicle_command_ack`（px4_msgs/VehicleCommandAck）

註：`prefix` 於雙機 SITL 時為 `/px4_1`、`/px4_2`；單機（無前綴）時為空字串。

## 疑難排解

- 找不到 `px4_msgs`：請先將套件放入工作區並建置，再 `source install/setup.bash`。
- 埠占用/Agent 問題：`MicroXRCEAgent udp4 -p 8888` 是否已啟動？或修改 `run.sh` 的埠。
- 無法進入 Offboard：請確認心跳持續送出、且機體已 ARM；`_keep_status()` 會自動嘗試設置。
- 無 ROS 話題：請先執行 `gazebo/run.sh`，等待 PX4 與 Agent 建立連線。

## 開發小提醒

- 入口點由 `setup.py` 註冊：`ros2 run buav main`、`ros2 run buav sim`。
- 若要客製多機/前綴/起始位姿，可在 `main.py` 的 `OffboardControl(...)` 初始化區塊調整。
