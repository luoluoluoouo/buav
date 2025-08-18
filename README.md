## multi_drone_ctl

ROS 2 node for PX4 multi-drone Offboard control with a simple RSSI beacon simulator. Includes a Gazebo Classic SITL script to launch two PX4 instances (`/px4_1`, `/px4_2`).

## 環境建置

Prerequisites (Ubuntu + ROS 2):

- ROS 2 (Humble 建議) 與基礎工具：colcon, ament
- PX4-Autopilot 已編譯之 SITL 與 Gazebo Classic 支援
- Micro-XRCE-DDS-Agent 安裝並可直接執行 `MicroXRCEAgent`
- px4_msgs 介面套件（從原始碼放在工作區 `src/` 底下）：
	- 來源：https://github.com/PX4/px4_msgs.git
	- 若尚未放入工作區，範例步驟：
		```bash
		cd ~/px4_ros2_ws/src
		git clone https://github.com/PX4/px4_msgs.git
		cd ..
		colcon build --packages-up-to px4_msgs
		source install/setup.bash
		```
- Python 依賴：numpy、scipy（ROS 2 Python 節點使用）
	- `sudo apt install python3-numpy python3-scipy` 或以 pip 安裝

專案位置與建置：

1) 將此套件放在你的 ROS 2 工作區（例如：`~/px4_ros2_ws/src/`）
2) 在工作區根目錄執行建置與覆蓋環境：

```bash
cd ~/px4_ros2_ws
colcon build --packages-select multi_drone_ctl
source install/setup.bash
```

PX4 與 Gazebo 準備：

- 編輯 `gazebo/run.sh` 的 `PX4_AUTOPILOT_PATH`，改成你本機 PX4-Autopilot 路徑。
- 該腳本會：
	- 啟動 Micro XRCE Agent（UDP: 8888）
	- 以 Gazebo Classic 啟動兩台 PX4 SITL（`iris` 機型），並載入 `gazebo/ble.world`

啟動模擬：

```bash
cd ~/px4_ros2_ws/src/multi_drone_ctl/gazebo
bash run.sh
```

看到每個 PX4 實例出現類似「XRCE session created」訊息後，代表與 Agent 連線完成（對應的 ROS 主題前綴為 `/px4_1` 與 `/px4_2`）。

## 使用方式

在另一個終端執行 ROS 2 節點（記得先 source 工作區）：

```bash
cd ~/px4_ros2_ws
source install/setup.bash
ros2 run multi_drone_ctl main
```

啟動後進入互動模式：

1) 輸入 drone ID：`0` 或 `1`（分別對應 `/px4_1`、`/px4_2`）
2) 輸入指令（小寫）：
	 - `arm`：解鎖所有無人機
	 - `set`：設定單一無人機的目標位置（依序輸入 x, y, z）
	 - `disarm`：上鎖所有無人機
	 - `land`：所有無人機降落
	 - `est`：列印所選無人機的即時位置與 RSSI 估測位置

注意：此專案採用 PX4 NED 座標系，z 向上為負值（例如：高度約 5 公尺可輸入 z = -5）。

## 設定與話題

- 目前飛行器與信標參數在 `multi_drone_ctl/drone_ctl.py` 中硬編（`/px4_1` 與 `/px4_2` 各一台）。
- `multi_drone_ctl/config.yaml` 提供參考配置（尚未在程式中載入）。
- 主要訂閱話題（每台機器人會含前綴 `/px4_X`）：
	- `{prefix}/fmu/out/vehicle_local_position`
	- `{prefix}/fmu/out/vehicle_status_v1`
- 主要發布話題（經由 offboard/trajectory/command 封裝類別）：
	- OffboardControlMode、TrajectorySetpoint、VehicleCommand

## 疑難排解

- 找不到 `px4_msgs`：請確認已在工作區的 `src/` 內 clone `https://github.com/PX4/px4_msgs.git`，然後重新建置並 source：
	```bash
	cd ~/px4_ros2_ws
	colcon build --packages-up-to px4_msgs
	source install/setup.bash
	```
- 無法連到 Agent 或埠占用：確認 `MicroXRCEAgent udp4 -p 8888` 正常運行或更換埠號。
- Gazebo/SITL 未啟動：檢查 `PX4_AUTOPILOT_PATH` 是否正確、PX4 SITL 是否已完成編譯。
- 無 ROS 主題：請先啟動 `run.sh`，等待 PX4 連上 XRCE Agent 後再執行節點。

## 測試與開發
sudo lsof -i :8888
- 建置完成後可執行基本測試工具（flake8、pep257）透過 `colcon test`。
- 本套件入口點：`drone_ctl`（對應 `multi_drone_ctl/drone_ctl.py:main`）。

—

如需客製無人機與信標數量/位置，請修改 `drone_ctl.py` 中的初始化邏輯；若要改為讀取 `config.yaml`，可擴充在 `main()` 建立控制器前載入並套用設定。
