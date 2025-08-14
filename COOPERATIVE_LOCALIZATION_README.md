# Multi-Drone Cooperative Localization

基於 ROS 2 + PX4 的多無人機協同 BLE 定位系統

## 功能特色

1. **靜態 BLE 信標定位**: 使用四個靜態信標估測無人機位置
2. **協同定位**: Drone1 作為移動信標，提升 Drone2 的定位精度
3. **定位精度比較**: 自動比較靜態定位與協同定位的誤差
4. **實時位置估測**: 與實際位置進行即時比較

## 系統架構

### BLE 信標配置
- 4 個靜態信標位於場景四角：(0,0,0), (10,0,0), (0,10,0), (10,10,0)
- Drone1 可作為第5個移動信標
- 使用 log-distance 路徑損失模型模擬 RSSI

### 無人機設置  
- Drone1: `/px4_1` (目標系統 ID: 2)
- Drone2: `/px4_2` (目標系統 ID: 3)

### 座標系統
- Gazebo 世界座標系
- PX4 NED 局部座標系自動轉換

## 使用方法

### 1. 環境準備

```bash
# 確保 ROS 2 環境已設置
source /opt/ros/$ROS_DISTRO/setup.bash

# 啟動 PX4 多機 SITL (另一個終端)
# 確保 /px4_1 和 /px4_2 話題存在
```

### 2. 編譯運行

```bash
# 方法1: 使用提供的腳本
./run_demo.sh

# 方法2: 手動編譯運行
cd /home/ada/luoluo/px4_ros2_ws
colcon build --symlink-install --packages-select multi_drone_ctl
source install/setup.bash
python3 src/multi_drone_ctl/multi_drone_ctl/drone_ctl.py 

```

### 3. 操作命令

| 命令 | 說明 |
|------|------|
| `arm` | 解鎖所有無人機 |
| `set` | 設定特定無人機位置 |
| `est` | 使用 BLE 信標估測位置 |
| `coop` | 協同定位功能 |
| `land` | 降落所有無人機 |
| `disarm` | 上鎖所有無人機 |
| `quit` | 退出程式 |

### 4. 協同定位演示

1. 解鎖無人機: `arm`
2. 設置初始位置: `set` (例如讓兩架無人機分別飛到不同位置)
3. 執行協同定位: `coop`

系統會：
- 先使用靜態信標估測 Drone1 位置  
- 將 Drone1 設為移動信標
- 使用靜態+移動信標估測 Drone2 位置
- 比較只用靜態信標 vs 協同定位的精度

## 測試結果範例

```
=== 開始協同定位 ===

步驟1：估測 Drone1 位置（使用靜態 BLE 信標）
Current Position for Drone 1: x=3.00, y=4.00, z=-2.00
Estimated Position for Drone 1: x=2.85, y=4.12, z=-1.95
Position Error for Drone 1: x=0.15m, y=0.12m, z=0.05m, total=0.20m

步驟2：估測 Drone2 位置（使用靜態信標 + Drone1 移動信標）
Drone2 實際位置: x=7.00, y=6.00, z=-3.00
Drone2 估測位置（僅靜態信標）: x=6.82, y=5.95, z=-2.88
Drone2 估測位置（協同定位）: x=7.05, y=6.02, z=-2.97

=== 定位精度比較 ===
僅靜態信標誤差: 0.22m
協同定位誤差: 0.06m
精度改善: 72.7%
```

## 技術特點

### BLE 定位模型
```python
rssi = rssi0 - 10 * path_loss_n * log10(distance) + noise
```
- `rssi0`: 1米參考 RSSI (-50 dBm)
- `path_loss_n`: 路徑損失指數 (2.0)
- `noise_stddev`: 雜訊標準差 (1.0 dBm)

### 協同定位演算法
1. 最小二乘法優化 RSSI 殘差
2. 動態更新移動信標位置
3. 多信標融合定位

### 安全機制
- 限制位置設定點跳變
- 優雅關閉與錯誤處理
- 自動降落與解除武裝

## 檔案結構

```
multi_drone_ctl/
├── multi_drone_ctl/
│   ├── drone_ctl.py          # 主程式
│   ├── config.yaml           # 配置文件  
│   └── publisher/            # 發布者模組
├── gazebo/
│   ├── ble.world            # Gazebo 世界檔案
│   └── run.sh               # Gazebo 啟動腳本
├── test_cooperative_localization.py  # 協同定位測試
├── run_demo.sh              # 一鍵運行腳本
└── README.md                # 本說明文件
```

## 故障排除

### 常見問題

1. **無法找到 px4_msgs**
   - 確保已安裝 PX4 和 px4_msgs 包
   - 檢查 ROS 2 環境設置

2. **無人機不響應**  
   - 檢查 PX4 SITL 是否正常運行
   - 確認 `/px4_1`, `/px4_2` 話題存在

3. **定位精度差**
   - 檢查無人機是否在信標覆蓋範圍內
   - 調整 RSSI 模型參數

### 調試模式

```bash
# 啟用詳細日誌
export RCUTILS_LOGGING_SEVERITY=DEBUG
python3 -m multi_drone_ctl.drone_ctl
```

## 參數調整

### BLE 模型參數 (`drone_ctl.py`)

```python
RSSI_SETTINGS = {
    'rssi0': -50,        # 1米參考 RSSI
    'path_loss_n': 2.0,  # 路徑損失指數
    'noise_stddev': 1.0  # 雜訊標準差
}
```

### 信標位置
可在 `MultiDroneController.__init__()` 中修改靜態信標位置

## 擴展功能

- [ ] 視覺輔助定位 (ArUco 標記)
- [ ] 卡爾曼濾波器融合
- [ ] 更多無人機支援
- [ ] 動態路徑規劃
- [ ] Web 介面監控

## 授權

本專案遵循 MIT 授權條款
