#!/usr/bin/env python3
import socket, time, threading
from pymavlink import mavutil

GCS_SYSID = 255     # QGC 也常用 255
GCS_COMPID = 190    # MAV_COMP_ID_MISSIONPLANNER (任一 GCS 類型皆可)

listen_addr = ("0.0.0.0", 14550)   # GCS 必須聽 14550（PX4 的 GCS 連線埠）
print(f"[GCS] listening on {listen_addr[0]}:{listen_addr[1]} for PX4 peers...")

# 1) 用原生 socket 抓 PX4 來源 (ip, port)；不解析 MAVLink，只是拿位址
rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
rx_sock.bind(listen_addr)
rx_sock.settimeout(0.2)

# 2) 每個已知 PX4 對端各自維護一條 udpout 連線來送 HEARTBEAT
peers = {}   # (ip, port) -> mavutil connection
lock  = threading.Lock()

def hb_loop():
    while True:
        with lock:
            conns = list(peers.values())
        for c in conns:
            try:
                # 以 GCS 身分送 HEARTBEAT（1 Hz 以上）
                c.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,  # 設置適當的模式標誌
                    0, 
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
                # 也發送系統狀態
                c.mav.sys_status_send(
                    onboard_control_sensors_present=0,
                    onboard_control_sensors_enabled=0,
                    onboard_control_sensors_health=0,
                    load=0,
                    voltage_battery=12000,  # 12V
                    current_battery=-1,
                    battery_remaining=100,
                    drop_rate_comm=0,
                    errors_comm=0,
                    errors_count1=0,
                    errors_count2=0,
                    errors_count3=0,
                    errors_count4=0
                )
            except Exception as e:
                print(f"[GCS] Error sending heartbeat: {e}")
        time.sleep(0.5)  # 增加頻率到 2Hz

threading.Thread(target=hb_loop, daemon=True).start()

# 3) 不斷接收任意 PX4 封包，記下來源位址；新來源立刻建立 udpout 連線
buf = bytearray(2048)
while True:
    try:
        nbytes, src = rx_sock.recvfrom_into(buf)
    except socket.timeout:
        continue
    except KeyboardInterrupt:
        break

    # src = (ip, port) 例如 ('127.0.0.1', 18571)
    with lock:
        if src not in peers:
            uri = f"udpout:{src[0]}:{src[1]}"
            peers[src] = mavutil.mavlink_connection(uri, source_system=GCS_SYSID, source_component=GCS_COMPID)
            print(f"[GCS] discovered PX4 peer {src}, start HEARTBEAT to {uri}")
            
            # 立即發送一個心跳包以建立連接
            try:
                peers[src].mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
            except Exception as e:
                print(f"[GCS] Error sending initial heartbeat: {e}")