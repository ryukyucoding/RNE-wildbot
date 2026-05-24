# Bridge Navigation Mission

自動流程：Nav2 導航至橋頭 → 原地旋轉對齊橋軸 → 定時直行過橋（上坡 / 平台 / 下坡）。

---

## 快速啟動流程（整體三階段）

```
Phase 1: 掃地圖（SLAM — 保持運行）
Phase 2: 填入座標 bridge_params.yaml
Phase 3: 啟動 Nav2，執行過橋任務（SLAM 繼續提供定位）
```

> **注意：SLAM 全程不關閉。** 掃完地圖後 SLAM 容器繼續跑，
> 到 Phase 3 直接加上 Nav2 即可，不需要切換到 AMCL。

---

## Phase 1 — 掃地圖

### Terminal 1（pros_app 目錄，保持開著）

```bash
cd /Users/albert/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_app
./mission/start_slam.sh
```

**→ 開啟 Unity → 按 Play**

---

### Terminal 2（進入 car_control 容器）

```bash
cd /Users/albert/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_car
./car_control.sh
```

容器內：

```bash
r                                   # colcon build + source（第一次必做；之後程式碼有修改才需重做）
ros2 run pros_car_py robot_control  # 手動控制車子掃地圖
```

手動控制鍵盤：

| 鍵 | 動作 |
|----|------|
| `w` | 前進 |
| `s` | 後退 |
| `a` | 左斜走 |
| `d` | 右斜走 |
| `e` | 左自轉 |
| `r` | 右自轉 |
| `z` | 停止 |
| `q` | 回主選單 |

> 在 Foxglove 確認 `/map` 已把橋與周圍牆壁掃清楚後退出。

---

### 讀取橋的座標（Foxglove Publish Point）

打開 Foxglove → 3D Panel → 工具列選 **"Publish point"**。

**先點 Point A（橋頭）：**
- 點擊橋入口前的平地，橋寬中線，離斜坡起點約 0.3 m 處。
- 在 Terminal 2 容器內讀值：

```bash
ros2 topic echo /clicked_point --once
```

記下 `x` 和 `y`（這是 `bridge_foot_x` / `bridge_foot_y`）。

**再點 Point B（橋尾）：**
- 點擊橋出口後的平地，橋寬中線。
- 再次讀值，記下 `x` 和 `y`。

**計算 `bridge_heading_deg`：**

```bash
python3 -c "
import math
Ax = <Point_A_x>   # 替換成實際數字
Ay = <Point_A_y>
Bx = <Point_B_x>
By = <Point_B_y>
deg = math.degrees(math.atan2(By - Ay, Bx - Ax))
print(f'bridge_heading_deg = {deg:.2f}')
"
```

---

### 儲存地圖（SLAM 繼續運行）

```bash
# 回到 Terminal 1（pros_app 目錄）
./mission/save_map.sh
```

> `save_map.sh` 只儲存地圖，**不停止** SLAM 容器。
> 執行後 SLAM 繼續提供 `map → odom → base_footprint` TF。

---

## Phase 2 — 填入座標

用任意文字編輯器在**主機**上開啟：

```
workspace/pros/pros_car/src/car_control_pkg/launch/bridge_params.yaml
```

範例（替換成你的實際數值，數字必須保留小數點）：

```yaml
car_control_node:
  ros__parameters:
    bridge_foot_x: 1.23         # Point A x
    bridge_foot_y: -0.45        # Point A y
    bridge_foot_yaw: 35.0       # 與 bridge_heading_deg 相同
    bridge_heading_deg: 35.0    # python3 計算出的角度
    foot_reached_thresh_m: 0.3
    foot_reached_hold_sec: 1.0    # SLAM 在橋上會跳 — 需持續靠近 1s 才判定到達橋腳
    foot_min_approach_dist_m: 0.5
    align_tol_deg: 8.0
    # 過橋段：以 odom 沿橋方向位移為主（SLAM 在坡道不可靠）
    cross_up_dist_m: 0.7          # 上坡距離 — 依 Foxglove A→平台測量
    cross_platform_dist_m: 1.05
    cross_down_dist_m: 0.7
    cross_segment_max_sec: 25.0
    cross_stuck_time_sec: 2.5
    cross_min_progress_m: 0.04
    cross_up_sec: 8.0             # odom 不可用時的時間後備
    cross_platform_sec: 8.0
    cross_down_sec: 8.0
    cross_up_action: "FORWARD"
    cross_platform_action: "FORWARD"
    cross_down_action: "FORWARD"
```

> **調整提示：**
> - `cross_*_dist_m`：在 Foxglove 量 A→平台頂、平台長、下坡長（A→B 總長約 2.45 m 可拆成 0.7+1.05+0.7）。
> - 上坡容易打滑 → 保持 `FORWARD`（全速）+ 調大 `cross_up_dist_m`。
> - 看 log `CROSS_UP: odom=0.35/0.70 m` — 若卡在坡道 odom 不增加，會持續驅動直到距離達標或 `cross_segment_max_sec`。
> - 下坡容易過快 → `cross_down_action: "FORWARD_SLOW"` + 適當縮短秒數。

---

## Phase 3 — 執行過橋任務

### Terminal 1（啟動 Nav2）

```bash
cd /Users/albert/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_app
./mission/start_nav.sh
```

> SLAM 已在跑，`start_nav.sh` 只額外啟動 Nav2。
> **不需要** 設定初始位姿（AMCL 的 2D Pose Estimate）——
> SLAM 從 Phase 1 開始就持續追蹤機器人位置。

確認定位正常：
```bash
ros2 run tf2_ros tf2_echo map base_footprint
```
應看到持續更新的座標輸出。

---

### Terminal 2（啟動 bridge_nav）

> **不要用** `./car_control.sh` 選單裡的 `robot_control` 或 Auto Navigation — 那些不是 Bridge_Nav。
> 必須用下面的 `bridge_nav.launch.py`。

```bash
cd /Users/albert/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_car
./car_control.sh
```

容器內（程式碼有修改時必須 `r`）：

```bash
r                                               # colcon build + source
ros2 launch car_control_pkg bridge_nav.launch.py
```

等到看到 `Navigation Action Server initialized` 後，繼續下一步。

---

### Terminal 3（觸發任務）

> **重要：** `docker exec` 必須進入 **正在跑 `bridge_nav.launch.py` 的那個容器**（`docker ps` 看最新啟動的 `pros_car_docker_image`）。
> 你有兩個 pros_car 容器時，送錯容器 = 任務不會動。

**開新的 Terminal**，exec 進 **Terminal 2 同一個** Docker 容器：

```bash
# 先找容器 ID（選正在跑 bridge_nav 的那個，通常是最近 Created 的）
docker ps

# exec 進去（把 <CONTAINER_ID> 換成 cb90934c52f5 這類 ID）
docker exec -it <CONTAINER_ID> bash
```

容器內：

```bash
source /workspaces/install/setup.bash

# 觸發 Bridge_Nav 任務（不需要在地圖上點任何目標）
ros2 action send_goal /nav_action_server action_interface/action/NavGoal "{mode: 'Bridge_Nav'}"
```

---

## 觀察任務進度（Terminal 2 logs）

成功的完整流程應看到：

```
Bridge_Nav reset; params={...}
APPROACH: dist=1.10 m, bearing_err=35.0° → CCW
APPROACH: dist=0.55 m, bearing_err=8.0° → forward
Bridge_Nav: reached foot (dist=0.28 m); aligning
ALIGN: diff=32.1° → rotating CW
ALIGN: diff=...° → close enough; starting climb
Bridge_Nav: CROSS_UP done -> CROSS_PLATFORM
Bridge_Nav: CROSS_PLATFORM done -> CROSS_DOWN
Bridge_Nav: CROSS_DOWN done -> DONE
Bridge_Nav: bridge traversed
```

若卡在 APPROACH，可能會看到（每 2 秒一次）：
`APPROACH: waiting for global plan on /received_global_plan...`

---

## 緊急停止

| 方式 | 效果 |
|------|------|
| Terminal 3 按 `Ctrl+C` | 取消 action → 觸發 `cancel_callback` → 發 STOP |
| Terminal 2 按 `Ctrl+C` | 整個 car_control_node 停止 |

---

## 常見問題排解

| 問題 | 可能原因 | 解法 |
|------|----------|------|
| APPROACH 一直等，車不動 | TF 未就緒、bearing 不變、或 Unity 未 Play | 確認 Unity Play；`ros2 topic echo /car_C_rear_wheel --once` 應看到 ±450 等級（不是 ±10）；`tf2_echo map base_footprint` 看 pose 是否更新 |
| 只看到 params 就停住 | 用了 control 選單而非 `bridge_nav.launch.py` | 改用 `ros2 launch car_control_pkg bridge_nav.launch.py` + Terminal 3 送 action |
| 旋轉方向反了 | `bridge_heading_deg` 算錯（可能差 180°） | 確認目前 yaw：`ros2 run tf2_ros tf2_echo map base_footprint` |
| 卡在坡道、卻顯示過橋完成 | SLAM 在橋上漂移，純計時過橋太早結束 | 看 log `CROSS_UP: odom=X/Y m`；調大 `cross_up_dist_m`；確認 odom TF 存在 |
| 還沒到橋腳就進 ALIGN | SLAM 誤判距離 | 調大 `foot_reached_hold_sec`；確認曾 `max approach dist > foot_min_approach_dist_m` |
| 到達橋頭後不停止 | `foot_reached_thresh_m` 太小，或地圖 offset 大 | 調大 `foot_reached_thresh_m`（0.4~0.5） |
| 爬坡爬不上去 | 速度或距離不夠 | `cross_up_action: "FORWARD"` + 調大 `cross_up_dist_m` |
| 過橋後 SLAM 位置跑掉 | 橋上 LiDAR 掃不到地圖特徵（正常現象） | 落地後等幾秒讓 SLAM 重新收斂；過橋段靠 odom 不靠 map |
| `Bridge_Nav` mode 不存在 | 沒有重新 `colcon build` | Terminal 2 容器內執行 `r` |
| `TF lookup map→base_footprint failed` | SLAM 尚未發布 TF（剛啟動） | 等幾秒後重試；確認 `ros2 run tf2_ros tf2_echo map base_footprint` 有輸出 |

---

## 相關程式碼

| 檔案 | 說明 |
|------|------|
| `src/car_control_pkg/car_control_pkg/car_nav_controller.py` | `bridge_nav()` 狀態機（APPROACH / ALIGN / CROSS / DONE） |
| `src/car_control_pkg/car_control_pkg/car_control_common.py` | TF2 pose 讀取 / `publish_goal_pose()` / 訂閱 `/received_global_plan` |
| `src/car_control_pkg/car_control_pkg/car_action_server.py` | `Bridge_Nav` mode 串接 |
| `src/car_control_pkg/launch/bridge_params.yaml` | **← 你需要修改這個** |
| `src/car_control_pkg/launch/bridge_nav.launch.py` | 啟動腳本 |
| `workspace/pros/pros_app/mission/start_slam.sh` | Phase 1：啟動 SLAM |
| `workspace/pros/pros_app/mission/save_map.sh` | Phase 1 結束：儲存地圖（SLAM 繼續跑） |
| `workspace/pros/pros_app/mission/start_nav.sh` | Phase 3：啟動 Nav2（不含 AMCL） |
