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
    align_tol_deg: 8.0
    cross_up_sec: 3.0           # ← 依實際橋長調整
    cross_platform_sec: 2.0
    cross_down_sec: 3.0
    cross_up_action: "FORWARD"
    cross_platform_action: "FORWARD_SLOW"
    cross_down_action: "FORWARD_SLOW"
```

> **調整提示：**
> - `cross_*_sec`：用秒數 × 速度估橋長。在 Unity 先跑一次，依實際情況微調，再換到實體機。
> - 上坡容易打滑 → 把 `cross_up_sec` 加長 + 保持 `FORWARD`（全速）。
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

```bash
cd /Users/albert/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_car
./car_control.sh
```

容器內（如果前一步已經 `r` 過且程式碼沒改，可跳過 `r`）：

```bash
r                                               # 有改程式碼才需要重新 build
ros2 launch car_control_pkg bridge_nav.launch.py
```

等到看到 `Navigation Action Server initialized` 後，繼續下一步。

---

### Terminal 3（觸發任務）

**開新的 Terminal**，exec 進同一個 Docker 容器：

```bash
# 先找容器 ID
docker ps

# exec 進去（把 <CONTAINER_ID> 換成實際 ID，前幾個字元即可）
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
Published bridge foot goal to /goal_pose: (x, y, yaw=...)
Bridge_Nav: reached foot (dist=... m); aligning
Bridge_Nav: aligned; starting climb
Bridge_Nav: CROSS_UP done -> CROSS_PLATFORM
Bridge_Nav: CROSS_PLATFORM done -> CROSS_DOWN
Bridge_Nav: CROSS_DOWN done -> DONE
Bridge_Nav: bridge traversed
```

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
| APPROACH 一直等，車不動 | Nav2 沒收到 `/goal_pose` 或 SLAM 未啟動 | 確認 SLAM 在跑（`ros2 topic echo /map --once`）；查 `/received_global_plan` |
| 旋轉方向反了 | `bridge_heading_deg` 算錯（可能差 180°） | 確認目前 yaw：`ros2 run tf2_ros tf2_echo map base_footprint` |
| 到達橋頭後不停止 | `foot_reached_thresh_m` 太小，或地圖 offset 大 | 調大 `foot_reached_thresh_m`（0.4~0.5） |
| 爬坡爬不上去 | 速度或時間不夠 | `cross_up_action: "FORWARD"` + 調長 `cross_up_sec` |
| 過橋後 SLAM 位置跑掉 | 橋上 LiDAR 掃不到地圖特徵（正常現象） | 落地後等幾秒讓 SLAM 重新收斂；或在 Foxglove 確認 `/map` 持續更新 |
| `Bridge_Nav` mode 不存在 | 沒有重新 `colcon build` | Terminal 2 容器內執行 `r` |
| `TF lookup map→base_footprint failed` | SLAM 尚未發布 TF（剛啟動） | 等幾秒後重試；確認 `ros2 run tf2_ros tf2_echo map base_footprint` 有輸出 |

---

## 相關程式碼

| 檔案 | 說明 |
|------|------|
| `src/car_control_pkg/car_control_pkg/car_nav_controller.py` | `bridge_nav()` 狀態機（APPROACH / ALIGN / CROSS / DONE） |
| `src/car_control_pkg/car_control_pkg/car_control_common.py` | TF2 pose 讀取 / `publish_goal_pose()` / bridge 參數宣告 |
| `src/car_control_pkg/car_control_pkg/car_action_server.py` | `Bridge_Nav` mode 串接 |
| `src/car_control_pkg/launch/bridge_params.yaml` | **← 你需要修改這個** |
| `src/car_control_pkg/launch/bridge_nav.launch.py` | 啟動腳本 |
| `workspace/pros/pros_app/mission/start_slam.sh` | Phase 1：啟動 SLAM |
| `workspace/pros/pros_app/mission/save_map.sh` | Phase 1 結束：儲存地圖（SLAM 繼續跑） |
| `workspace/pros/pros_app/mission/start_nav.sh` | Phase 3：啟動 Nav2（不含 AMCL） |
