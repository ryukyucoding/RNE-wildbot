# open_door 測試與開發指南（macOS + pros_twin）

本指南說明如何在 **Mac** 上啟動 Unity 虛擬環境、ROS2 Docker 堆疊與 YOLO，並測試 / 修改 `open_door` 自動開門任務。

---

## 架構概覽

```
┌─────────────────┐     WebSocket      ┌──────────────────┐
│  pros_twin      │ ◄──────────────────► │  rosbridge       │
│  (Unity, Mac)   │   ws://127.0.0.1:9090│  (Docker)        │
└─────────────────┘                      └────────┬─────────┘
                                                  │ ROS2 DDS
                    ┌─────────────────────────────┼─────────────────────────────┐
                    │         compose_my_bridge_network                        │
                    │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
                    │  │ slam_unity   │  │ yolo_node    │  │ pros_car     │   │
                    │  │ (可選)       │  │ (YOLO 容器)  │  │ (car 容器)   │   │
                    │  └──────────────┘  └──────────────┘  └──────────────┘   │
                    └─────────────────────────────────────────────────────────┘
```

| 程式 | 跑在哪裡 | 用途 |
|------|----------|------|
| pros_twin | Mac 本機 | 虛擬場景、相機、車子物理 |
| pros_app (rosbridge / slam) | Docker | ROS2 橋接、SLAM |
| yolo_example_pkg `yolo_node` | YOLO Docker | 辨識 `knob` / `bear`，發 `/yolo/*` |
| pros_car `door_open` | pros_car Docker | 開門狀態機 |

**注意：** 不要在 `pros_car` 容器裡跑 `yolo_node`（會出現 `Package 'yolo_example_pkg' not found`）。

---

## 前置條件

- Docker Desktop 已啟動
- 已 clone `RNE-wildbot`（或 `workspace/pros` 下的 `pros_app`、`pros_car`、`ros2_yolo_integration`）
- pros_twin 已安裝並可登入
- `pros_car` 目錄有 `.env`（若沒有）：

```bash
cat > ~/workspace/RNE-wildbot/workspace/pros/pros_car/.env << 'EOF'
ROS_DOMAIN_ID=1
WHEEL_SPEED=10
EOF
```

（若你用的是 `~/workspace/pros/pros_car`，路徑改成對應位置。）

---

## 第一次使用：建立 Docker 網路

只需做一次（若已存在可略過）：

```bash
docker network create compose_my_bridge_network 2>/dev/null || true
```

---

## 標準啟動流程（4 個 Terminal）

以下假設 repo 在：

```bash
export PROS_ROOT=~/workspace/RNE-wildbot/workspace/pros
# 或：export PROS_ROOT=~/workspace/pros
```

### Terminal 1 — rosbridge（必開，保持運行）

```bash
cd $PROS_ROOT/pros_app
python3 ./control.py -s
```

在選單輸入 **`13`**（`./rosbridge_server.sh`）。

看到 rosbridge 啟動成功後，按 **`b`** 回到選單（**不要按 `q`**），讓它在背景跑。

驗證：

```bash
# 另開 shell
lsof -i :9090
# 應有 Docker 在 LISTEN
```

---

### pros_twin（Unity）

1. 啟動 **pros_twin**，登入
2. 選 **FINAL PROJECT**（或你的測試場景），等載入完成
3. **Car → Mode → `AI`**（必須，否則不吃 ROS 輪速指令）
4. Rosbridge 連線：
   - **IP：** `127.0.0.1`（不要用 `ws://` 前綴填在 IP 欄）
   - **Port：** `9090`
   - 按 **Reload**
5. 連線成功後，畫面上的 reload 提示應消失

> 若 Unity 在 VM 裡跑，IP 改填 **Mac 主機的區網 IP**（例如 `192.168.x.x`），不是 VM 的 `127.0.0.1`。

---

### Terminal 2 — SLAM（建議開，用於地圖 / 導航）

**rosbridge 已在 Terminal 1 跑著時**，同一個或新開 `control.py`：

```bash
cd $PROS_ROOT/pros_app
python3 ./control.py -s
```

輸入 **`2`**（`./slam_unity.sh`），保持運行。

> 若只做「手動開車 + 開門測試」、`SKIP_NAVIGATION = True`，可暫時不開 slam，但 Foxglove 看 `/map`、`/scan` 會沒資料。

---

### Terminal 3 — YOLO（必開，保持運行）

```bash
cd $PROS_ROOT/ros2_yolo_integration
./yolo_activate.sh
```

進入容器後：

```bash
r
ros2 run yolo_example_pkg yolo_node
```

預期 log：

```text
Model: detection.pt, classes: {0: 'bear', 1: 'knob'}
Subscribed to /camera/image/compressed (TRANSIENT_LOCAL QoS). Waiting for Unity camera...
Receiving /camera/image/compressed — YOLO pipeline active.
```

**`Receiving ...` 出現後**，Foxglove 才應該有 YOLO 輸出。

可選：降低信心門檻

```bash
YOLO_CONF=0.3 ros2 run yolo_example_pkg yolo_node
```

#### 替代：Mac 本機跑 YOLO（不用 YOLO Docker）

```bash
cd $PROS_ROOT/ros2_yolo_integration/scripts
pip install -r requirements_mac.txt   # 首次
python3 yolo_detect.py
```

**二選一**：不要同時跑 Docker `yolo_node` 和 Mac `yolo_detect.py`。

---

### Terminal 4 — pros_car（手動控車或跑 door_open）

```bash
cd $PROS_ROOT/pros_car
./car_control.sh
```

進容器後：

```bash
r
```

#### 手動把車開到門前（測試用）

```bash
ros2 run pros_car_py robot_control
```

選 **Control Vehicle**，在此 terminal 按 `w/s/a/d/e/r` 控車。

#### 跑自動開門

```bash
ros2 run pros_car_py door_open
```

---

## Foxglove 監控

連線：**Rosbridge** → `ws://localhost:9090`

| 面板 | Topic | 說明 |
|------|-------|------|
| Image | `/yolo/detection/compressed` | YOLO 畫框結果 |
| Raw Messages | `/yolo/target_info` | `[found, distance_m, pixel_offset]` |
| Raw Messages | `/target_label` | 開門時應為 `knob` |

### `/yolo/target_info` 的 `data`

| 索引 | 值 | 意義 |
|------|-----|------|
| `data[0]` | `0` / `1` | 沒找到 / 找到目標 |
| `data[1]` | 公尺 | 深度（太近可能為 `-1`） |
| `data[2]` | 像素 | 相對畫面中心偏移（正=右） |

**沒有** `/yolo/target_info/compressed` 這個 topic；影像才是 `.../detection/compressed`。

---

## 驗證指令（除錯用）

在 Mac 上執行：

```bash
# YOLO 有沒有在發？
docker exec compose-rosbridge-1 bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic info /yolo/target_info"

# 應顯示 Publisher count: 1（yolo_node 在跑）

# 相機有沒有影像？
docker exec compose-rosbridge-1 bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic hz /camera/image/compressed"
```

---

## 修改 open_door 時要動的檔案

| 檔案 | 用途 |
|------|------|
| `pros_car/.../door_open_task.py` | 狀態機、門前座標、`SKIP_NAVIGATION`、`YOLO_TARGET_LABEL`、各種閾值 |
| `ros2_yolo_integration/.../object_detect.py` | Docker 內 `yolo_node` 邏輯 |
| `ros2_yolo_integration/scripts/yolo_detect.py` | Mac 本機 YOLO（roslibpy） |
| `ros2_yolo_integration/.../models/detection.pt` | 模型權重（類別：`bear`, `knob`） |

### 改程式後如何生效

**door_open / pros_car：**

```bash
cd $PROS_ROOT/pros_car
./car_control.sh
# 容器內
r
ros2 run pros_car_py door_open
```

**YOLO（object_detect.py）：**

```bash
cd $PROS_ROOT/ros2_yolo_integration
./yolo_activate.sh
# 容器內
r
ros2 run yolo_example_pkg yolo_node
```

**YOLO（yolo_detect.py，Mac）：** 存檔後 Ctrl+C 重跑 `python3 yolo_detect.py`。

---

## `door_open_task.py` 常用參數

路徑：`workspace/pros/pros_car/src/pros_car_py/pros_car_py/door_open_task.py`

```python
DOOR_APPROACH_GOAL = [2.5, 1.0]   # Nav2 目標 [x, y]，用 Foxglove 量測後修改
SKIP_NAVIGATION = True            # True = 跳過導航，假設你已手動把車開到門前
YOLO_TARGET_LABEL = "knob"        # 必須與 detection.pt 類別名一致
SEARCH_MAX_ITER = 600             # 搜尋門把最長等待（0.1s × 次數）
ALIGN_PIXEL_TOL = 60              # 車身對齊像素容差
```

---

## 常見問題

### `Package 'yolo_example_pkg' not found`

在 **pros_car** 容器執行了 `yolo_node`。改到 **YOLO 容器**（`yolo_activate.sh`）或 Mac 上跑 `yolo_detect.py`。

### Foxglove 收不到 `/yolo/detection/compressed` 或 `/yolo/target_info`

1. `yolo_node` 是否在跑？（`Publisher count` 應為 1）
2. log 有沒有 `Receiving /camera/image/compressed`？
3. Unity **Car Mode = AI**、rosbridge 已 Reload
4. 改完 `object_detect.py` 後有沒有在 YOLO 容器內 `r` rebuild？

### `/yolo/target_info` 一直是 `[0, 0, 0]`

- topic 正常，但 **沒偵測到 knob**
- 車子對準門、門把在視野內
- 看 `/yolo/detection/compressed` 有沒有 `knob` 框
- 試 `YOLO_CONF=0.3`

### 鍵盤控車沒反應

- Unity **Car Mode = AI**
- 在 **Vehicle Mode** 的 terminal 按鍵（需 focus）
- `ros2 topic echo /car_C_front_wheel` 按 `w` 時應有數值

### `docker: open .env: no such file or directory`（car_control.sh）

在 `pros_car` 目錄建立 `.env`（見前置條件）。

### pros_twin 顯示請按 Reload

- rosbridge（Terminal 1）要先啟動
- IP `127.0.0.1`、Port `9090`，再按 Reload

---

## 建議測試順序（第一次）

1. Terminal 1：`rosbridge_server.sh`（選 13）
2. Unity：場景 + **AI** + Reload 連線
3. Terminal 3：`yolo_node`，確認 `Receiving /camera/image/compressed`
4. Foxglove：確認 `/yolo/detection/compressed` 有畫面
5. Terminal 4：手動控車到門前
6. Foxglove：`/yolo/target_info` 出現 `[1, ...]`
7. Terminal 4：`ros2 run pros_car_py door_open`

---

## 一頁速查

```bash
# T1
cd $PROS_ROOT/pros_app && python3 ./control.py -s   # → 13, 再 b

# Unity: FINAL PROJECT, Car=AI, 127.0.0.1:9090 Reload

# T2（可選）
cd $PROS_ROOT/pros_app && python3 ./control.py -s   # → 2

# T3
cd $PROS_ROOT/ros2_yolo_integration && ./yolo_activate.sh
# 容器內: r && ros2 run yolo_example_pkg yolo_node

# T4
cd $PROS_ROOT/pros_car && ./car_control.sh
# 容器內: r && ros2 run pros_car_py door_open
```

---

## 相關分支

本指南對應 **`open_door`** 分支開發。YOLO 模型為 `detection.pt`（類別 `bear`, `knob`），與 HW4 分割模型 `segmentation_hw4_v5.pt`（`bridge`, `road`）不同；開門請勿混用。
