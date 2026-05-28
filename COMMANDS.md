# ˊWildbot 常用指令手冊

---

## 🐳 Docker 建置

```bash

cd wildbot_workspace
./launch_shell.sh


# 建置 wildbot 主容器（在 wildbot_workspace/ 目錄下）
cd wildbot_workspace
docker build --no-cache --network=host -t wildbot_workspace .



# 建置 YOLO 容器（在 ros2_yolo_integration/ 目錄下）
cd workspace/pros/ros2_yolo_integration
docker build --no-cache --network=host -t ros2_yolo_integration .
```

---

## 🚗 啟動車子

```bash
# 啟動所有服務（car + camera + LiDAR + rosbridge + static TF）
cd wildbot_workspace
./scripts/00_start_all.sh

# 進入 wildbot 容器 (可以多開一個./launch_shell.sh 打指令的地方)
docker exec -it wildbot bash
```

---

## 🗺️ 實車建圖 → AMCL → Nav2 回家

**不要**在實體車開 `localization_unity.sh`（Unity 專用）。  
Wildbot 用下面這套；`00_start_all.sh` **不含** localization，需另外開。

### 前置：統一 ROS_DOMAIN_ID

`wildbot_workspace/docker/compose/.env` 與 `workspace/pros/pros_app/docker/compose/.env` 必須相同（目前皆為 `0`）。

### 第一次：建圖（只需做一次）

```bash
# Terminal 1：硬體（車 + 相機 + LiDAR）
cd ~/RNE/wildbot_workspace
./scripts/00_start_all.sh

# Terminal 2：SLAM 建圖
./scripts/01_mapping.sh
# Foxglove：Add panel → Map → Topic=/map，Fixed frame=map
# 用 cmd_vel 慢速繞場地一圈（見下方「車輪控制」）

# Terminal 3：存地圖（**01_mapping 仍在跑時**執行）
./scripts/02_save_map.sh
# 成功會看到 map01.yaml + map01.pgm；失敗請確認 Foxglove /map 有在長
# 成功後 Ctrl+C 停 Terminal 2
./scripts/stop_nav_stack.sh
```

地圖會存到 `wildbot_workspace/maps/map01/map01.yaml` + `map01.pgm`。

### 之後每次：定位 + Nav2 完整流程

**若剛改過 `Dockerfile`（加入 nav2）或 localization compose，須先重建映像：**

```bash
cd ~/RNE/wildbot_workspace
docker build -t wildbot_workspace .
```

#### Step 1：把車子放到起始位置，啟動硬體

```bash
# Terminal 1（保持開著）
cd ~/RNE/wildbot_workspace
./scripts/00_start_all.sh


#找出自己目前位置
ros2 topic echo /amcl_pose --once
```

#### Step 2：啟動定位 + Nav2

```bash
# Terminal 2（保持開著）
cd ~/RNE/wildbot_workspace
./scripts/03_localization_nav.sh
```

#### Step 3：修正 DDS 發現問題（**每次必做**）

FastDDS 在 Docker bridge network 有單向發現 bug，localization 容器收不到 scan 資料就無法發 TF。啟動後**等 10 秒**再重啟 localization：

```bash
sleep 10 && docker restart compose-localization-1
```

#### Step 4：設 initial pose

```bash
cd ~/RNE/wildbot_workspace
./scripts/set_initial_pose.sh 4.870740515182902 2.9381473065802726 2.8279
```

> **座標更新方式**：機器人放好、AMCL 收斂後，跑一次 `./scripts/save_current_pose.sh`，
> 它會自動讀取目前位置並更新上面這行。下次直接用就好，不需要手動轉換四元數。

#### Step 5：驗證

```bash
cd ~/RNE/wildbot_workspace
./scripts/verify_nav.sh
# 應看到 bt_navigator: active
```

Foxglove 確認：3D panel Fixed frame 改成 `map`，地圖出現，機器人模型在正確位置。

---

**⚠️ 注意事項**

- AMCL 只追蹤**輪子轉動**的移動（cmd_vel 驅動）。用手搬動機器人 odom 不更新，AMCL 不會跟。搬到新位置後必須重設 initial pose。
- 每次 `03_localization_nav.sh` 重啟後都要重做 Step 3 + Step 4。
- 座標有變時跑 `./scripts/save_current_pose.sh`，自動更新 COMMANDS.md 的 Step 4，不需要手動換算四元數。

---

**故障排除**

若 Foxglove 設 initial pose 但 AMCL 不動（`map` frame 不在 TF tree）：

```bash
# 確認 scan 有沒有進到 localization 容器
docker exec compose-localization-1 bash -lc \
  "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 topic hz /scan"
# 沒資料 → 重啟 localization
docker restart compose-localization-1
# 再設一次 initial pose（用 Step 4 的座標）
./scripts/set_initial_pose.sh <Step4的座標>
```

若 log 出現 `**Goal rejected**` / `**Action server is inactive**`：

```bash
./scripts/set_initial_pose.sh <Step4的座標>
./scripts/restart_navigation.sh
./scripts/verify_nav.sh
```

若 Foxglove 設 initial pose 後 AMCL 死掉（log 有 `symbol lookup error`），代表映像裡 FastCDR 版本太舊，須重建：

```bash
cd ~/RNE/wildbot_workspace
docker build -t wildbot_workspace .
./scripts/stop_nav_stack.sh
./scripts/03_localization_nav.sh
```

驗證（**在 host** 執行）：

```bash
cd ~/RNE/wildbot_workspace
./scripts/verify_nav.sh
```

### 跑 bear_mission（有 AMCL 時）

```bash
ros2 run pros_car_py bear_mission --ros-args \
  -p amcl_wait_timeout_sec:=30.0
```

預設已調低 FAR 區轉向（`-40px` 級偏差不再猛轉半圈）。若仍覺得轉太多，可再加：

```bash
  -p visual_servo_max_yaw_far:=80.0 \
  -p visual_servo_yaw_deadband_px:=30.0
```

`amcl_wait_timeout_sec` 建議 **30.0** 以上（留時間設 initial pose）。有 AMCL 時會記錄 map 座標 home，夾取後用 **Nav2 NavigateToPose** 回家（不再被 odom 回程搶先）。

若 log 出現 `**Goal rejected`** / `**Action server is inactive`**：Nav2 在設 initial pose 前就啟動了。解法：

```bash
./scripts/set_initial_pose.sh <Step4的座標>
./scripts/restart_navigation.sh
./scripts/verify_nav.sh   # 須看到 bt_navigator lifecycle active
```

### 停止 nav stack（不關車體）

```bash
cd ~/RNE/wildbot_workspace
./scripts/stop_nav_stack.sh
```

---

## 🔨 Build ROS2 套件（容器內）

```bash
# 只 build pros_car_py（改了 bear_mission_node / arm_controller 等後執行）
cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

# 全部 build
cd /workspaces
colcon build --symlink-install && source install/setup.bash
```

---

## 🤖 YOLO 啟動

```bash
# 主機上：啟動 YOLO 容器（x86_64，自動嘗試 GPU → 失敗則 CPU）
cd ~/RNE/workspace/pros/ros2_yolo_integration
./yolo_activate.sh

# 容器內：Build 並啟動 YOLO 節點
cd /workspaces && colcon build --symlink-install && source install/setup.bash
ros2 run yolo_example_pkg yolo_node --ros-args \
  --remap /camera/image/compressed:=/camera/color/image_raw/compressed \
  -p camera_optical_frame:=camera_color_optical_frame \ -p target_class:=knob
```

---

## 🦾 手臂控制

```bash
# 手臂回歸待機位置（任務開始前 / 測試用；夾爪打開）
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_1_joint','arm_2_joint','gripper_joint'], \
    points: [{positions: [2.0, 0.5, 3.5], time_from_start: {sec: 1}}]}"
```

---

## 🐻 去抓熊任務

### startup_moves 啟動移動序列（選用）

用逗號串接多個動作，取代單純的 `startup_forward_m`：


| Token     | 動作        | 單位  |
| --------- | --------- | --- |
| `F:<m>`   | 前進        | 公尺  |
| `B:<m>`   | 後退        | 公尺  |
| `R:<deg>` | 原地右轉（順時鐘） | 度   |
| `L:<deg>` | 原地左轉（逆時鐘） | 度   |


```
-p startup_moves:="F:0.35,R:90,F:0.10"   # 前進 0.85m → 右轉 90° → 前進 0.50m
-p startup_forward_speed:=0.25            # 前進速度 m/s（F/B 共用）
-p startup_rotation_speed:=0.8           # 旋轉速度 rad/s（R/L 共用）
```

原本：

```
  -p startup_forward_m:=0.85 \
  -p startup_forward_speed:=0.25
```

> `startup_moves` 有設定時會優先執行，忽略 `startup_forward_m`。
> 不設 `startup_moves` 則維持舊行為（`startup_forward_m` 直線前進）。

```bash
cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

ros2 run pros_car_py bear_mission --ros-args \
  -p amcl_wait_timeout_sec:=30.0 \
  -p grasp_trigger_dist_m:=0.4 \
  -p approach_stop_dist_m:=0.3 \
  -p visual_servo_target_depth_m:=0.6 \
  -p visual_servo_yaw_deadband_px:=80.0 \
  -p visual_servo_yaw_soft_scale_px:=300.0 \
  -p visual_servo_max_yaw_near:=10.0 \
  -p visual_servo_max_yaw_far:=18.0 \
  -p visual_servo_search_spin_speed:=8.0 \
  -p align_pixel_thresh:=100.0 \
  -p grasp_bbox_px:=200.0 \
  -p grasp_confirm_frames:=4 \
  -p visual_servo_dx_ema_alpha:=0.15 \
  -p visual_servo_depth_ema_alpha:=0.20 \
  -p approach_max_speed_mps:=0.80 \
  -p visual_servo_max_forward_speed_far:=500.0 \
  -p obstacle_source_debug_enabled:=false \
  -p approach_yolo_lost_grace_sec:=1.5 \
  -p approach_yolo_search_spin_speed_tier:=slow \
  -p approach_yolo_explore_forward_sec:=2.0 \
  -p visual_servo_dx_ema_alpha:=0.08 \
  -p visual_servo_min_yaw_large_px:=12.0 \
  -p startup_moves:="F:1.6,L:75,F:0.1"   # 前進 0.85m → 右轉 90° → 前進 0.50m
  -p startup_moves:="F:1.2"
  -p startup_forward_speed:=0.18            # 前進速度 m/s（F/B 共用）
  -p startup_rotation_speed:=0.8           # 旋轉速度 rad/s（R/L 共用）

```

---

## 🐻 推熊回家任務（擋板策略）

```bash
cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

ros2 run pros_car_py push_mission --ros-args \
  -p amcl_wait_timeout_sec:=30.0 \
  -p approach_stop_dist_m:=0.28 \
  -p approach_max_speed_mps:=0.55 \
  -p approach_max_sec:=12.0 \
  -p approach_stall_min_sec:=4.0 \
  -p approach_stall_max_dist_m:=0.45 \
  -p visual_servo_yaw_deadband_px:=80.0 \
  -p visual_servo_yaw_soft_scale_px:=300.0 \
  -p visual_servo_max_yaw_near:=6.0 \
  -p collect_forward_sec:=0.8 \
  -p nav_home_timeout_sec:=120.0 \
  -p home_arrival_thresh_m:=0.30 \
  -p score_back_max_sec:=5.0 \
  -p score_approach_dist_m:=0.30 \
  -p score_lift_deg:=20.0 \
  -p approach_yolo_lost_grace_sec:=1.2 \
  -p approach_yolo_search_spin_speed:=8.0 \
  -p approach_yolo_explore_forward_sec:=2.0

```

流程：YOLO 靠近 → 進入收集區 → 前進夾住 → **Nav2 NavigateToPose 推回 home** → 退後找熊 → 夾起上抬**放開**得分

---

## ⬜ 長方形自動走行

不需 AMCL / Nav2 / 地圖，只要硬體與 odom 有在跑。預設 **2.0m × 2.0m**（正方形）；可用 `length_m` / `width_m` 或 `side1_m`～`side4_m` 指定各邊。

```bash
# Terminal 1：硬體
cd ~/RNE/wildbot_workspace
./scripts/00_start_all.sh

# Terminal 2：wildbot 容器內
docker exec -it wildbot bash
cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

# 預設 2m × 2m
ros2 run pros_car_py rectangle_drive

# 長方形：長邊 2m、寬邊 1m（邊 1→3 走 2m，邊 2→4 走 1m）
ros2 run pros_car_py rectangle_drive --ros-args -p length_m:=2.0 -p width_m:=1.0

# 四邊各自指定（未指定者：side1/3 用 length_m，side2/4 用 width_m）
ros2 run pros_car_py rectangle_drive --ros-args \
  -p side1_m:=0.85 -p side2_m:=2.95 -p side3_m:=0.85 -p side4_m:=2.95 \
  -p turn1_deg:=48.5 -p turn2_deg:=49.0 -p turn3_deg:=50.0

# 小範圍測試
ros2 run pros_car_py rectangle_drive --ros-args -p length_m:=0.5 -p width_m:=0.3

# 慣性仍過頭時，再調小 turn_deg（預設 85°，實際約接近 90°）
ros2 run pros_car_py rectangle_drive --ros-args -p length_m:=0.5 -p width_m:=0.3 -p turn_deg:=82.0

# 每次轉彎可單獨調角（未指定者沿用 turn_deg）
ros2 run pros_car_py rectangle_drive --ros-args -p length_m:=0.85 -p width_m:=3.12 \
  -p turn1_deg:=54.0 -p turn2_deg:=82.0 -p turn3_deg:=85.0
```

走行順序：前進 side1 → 右轉 → 前進 side2 → 右轉 → …。`length_m` / `width_m` 為 side 預設（1/3 用 length、2/4 用 width）；`side1_m`～`side4_m` 可個別覆寫。轉彎同理：`turn_deg` 為預設，`turn1_deg`～`turn3_deg` 可個別覆寫。Ctrl+C 會自動停車。

轉彎脈衝等進階參數仍寫在 `pros_car_py/rectangle_drive_node.py`（`TURN_PULSE_SEC` 等），實車偏差大時改常數後重新 build。

---

## 🤖 接近並抓取（approach_grab）

不需 AMCL / Nav2 / YOLO / 地圖，只要硬體、odom 與手臂在跑。

走行順序：**前進 forward1 → 右轉 turn_deg → 前進 forward2 → 在三個角度各夾取一次**（`grab_turn1_deg`～`grab_turn3_deg` 為每次抓取前的增量轉角；預設 `0° / 120° / 120°`）。

```bash
# Terminal 1：硬體
cd ~/RNE/wildbot_workspace
./scripts/00_start_all.sh

# Terminal 2：wildbot 容器內
docker exec -it wildbot bash
cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

# 預設參數
ros2 run pros_car_py approach_grab

# 自訂走行與掃描角度
ros2 run pros_car_py approach_grab --ros-args \
  -p forward1_m:=0.85 -p forward2_m:=0.55 -p turn_deg:=48.5 \
  -p grab_turn1_deg:=0.0 -p grab_turn2_deg:=120.0 -p grab_turn3_deg:=120.0

# 較窄掃描（例：0° / 45° / 90°）
ros2 run pros_car_py approach_grab --ros-args \
  -p grab_turn1_deg:=0.0 -p grab_turn2_deg:=45.0 -p grab_turn3_deg:=45.0

# Launch
ros2 launch pros_car_py approach_grab.launch.py \
  forward1_m:=0.85 forward2_m:=0.55 turn_deg:=48.5
```

**注意**：`run_grasp_blocking()` 無「已夾到物體」回饋，節點會依序完成三次夾取動作（除非 Ctrl+C）。抓取間預設會打開夾爪回到待機姿勢（`reopen_gripper_between_tries:=true`）。進階脈衝常數見 `pros_car_py/approach_grab_node.py`。

---

## 🔍 偵錯指令

```bash
# 確認 topic 有沒有在跑
ros2 topic list
ros2 topic hz /yolo/target_info
ros2 topic hz /base_controller/odom

# 確認 YOLO 有在發目標資訊
ros2 topic echo /yolo/target_info

# 確認 odom 有在發
ros2 topic echo /base_controller/odom --once

# 確認 TF tree
ros2 run tf2_tools view_frames

# 確認手臂狀態
ros2 topic echo /joint_states

# 底盤直接下速度指令（測試用）
ros2 topic pub --once /base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

## 夾爪失敗

# 在 wildbot 容器內執行（背景跑

ros2 run tf2_ros static_transform_publisher  
  --x 0.105 --y 0.0 --z 0.255  
  --roll 0 --pitch 0 --yaw 0  
  --frame-id base_link --child-frame-id camera_link &

驗證 TF 有沒有通
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

第一步：進入容器

  進去後先 source：
  source /opt/ros/jazzy/setup.bash

---

##車輪控制

  往前走：
  ros2 topic pub -r 10 /base_controller/cmd_vel geometry_msgs/msg/TwistStamped  
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"

  停止（一定要記得停）：
  ros2 topic pub --once /base_controller/cmd_vel geometry_msgs/msg/TwistStamped  
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"

  左轉： angular: {z: 0.4} ／ 右轉： angular: {z: -0.4}

---

## 📡 Foxglove 常用 Topics


| 畫面             | Topic                                | 備註                          |
| -------------- | ------------------------------------ | --------------------------- |
| **建圖地圖**       | `/map`                               | Map panel，Fixed frame=`map` |
| YOLO 偵測影像      | `/yolo/detection/compressed`         |                             |
| 雷達掃描（3D panel） | `/scan`                              |                             |
| odom 位置        | `/base_controller/odom`              |                             |
| 相機影像           | `/camera/color/image_raw/compressed` |                             |
| 手臂關節狀態         | `/joint_states`                      |                             |


# 進入 wildbot 容器

docker exec -it wildbot bash

# 容器內：build

cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

# 上橋

```bash
cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

ros2 run pros_car_py rectangle_drive --ros-args \
-p length_m:=0.07 -p width_m:=2.95 -p side2_m:=2.95 \
-p turn1_deg:=48.45 -p turn2_deg:=48.5 -p turn3_deg:=53.0
```

# 左右移動

```bash
docker exec -it wildbot bash

source /opt/ros/jazzy/setup.bash
source /workspaces/install/setup.bash
python3 /workspaces/teleop_key.py
```

