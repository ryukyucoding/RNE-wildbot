# Wildbot 常用指令手冊

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
  -p camera_optical_frame:=camera_color_optical_frame
```

---

## 🦾 手臂控制

```bash
# 手臂回歸待機位置（任務開始前 / 測試用）
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_1_joint','arm_2_joint','gripper_joint'], \
    points: [{positions: [2.7, 1.5, 4.0], time_from_start: {sec: 1}}]}"

# 夾爪打開（gripper_joint 設為最大值 4.0）
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_1_joint','arm_2_joint','gripper_joint'], \
    points: [{positions: [2.7, 1.5, 4.0], time_from_start: {sec: 1}}]}"
```

---

## 🐻 去抓熊任務

```bash
cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

ros2 run pros_car_py bear_mission --ros-args \
  -p amcl_wait_timeout_sec:=3.0 \
  -p grasp_trigger_dist_m:=0.50 \
  -p approach_stop_dist_m:=0.40 \
  -p visual_servo_target_depth_m:=0.45 \
  -p visual_servo_yaw_deadband_px:=100.0 \
  -p visual_servo_yaw_soft_scale_px:=350.0 \
  -p visual_servo_max_yaw_near:=8.0 \
  -p visual_servo_max_yaw_far:=15.0 \
  -p visual_servo_search_spin_speed:=8.0 \
  -p align_pixel_thresh:=100.0 \
  -p grasp_bbox_px:=200.0 \
  -p grasp_confirm_frames:=4 \
  -p visual_servo_dx_ema_alpha:=0.15 \
  -p visual_servo_depth_ema_alpha:=0.20 \
  -p approach_max_speed_mps:=0.80 \
  -p visual_servo_max_forward_speed_far:=500.0 \
  -p obstacle_source_debug_enabled:=true \
  -p approach_yolo_lost_grace_sec:=1.5 \
  -p approach_yolo_search_spin_speed_tier:=slow \
  -p approach_yolo_explore_forward_sec:=2.0

```

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
# 在 wildbot 容器內執行（背景跑）
ros2 run tf2_ros static_transform_publisher \
  --x 0.105 --y 0.0 --z 0.255 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link &

驗證 TF 有沒有通
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame



---

## 📡 Foxglove 常用 Topics

| 畫面 | Topic |
|---|---|
| YOLO 偵測影像 | `/yolo/detection/compressed` |
| 雷達掃描（3D panel）| `/scan` |
| odom 位置 | `/base_controller/odom` |
| 相機影像 | `/camera/color/image_raw/compressed` |
| 手臂關節狀態 | `/joint_states` |




# 進入 wildbot 容器
docker exec -it wildbot bash

# 容器內：build
cd /workspaces
colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash

# 跑 bear mission
ros2 run pros_car_py bear_mission --ros-args \
  -p amcl_wait_timeout_sec:=3.0 \
  -p grasp_trigger_dist_m:=0.65 \
  -p approach_stop_dist_m:=0.45 \
  -p visual_servo_target_depth_m:=0.45 \
  -p visual_servo_yaw_deadband_px:=30.0 \
  -p visual_servo_yaw_soft_scale_px:=200.0 \
  -p visual_servo_max_yaw_near:=80.0 \
  -p visual_servo_max_yaw_far:=140.0 \
  -p visual_servo_search_spin_speed:=60.0 \
  -p align_pixel_thresh:=60.0 \
  -p grasp_bbox_px:=200.0 \
  -p grasp_confirm_frames:=4 \
  -p obstacle_source_debug_enabled:=true \
  -p approach_yolo_lost_grace_sec:=1.5 \
  -p approach_yolo_search_spin_speed_tier:=slow \
  -p approach_yolo_explore_forward_sec:=2.0
