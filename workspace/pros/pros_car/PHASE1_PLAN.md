# Phase 1 實作計畫：辨識熊 → 走過去 → 夾爪

## 目標
讓 bear_mission_node 可以在實體機器人上：
1. 用 YOLO 辨識熊
2. 視覺伺服走過去
3. 啟動夾爪夾取

Nav2 回起點（Phase 2）暫時跳過。

---

## 虛擬 vs 實體差異總覽

| 功能 | 虛擬環境 | 實體機器人 | 需改動 |
|------|----------|------------|--------|
| 車輪 topic | `car_C_rear_wheel` / `car_C_front_wheel` | `/base_controller/cmd_vel` | ✅ 要改 |
| 車輪訊息型別 | `Float32MultiArray [FL,FR,RL,RR]` | `TwistStamped (linear.x, angular.z)` | ✅ 要改 |
| 手臂 topic | `robot_arm` | `/arm_controller/joint_trajectory` | ✅ 要改 |
| 手臂訊息型別 | `JointTrajectoryPoint` | `JointTrajectory`（含 joint_names、header）| ✅ 要改 |
| 手臂角度單位 | radians（arm_controller_2D.py 已轉換）| radians | ✅ 不用改 |
| YOLO camera topic | 待確認 | `/camera/color/image_raw/compressed` | 🔍 要確認 |
| Nav2 / AMCL | 使用 | 跳過（Phase 2）| ⏭ 暫時跳過 |

---

## Step 1 — 改 `ros_communicator_config.py`

**改 ACTION_MAPPINGS 格式**：從四輪速度 `[FL, FR, RL, RR]` 改為 `(linear_x, angular_z)`。

速度限制參考（來自 controllers.yaml）：
- linear.x 最大 **0.546 m/s**
- angular.z 最大 **3.983 rad/s**

```python
# 改後格式：(linear_x, angular_z)
ACTION_MAPPINGS = {
    "FORWARD":                    (0.3,    0.0),
    "FORWARD_SLOW":               (0.15,   0.0),
    "BACKWARD":                   (-0.3,   0.0),
    "BACKWARD_SLOW":              (-0.15,  0.0),
    "CLOCKWISE_ROTATION":         (0.0,   -1.5),
    "CLOCKWISE_ROTATION_SLOW":    (0.0,   -0.8),
    "CLOCKWISE_ROTATION_MEDIAN":  (0.0,   -1.2),
    "COUNTERCLOCKWISE_ROTATION":  (0.0,    1.5),
    "COUNTERCLOCKWISE_ROTATION_SLOW":   (0.0, 0.8),
    "COUNTERCLOCKWISE_ROTATION_MEDIAN": (0.0, 1.2),
    "LEFT_FRONT":                 (0.2,    0.8),
    "RIGHT_FRONT":                (0.2,   -0.8),
    "STOP":                       (0.0,    0.0),
}
```

> ⚠️ 速度值需要在實車上測試後微調。

---

## Step 2 — 改 `ros_communicator.py`：車輪控制

**改動位置**：
- 移除 `publisher_rear`、`publisher_forward`（Float32MultiArray publishers）
- 新增 `publisher_cmd_vel`（TwistStamped publisher to `/base_controller/cmd_vel`）
- 改寫 `publish_car_control()`：查 ACTION_MAPPINGS → 發 TwistStamped
- 改寫 `publish_raw_car_control()`：接受 `(linear_x, angular_z)` → 發 TwistStamped

---

## Step 3 — 改 `ros_communicator.py`：手臂控制

**改動位置**：
- 移除 `publisher_joint_trajectory`（JointTrajectoryPoint publisher）
- 新增 `publisher_joint_trajectory`（JointTrajectory publisher to `/arm_controller/joint_trajectory`）
- 改寫 `publish_robot_arm_angle()`：包成完整 JointTrajectory

```python
# 改後格式
def publish_robot_arm_angle(self, angle):
    msg = JointTrajectory()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']
    point = JointTrajectoryPoint()
    point.positions = [float(a) for a in angle]
    point.time_from_start.sec = 1
    msg.points = [point]
    self.publisher_joint_trajectory.publish(msg)
```

> ⚠️ 手臂角度限制（radians）：
> - arm_1_joint: 0.524 ～ 3.665 rad（30° ～ 210°）
> - arm_2_joint: 0.0 ～ 4.189 rad（0° ～ 240°）
> - gripper_joint: 2.932 ～ 4.189 rad（168° ～ 240°），**嚴禁低於 2.932 rad**

---

## Step 4 — 確認 YOLO camera topic

`object_detect.py` 訂閱 `/camera/image/compressed`，但實體相機發布的是 `/camera/color/image_raw/compressed`。

**啟動 YOLO node 時需加 remap：**
```bash
ros2 run yolo_example_pkg object_detect --ros-args \
  --remap /camera/image/compressed:=/camera/color/image_raw/compressed
```

---

## Step 5 — 測試順序

```bash
# 1. 先單獨測試車輪
ros2 topic pub /base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.1}}}"

# 2. 先單獨測試手臂
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_1_joint','arm_2_joint','gripper_joint'], \
    points: [{positions: [1.57, 1.57, 4.0], time_from_start: {sec: 1}}]}"

# 3. 跑 YOLO 確認偵測
# 4. 跑完整 bear_mission_node
```

---

## Phase 2（之後再做）
- 實作 odom → base_link TF broadcaster
- 設定 Nav2（SLAM map、AMCL）
- 測試 NavigateToPose 回起點
