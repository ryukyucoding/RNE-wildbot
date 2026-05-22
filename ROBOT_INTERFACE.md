# Wildbot ROS2 介面文件

## LiDAR

| 項目 | 內容 |
|------|------|
| Topic | `/scan` |
| Type | `sensor_msgs/msg/LaserScan` |

**Message 格式：**
```
std_msgs/Header header
float angle_min        # 掃描起始角度 [rad]
float angle_max        # 掃描結束角度 [rad]
float angle_increment  # 每次掃描角度增量 [rad]
float time_increment
float scan_time
float range_min        # 最小測距值 [m]
float range_max        # 最大測距值 [m]
float[] ranges
float[] intensities
```

---

## 車輪控制

| 項目 | 內容 |
|------|------|
| Topic | `/base_controller/cmd_vel` |
| Type | `geometry_msgs/msg/TwistStamped` |

**Message 格式：**
```
Header header          # frame_id: base_link
Vector3 linear
  float64 x            # 前後速度 [m/s]
  float64 y
  float64 z
Vector3 angular
  float64 x
  float64 y
  float64 z            # 旋轉角速度 [rad/s]
```

**速度限制（來自 controllers.yaml）：**
- linear.x：最大 0.546 m/s，加速度 0.5 m/s²，減速度 -1.5 m/s²
- angular.z：最大 3.983 rad/s，加速度 1.0 rad/s²，減速度 -3.0 rad/s²

**範例指令：**
```bash
ros2 topic pub /base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.2}, angular: {z: 0.4}}}"
```

---

## 手臂控制

| 項目 | 內容 |
|------|------|
| Topic | `/arm_controller/joint_trajectory` |
| Type | `trajectory_msgs/msg/JointTrajectory` |
| 備註 | 單次控制，需加 `--once` 旗標 |

**Message 格式：**
```
Header header
  uint32 seq
  time stamp
  string frame_id
string[] joint_names
JointTrajectoryPoint[] points
  float64[] positions       # 關節位置（rad）
  float64[] velocities
  float64[] accelerations
  float64[] effort
  duration time_from_start
```

> 每個 trajectory point 指定 positions[, velocities[, accelerations]] 或 positions[, effort]。
> 所有值的順序必須與 joint_names 相同。

**關節名稱：**
```
['arm_1_joint', 'arm_2_joint', 'gripper_joint']
```

### 手臂關節角度限制

| 關節 | 範圍 | 注意事項 |
|------|------|----------|
| arm_1_joint | 30° ～ 210° | 手臂可能會碰到地板 |
| arm_2_joint | 0° ～ 240° | 手臂可能會碰到地板 |
| gripper_joint | 168° ～ 240° | **240° = 全開；168° = 接近闔起**，⚠️ 嚴禁低於 168°，否則過夾會燒馬達 |

> 夾住東西後 0.5 秒會自動退 2° 以防止燒壞馬達。

**範例指令：**
```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_1_joint','arm_2_joint','gripper_joint'], \
    points: [{positions: [1.0, 1.1, 2.0], time_from_start: {sec: 1}}]}"
```

---

## 導航（里程計）

| 項目 | 內容 |
|------|------|
| Topic | `/base_controller/odom` |
| Type | `nav_msgs/msg/Odometry` |

**Message 格式：**
```
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose    # 相對 world frame 的估測位姿
geometry_msgs/TwistWithCovariance twist  # 相對 child_frame_id 的線速度與角速度
```

> ⚠️ **注意：** Controller 不會發布 `odom -> base_link` 的 TF，需要自行實作。
> 可利用 `/scan`（LiDAR）Topic 輔助計算。

---

## ros2_control 參數

設定檔路徑：`wildbot_workspace/docker/compose/configs/controllers.yaml`

**重要參數：**
- `wheel_separation_multiplier: 2.21` — 需要透過現場測試調整（摩擦力不同），以得到正確的轉圈速度（angular-z vel）
- `enable_odom_tf: false` — odom TF 需自行發布

---

## 程式實作

- **ROS node 放置位置：** `wildbot_workspace/workspaces/src`
- **新增 ROS2 套件：** 修改 `wildbot_workspace/Dockerfile`，在指定區段加入 apt 套件名稱

```dockerfile
apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-<your-package> \
    ...
```
