# -*- coding: utf-8 -*-

# config.py

"""
實體機器人速度設定（TwistStamped 格式）
格式：(linear_x [m/s], angular_z [rad/s])

速度限制來自 controllers.yaml：
  linear.x 最大 0.546 m/s
  angular.z 最大 3.983 rad/s

⚠️ 速度值需在實車上測試後微調（wheel_separation_multiplier 影響轉速）
"""

ACTION_MAPPINGS = {
    "FORWARD":                          (0.3,   0.0),
    "FORWARD_SLOW":                     (0.15,  0.0),
    "BACKWARD":                         (-0.15, 0.0),
    "BACKWARD_SLOW":                    (-0.08, 0.0),
    "CLOCKWISE_ROTATION":               (0.0,  -1.2),
    "CLOCKWISE_ROTATION_SLOW":          (0.0,  -0.6),
    "CLOCKWISE_ROTATION_MEDIAN":        (0.0,  -0.9),
    "COUNTERCLOCKWISE_ROTATION":        (0.0,   1.2),
    "COUNTERCLOCKWISE_ROTATION_SLOW":   (0.0,   0.6),
    "COUNTERCLOCKWISE_ROTATION_MEDIAN": (0.0,   0.9),
    "LEFT_FRONT":                       (0.2,   0.8),
    "RIGHT_FRONT":                      (0.2,  -0.8),
    "RIGHT_SHIFT":                      (0.0,  -1.5),
    "LEFT_SHIFT":                       (0.0,   1.5),
    "STOP":                             (0.0,   0.0),
}
