"""Fuse LiDAR sectors and camera depth for reactive obstacle clearance."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass
class ObstacleGuardResult:
    min_clearance_m: float
    front_clearance_m: float
    left_clearance_m: float
    right_clearance_m: float
    speed_scale: float
    block_cmd: Optional[str] = None  # STOP | CLOCKWISE_ROTATION | COUNTERCLOCKWISE_ROTATION
    sensor_front_m: float = float("inf")
    sensor_left_m: float = float("inf")
    sensor_right_m: float = float("inf")


def _finite_clearance(value: Optional[float], unknown_as_blocked_m: float = 0.0) -> float:
    """Missing/inf LiDAR → treat as blocked (not wide open)."""
    if value is None or not math.isfinite(value):
        return unknown_as_blocked_m
    return float(value)


def _valid_depth_m(value: float, min_m: float, max_m: float) -> bool:
    return (
        value > 0.0
        and value != -1.0
        and math.isfinite(value)
        and min_m <= value <= max_m
    )


def _valid_lidar_m(value: float, min_m: float, max_m: float) -> bool:
    return math.isfinite(value) and min_m <= value <= max_m


def _min_valid(values: Sequence[float], valid_fn) -> Optional[float]:
    ok = [v for v in values if valid_fn(v)]
    return min(ok) if ok else None


class ObstacleGuard:
    """Combine LiDAR + depth (multi-point or sector) into speed_scale / block_cmd."""

    # camera x_multi_depth: 20 points, indices 0-6 left, 7-12 front, 13-19 right
    DEPTH_LEFT_SLICE = slice(0, 7)
    DEPTH_FRONT_SLICE = slice(7, 13)
    DEPTH_RIGHT_SLICE = slice(13, 20)

    # /obstacle/sector_min_depth layout (5 values)
    SECTOR_FRONT = 0
    SECTOR_FRONT_LEFT = 1
    SECTOR_LEFT = 2
    SECTOR_FRONT_RIGHT = 3
    SECTOR_RIGHT = 4

    def __init__(
        self,
        stop_m: float = 0.25,
        slow_m: float = 0.45,
        side_stop_m: float = 0.30,
        depth_min_m: float = 0.40,
        depth_max_m: float = 3.0,
        lidar_min_m: float = 0.12,
        lidar_max_m: float = 2.8,
        side_clearance_bias_m: float = 0.08,
        side_block_min_asymmetry_m: float = 0.15,
    ):
        self.stop_m = stop_m
        self.slow_m = max(slow_m, stop_m + 0.05)
        self.side_stop_m = side_stop_m
        self.side_block_min_asymmetry_m = side_block_min_asymmetry_m
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m
        self.lidar_min_m = lidar_min_m
        self.lidar_max_m = lidar_max_m
        self.side_clearance_bias_m = side_clearance_bias_m

    @classmethod
    def from_profile(cls, unity: bool) -> "ObstacleGuard":
        if unity:
            return cls(
                stop_m=0.60,
                slow_m=1.20,
                side_stop_m=0.32,
                depth_min_m=0.40,
                depth_max_m=15.0,
                lidar_min_m=0.25,
                lidar_max_m=8.0,
            )
        return cls()

    def _depth_valid(self, v: float) -> bool:
        return _valid_depth_m(v, self.depth_min_m, self.depth_max_m)

    def _lidar_valid(self, v: float) -> bool:
        return _valid_lidar_m(v, self.lidar_min_m, self.lidar_max_m)

    def _lidar_sector_mins_from_tuple(
        self,
        lidar_sectors: Optional[Tuple[Optional[float], Optional[float], Optional[float]]],
    ) -> tuple[Optional[float], Optional[float], Optional[float]]:
        if lidar_sectors is None:
            return None, None, None
        front, left, right = lidar_sectors
        out_f = front if front is not None and self._lidar_valid(front) else None
        out_l = left if left is not None and self._lidar_valid(left) else None
        out_r = right if right is not None and self._lidar_valid(right) else None
        return out_f, out_l, out_r

    def _depth_sector_mins(
        self,
        multi_depth: Optional[List[float]],
        sector_depth: Optional[List[float]],
    ) -> tuple[Optional[float], Optional[float], Optional[float]]:
        if sector_depth and len(sector_depth) >= 5:
            front = sector_depth[self.SECTOR_FRONT]
            left = min(
                sector_depth[self.SECTOR_LEFT],
                sector_depth[self.SECTOR_FRONT_LEFT],
            )
            right = min(
                sector_depth[self.SECTOR_RIGHT],
                sector_depth[self.SECTOR_FRONT_RIGHT],
            )
            vals_f = [front] if self._depth_valid(front) else []
            vals_l = [left] if self._depth_valid(left) else []
            vals_r = [right] if self._depth_valid(right) else []
            return (
                min(vals_f) if vals_f else None,
                min(vals_l) if vals_l else None,
                min(vals_r) if vals_r else None,
            )

        if not multi_depth or len(multi_depth) < 20:
            return None, None, None
        return (
            _min_valid(multi_depth[self.DEPTH_FRONT_SLICE], self._depth_valid),
            _min_valid(multi_depth[self.DEPTH_LEFT_SLICE], self._depth_valid),
            _min_valid(multi_depth[self.DEPTH_RIGHT_SLICE], self._depth_valid),
        )

    def evaluate(
        self,
        lidar_sectors: Optional[
            Tuple[Optional[float], Optional[float], Optional[float]]
        ] = None,
        multi_depth: Optional[List[float]] = None,
        sector_depth: Optional[List[float]] = None,
        lidar_flat: Optional[List[float]] = None,
        approach_target_depth_m: Optional[float] = None,
        approach_mode: bool = False,
    ) -> ObstacleGuardResult:
        del lidar_flat  # legacy arg, unused
        lf, ll, lr = self._lidar_sector_mins_from_tuple(lidar_sectors)
        df, dl, dr = self._depth_sector_mins(multi_depth, sector_depth)

        def fuse(a: Optional[float], b: Optional[float]) -> Optional[float]:
            vals = [
                v
                for v in (a, b)
                if v is not None and math.isfinite(v)
            ]
            return min(vals) if vals else None

        def fuse_side(lidar_v: Optional[float], depth_v: Optional[float]) -> Optional[float]:
            """Side clearance: prefer LiDAR (walls); depth often misreads floor/body."""
            if lidar_v is not None and self._lidar_valid(lidar_v):
                return lidar_v
            if depth_v is not None and self._depth_valid(depth_v):
                return depth_v
            return None

        sensor_front = fuse(lf, df)
        sensor_left = fuse_side(ll, dl)
        sensor_right = fuse_side(lr, dr)

        front_eff = _finite_clearance(sensor_front)
        left_eff = _finite_clearance(sensor_left)
        right_eff = _finite_clearance(sensor_right)

        # 接近熊時可抬高「顯示用 front」，但牆壁安全判斷仍用 sensor 原始值
        front = front_eff
        if (
            approach_target_depth_m is not None
            and approach_target_depth_m > self.stop_m
            and sensor_front is not None
            and sensor_front > self.stop_m
        ):
            target_floor = max(
                self.stop_m * 0.85,
                approach_target_depth_m * 0.42,
            )
            if front < target_floor:
                front = target_floor

        min_clearance = min(front_eff, left_eff, right_eff)

        speed_scale = 1.0
        block_cmd = None

        side_asym = abs(left_eff - right_eff)
        side_bias = max(self.side_clearance_bias_m, self.side_block_min_asymmetry_m)

        # 安全停車：用 sensor 原始距離，避免把牆/窄通道當成可前進
        hard_front_block = sensor_front is not None and sensor_front < self.stop_m
        hard_side_block = (
            min(left_eff, right_eff) < self.stop_m + 0.03
            and sensor_front is not None
            and sensor_front < self.slow_m
        )

        if hard_front_block:
            speed_scale = 0.0
            block_cmd = "STOP"
            if not approach_mode:
                if left_eff + side_bias < right_eff and side_asym >= side_bias:
                    block_cmd = "CLOCKWISE_ROTATION"
                elif right_eff + side_bias < left_eff and side_asym >= side_bias:
                    block_cmd = "COUNTERCLOCKWISE_ROTATION"
        elif sensor_front is not None and sensor_front < self.slow_m:
            speed_scale = max(
                0.0,
                min(
                    1.0,
                    (sensor_front - self.stop_m) / (self.slow_m - self.stop_m),
                ),
            )
        elif hard_side_block:
            speed_scale = min(speed_scale, 0.15)
            block_cmd = "STOP"
        elif not approach_mode and (
            left_eff < self.side_stop_m
            and right_eff >= left_eff + side_bias
            and side_asym >= side_bias
            and sensor_front is not None
            and sensor_front >= self.stop_m
        ):
            speed_scale = min(speed_scale, 0.5)
            block_cmd = "CLOCKWISE_ROTATION"
        elif not approach_mode and (
            right_eff < self.side_stop_m
            and left_eff >= right_eff + side_bias
            and side_asym >= side_bias
            and sensor_front is not None
            and sensor_front >= self.stop_m
        ):
            speed_scale = min(speed_scale, 0.5)
            block_cmd = "COUNTERCLOCKWISE_ROTATION"

        return ObstacleGuardResult(
            min_clearance_m=min_clearance,
            front_clearance_m=front,
            left_clearance_m=left_eff,
            right_clearance_m=right_eff,
            speed_scale=speed_scale,
            block_cmd=block_cmd,
            sensor_front_m=_finite_clearance(sensor_front, float("inf")),
            sensor_left_m=_finite_clearance(sensor_left, float("inf")),
            sensor_right_m=_finite_clearance(sensor_right, float("inf")),
        )

    def _corridor_forward_scale(
        self,
        front_c: float,
        left_c: float,
        right_c: float,
        approach_mode: bool = False,
    ) -> float:
        """
        Scale forward speed in narrow corridors.
        Only hard-stop when front blocked or both sides extremely tight.
        """
        if math.isfinite(front_c) and front_c < self.stop_m + 0.04:
            return 0.0

        side_min = min(left_c, right_c)
        # Unknown side (0 from missing data) — do not hard-block in approach
        if side_min <= 1e-6 and approach_mode:
            return 1.0

        tight = self.side_stop_m + (0.14 if approach_mode else 0.08)
        if side_min >= tight:
            return 1.0
        if side_min < self.stop_m + 0.02:
            return 0.0

        ratio = side_min / max(tight, 1e-3)
        return max(0.35 if approach_mode else 0.20, min(1.0, ratio))

    def apply_to_wheel_cmd(
        self, wheel_cmd: List[float], result: ObstacleGuardResult,
        approach_mode: bool = False,
    ) -> List[float]:
        """Scale forward component; damp yaw toward blocked sides."""
        left_w, right_w = float(wheel_cmd[0]), float(wheel_cmd[1])
        fwd = 0.5 * (left_w + right_w)
        yaw = 0.5 * (left_w - right_w)

        left_c = _finite_clearance(result.sensor_left_m, float("inf"))
        right_c = _finite_clearance(result.sensor_right_m, float("inf"))
        front_c = _finite_clearance(result.sensor_front_m, float("inf"))
        if not math.isfinite(left_c):
            left_c = result.left_clearance_m
        if not math.isfinite(right_c):
            right_c = result.right_clearance_m

        # 不要往窄側轉（yaw>0 = 右轉，yaw<0 = 左轉）
        if yaw > 0.0 and math.isfinite(right_c) and right_c < self.side_stop_m + 0.10:
            yaw *= max(0.0, right_c / max(self.side_stop_m + 0.10, 1e-3))
        elif yaw < 0.0 and math.isfinite(left_c) and left_c < self.side_stop_m + 0.10:
            yaw *= max(0.0, left_c / max(self.side_stop_m + 0.10, 1e-3))

        if fwd > 0.0:
            fwd *= result.speed_scale
            corridor_scale = self._corridor_forward_scale(
                front_c if math.isfinite(front_c) else result.front_clearance_m,
                left_c if math.isfinite(left_c) else self.side_stop_m,
                right_c if math.isfinite(right_c) else self.side_stop_m,
                approach_mode=approach_mode,
            )
            fwd *= corridor_scale

        wheel_lim = 480.0
        scaled_left = max(-wheel_lim, min(wheel_lim, fwd + yaw))
        scaled_right = max(-wheel_lim, min(wheel_lim, fwd - yaw))
        return [scaled_left, scaled_right, scaled_left, scaled_right]


def get_lidar_sector_minimums(data_processor):
    """Return (front_min, left_min, right_min) from /scan, or (None, None, None)."""
    try:
        return data_processor.get_lidar_sector_minimums()
    except (AttributeError, TypeError, IndexError):
        return None, None, None
