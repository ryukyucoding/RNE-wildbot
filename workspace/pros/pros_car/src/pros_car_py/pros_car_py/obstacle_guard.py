"""Fuse LiDAR sectors and camera depth for reactive obstacle clearance."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass
class ObstacleSourceDebug:
    """Per-sensor breakdown for obstacle clearance (diagnostics only)."""

    lidar_front: Optional[float] = None
    lidar_left: Optional[float] = None
    lidar_right: Optional[float] = None
    depth_front: Optional[float] = None
    depth_front_left: Optional[float] = None
    depth_left: Optional[float] = None
    depth_front_right: Optional[float] = None
    depth_right: Optional[float] = None
    depth_rear: Optional[float] = None
    depth_left_combined: Optional[float] = None
    depth_left_filtered: bool = False
    lidar_left_filtered: bool = False
    lidar_right_filtered: bool = False
    left_winner: str = "none"
    corridor_scale: float = 1.0

    def format_compact(self) -> str:
        def _m(v: Optional[float]) -> str:
            return f"{v:.2f}" if v is not None and math.isfinite(v) else "n/a"

        parts = [
            f"L({_m(self.lidar_left)})",
            f"Dfl({_m(self.depth_front_left)})",
            f"Dl({_m(self.depth_left)})",
            f"Dr({_m(self.depth_right)})",
            f"→left={self.left_winner}",
        ]
        if self.depth_left_filtered:
            parts.append("depth_left_filtered")
        if self.lidar_left_filtered:
            parts.append("lidar_left_filtered")
        if self.lidar_right_filtered:
            parts.append("lidar_right_filtered")
        parts.append(f"corridor={self.corridor_scale:.2f}")
        return " ".join(parts)


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
    sensor_rear_m: float = float("inf")
    rear_clearance_m: float = float("inf")
    backward_speed_scale: float = 1.0
    source_debug: Optional[ObstacleSourceDebug] = None


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
    SECTOR_REAR = 5

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
                stop_m=0.35,
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

    @staticmethod
    def _diagonal_depth_mins(
        sector_depth: Optional[List[float]],
        depth_valid_fn,
    ) -> tuple[Optional[float], Optional[float]]:
        if not sector_depth or len(sector_depth) < 5:
            return None, None
        fl = sector_depth[ObstacleGuard.SECTOR_FRONT_LEFT]
        fr = sector_depth[ObstacleGuard.SECTOR_FRONT_RIGHT]
        out_l = fl if depth_valid_fn(fl) else None
        out_r = fr if depth_valid_fn(fr) else None
        return out_l, out_r

    @staticmethod
    def _pick_open_side_rotation(
        left_eff: float,
        right_eff: float,
        side_bias: float,
        side_asym: float,
    ) -> Optional[str]:
        if left_eff + side_bias < right_eff and side_asym >= side_bias:
            return "CLOCKWISE_ROTATION"
        if right_eff + side_bias < left_eff and side_asym >= side_bias:
            return "COUNTERCLOCKWISE_ROTATION"
        if left_eff >= right_eff + 0.04:
            return "CLOCKWISE_ROTATION"
        if right_eff >= left_eff + 0.04:
            return "COUNTERCLOCKWISE_ROTATION"
        return None

    def must_override_visual_yaw(self, result: ObstacleGuardResult) -> bool:
        """True when visual-servo yaw must not suppress obstacle escape."""
        sf = result.sensor_front_m
        if math.isfinite(sf) and sf < self.stop_m + 0.05:
            return True
        if result.block_cmd in ("CLOCKWISE_ROTATION", "COUNTERCLOCKWISE_ROTATION"):
            if result.speed_scale <= 0.40:
                return True
        if result.block_cmd == "STOP" and result.speed_scale <= 0.05:
            return True
        return False

    def _escape_yaw_wheel(self, block_cmd: Optional[str], approach_mode: bool) -> float:
        mag = 95.0 if approach_mode else 130.0
        if block_cmd == "CLOCKWISE_ROTATION":
            return mag
        if block_cmd == "COUNTERCLOCKWISE_ROTATION":
            return -mag
        return 0.0

    def _depth_only_floor_false_positive(
        self,
        sensor_front: Optional[float],
        sensor_left: Optional[float],
        sensor_right: Optional[float],
        lidar_available: bool,
        approach_target_depth_m: Optional[float],
        approach_mode: bool,
    ) -> bool:
        """
        Without LiDAR, sector depth bands (lower 65% of image) often read floor
        at ~0.45–0.55 m on all sides — not real walls.
        """
        if not approach_mode or lidar_available:
            return False
        vals = [
            v
            for v in (sensor_front, sensor_left, sensor_right)
            if v is not None and math.isfinite(v)
        ]
        if len(vals) < 2:
            return False
        vmin, vmax = min(vals), max(vals)
        if vmax - vmin > 0.15:
            return False
        if (
            approach_target_depth_m is not None
            and approach_target_depth_m > vmin + 0.25
            and vmin < self.stop_m + 0.15
        ):
            return True
        if vmax - vmin < 0.10 and vmin < self.stop_m + 0.05:
            return True
        return False

    @staticmethod
    def _depth_side_likely_rear_body(
        depth_side_m: Optional[float],
        approach_target_depth_m: Optional[float],
        approach_mode: bool,
    ) -> bool:
        """Depth side hit much closer than YOLO target → often ground near rear wheels."""
        if not approach_mode or depth_side_m is None or not math.isfinite(depth_side_m):
            return False
        if approach_target_depth_m is None or approach_target_depth_m <= 0.0:
            return False
        return (
            depth_side_m < 0.55
            and approach_target_depth_m > depth_side_m + 0.35
        )

    @staticmethod
    def _lidar_side_depth_disagreement(
        lidar_side_m: Optional[float],
        depth_side_m: Optional[float],
        approach_target_depth_m: Optional[float],
        approach_mode: bool,
    ) -> bool:
        """
        LiDAR side glancing hit (e.g. 38° ground/body) while depth/YOLO see open space.
        """
        if not approach_mode:
            return False
        if lidar_side_m is None or depth_side_m is None:
            return False
        if not (math.isfinite(lidar_side_m) and math.isfinite(depth_side_m)):
            return False
        if lidar_side_m >= 0.55 or depth_side_m < 0.50:
            return False
        if depth_side_m <= lidar_side_m + 0.20:
            return False
        if (
            approach_target_depth_m is not None
            and approach_target_depth_m > 0.0
            and approach_target_depth_m <= lidar_side_m + 0.25
        ):
            return False
        return True

    @staticmethod
    def _approach_side_open_by_depth(
        lidar_side_m: Optional[float],
        depth_side_m: Optional[float],
        approach_target_depth_m: Optional[float],
        approach_mode: bool,
    ) -> bool:
        """Approach: depth says side is open while LiDAR reads a near ground ring."""
        if not approach_mode or depth_side_m is None or not math.isfinite(depth_side_m):
            return False
        if lidar_side_m is None or not math.isfinite(lidar_side_m):
            return False
        if lidar_side_m >= 0.55 or depth_side_m < 0.50:
            return False
        if depth_side_m <= lidar_side_m + 0.18:
            return False
        if (
            approach_target_depth_m is not None
            and approach_target_depth_m > 0.0
            and approach_target_depth_m <= lidar_side_m + 0.25
        ):
            return False
        return True

    @staticmethod
    def _side_eff_clearance(
        sensor_side: Optional[float],
        approach_mode: bool,
    ) -> float:
        if sensor_side is not None and math.isfinite(sensor_side):
            return float(sensor_side)
        return float("inf") if approach_mode else 0.0

    def _rear_clearance_from_sensors(
        self,
        lidar_rear: Optional[float],
        depth_rear: Optional[float],
    ) -> tuple[Optional[float], float]:
        vals: list[float] = []
        if lidar_rear is not None and self._lidar_valid(lidar_rear):
            vals.append(lidar_rear)
        if depth_rear is not None and self._depth_valid(depth_rear):
            vals.append(depth_rear)
        if not vals:
            return None, 1.0
        rear_m = min(vals)
        if rear_m < self.stop_m:
            return rear_m, 0.0
        if rear_m < self.slow_m:
            scale = max(
                0.0,
                min(1.0, (rear_m - self.stop_m) / (self.slow_m - self.stop_m)),
            )
            return rear_m, scale
        return rear_m, 1.0

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
        speed_scale_floor: Optional[float] = None,
        lidar_rear_m: Optional[float] = None,
    ) -> ObstacleGuardResult:
        del lidar_flat  # legacy arg, unused
        lf, ll, lr = self._lidar_sector_mins_from_tuple(lidar_sectors)
        ll_raw = ll
        lr_raw = lr
        lidar_left_filtered = False
        lidar_right_filtered = False
        df, dl, dr = self._depth_sector_mins(multi_depth, sector_depth)
        dl_before_filter = dl
        dr_before_filter = dr
        depth_left_filtered = False
        diag_left, diag_right = self._diagonal_depth_mins(sector_depth, self._depth_valid)

        depth_front_raw: Optional[float] = None
        depth_front_left_raw: Optional[float] = None
        depth_left_raw: Optional[float] = None
        depth_front_right_raw: Optional[float] = None
        depth_right_raw: Optional[float] = None
        depth_rear_raw: Optional[float] = None
        if sector_depth and len(sector_depth) >= 5:
            if self._depth_valid(sector_depth[self.SECTOR_FRONT]):
                depth_front_raw = sector_depth[self.SECTOR_FRONT]
            if self._depth_valid(sector_depth[self.SECTOR_FRONT_LEFT]):
                depth_front_left_raw = sector_depth[self.SECTOR_FRONT_LEFT]
            if self._depth_valid(sector_depth[self.SECTOR_LEFT]):
                depth_left_raw = sector_depth[self.SECTOR_LEFT]
            if self._depth_valid(sector_depth[self.SECTOR_FRONT_RIGHT]):
                depth_front_right_raw = sector_depth[self.SECTOR_FRONT_RIGHT]
            if self._depth_valid(sector_depth[self.SECTOR_RIGHT]):
                depth_right_raw = sector_depth[self.SECTOR_RIGHT]
        if sector_depth and len(sector_depth) >= 6:
            dr_val = sector_depth[self.SECTOR_REAR]
            if self._depth_valid(dr_val):
                depth_rear_raw = dr_val

        if approach_mode and self._depth_side_likely_rear_body(
            dl, approach_target_depth_m, approach_mode
        ):
            depth_left_filtered = True
            dl = None
        if approach_mode and self._depth_side_likely_rear_body(
            dr, approach_target_depth_m, approach_mode
        ):
            dr = None

        def fuse(a: Optional[float], b: Optional[float]) -> Optional[float]:
            vals = [
                v
                for v in (a, b)
                if v is not None and math.isfinite(v)
            ]
            return min(vals) if vals else None

        def fuse_front(lidar_v: Optional[float], depth_v: Optional[float]) -> Optional[float]:
            if (
                approach_mode
                and self._lidar_side_depth_disagreement(
                    lidar_v, depth_v, approach_target_depth_m, approach_mode
                )
            ):
                return depth_v
            return fuse(lidar_v, depth_v)

        def fuse_side(lidar_v: Optional[float], depth_v: Optional[float]) -> Optional[float]:
            """Side clearance: in approach, trust depth when LiDAR reads a ground ring."""
            if approach_mode and self._approach_side_open_by_depth(
                lidar_v, depth_v, approach_target_depth_m, approach_mode
            ):
                return depth_v
            use_lidar = lidar_v
            if approach_mode and self._lidar_side_depth_disagreement(
                lidar_v, depth_v, approach_target_depth_m, approach_mode
            ):
                use_lidar = None
            vals: list[float] = []
            if use_lidar is not None and self._lidar_valid(use_lidar):
                vals.append(use_lidar)
            if depth_v is not None and self._depth_valid(depth_v):
                vals.append(depth_v)
            if not vals:
                return None
            if approach_mode:
                return min(vals)
            if use_lidar is not None and self._lidar_valid(use_lidar):
                return use_lidar
            return depth_v

        sensor_front = fuse_front(lf, df)
        sensor_left = fuse_side(ll, dl)
        sensor_right = fuse_side(lr, dr)

        if (
            approach_mode
            and ll_raw is not None
            and sensor_left is not None
            and dl is not None
            and abs(sensor_left - dl) < 0.08
            and ll_raw < dl - 0.20
        ):
            lidar_left_filtered = True
        if (
            approach_mode
            and lr_raw is not None
            and sensor_right is not None
            and dr is not None
            and abs(sensor_right - dr) < 0.08
            and lr_raw < dr - 0.20
        ):
            lidar_right_filtered = True

        front_eff = _finite_clearance(sensor_front)
        left_eff = self._side_eff_clearance(sensor_left, approach_mode)
        right_eff = self._side_eff_clearance(sensor_right, approach_mode)

        # 接近熊時可抬高「顯示用 front」，但牆壁安全判斷仍用 sensor 原始值
        front = front_eff
        if (
            approach_mode
            and approach_target_depth_m is not None
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
        diag_thresh = self.side_stop_m + (0.10 if approach_mode else 0.06)

        lidar_available = any(x is not None for x in (lf, ll, lr))
        depth_floor_fp = self._depth_only_floor_false_positive(
            sensor_front,
            sensor_left,
            sensor_right,
            lidar_available,
            approach_target_depth_m,
            approach_mode,
        )

        # 安全停車：用 sensor 原始距離，避免把牆/窄通道當成可前進
        hard_front_block = sensor_front is not None and sensor_front < self.stop_m
        hard_side_block = (
            min(left_eff, right_eff) < self.stop_m + 0.03
            and sensor_front is not None
            and sensor_front < self.slow_m
        )

        if depth_floor_fp:
            hard_front_block = False
            hard_side_block = False

        if hard_front_block:
            speed_scale = 0.0
            block_cmd = "STOP"
            escape = self._pick_open_side_rotation(
                left_eff, right_eff, side_bias, side_asym
            )
            if escape is not None:
                block_cmd = escape
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
        elif (
            left_eff < self.side_stop_m
            and right_eff >= left_eff + side_bias
            and side_asym >= side_bias
            and sensor_front is not None
            and sensor_front >= self.stop_m
            and not (
                approach_mode
                and approach_target_depth_m is not None
                and approach_target_depth_m > left_eff + 0.75
            )
        ):
            speed_scale = min(speed_scale, 0.5 if approach_mode else 0.5)
            block_cmd = "CLOCKWISE_ROTATION"
        elif (
            right_eff < self.side_stop_m
            and left_eff >= right_eff + side_bias
            and side_asym >= side_bias
            and sensor_front is not None
            and sensor_front >= self.stop_m
            and not (
                approach_mode
                and approach_target_depth_m is not None
                and approach_target_depth_m > right_eff + 0.75
            )
        ):
            speed_scale = min(speed_scale, 0.5 if approach_mode else 0.5)
            block_cmd = "COUNTERCLOCKWISE_ROTATION"

        # 對角深度（front_left / front_right）：補 LiDAR 掃不到的矮障礙（Unity 橋等）
        if (
            block_cmd is None
            and not depth_floor_fp
            and sensor_front is not None
            and sensor_front >= self.stop_m
        ):
            if (
                diag_left is not None
                and diag_left < diag_thresh
                and (diag_right is None or diag_left + 0.05 < diag_right)
            ):
                speed_scale = min(speed_scale, 0.45 if approach_mode else 0.35)
                block_cmd = "CLOCKWISE_ROTATION"
            elif (
                diag_right is not None
                and diag_right < diag_thresh
                and (diag_left is None or diag_right + 0.05 < diag_left)
            ):
                speed_scale = min(speed_scale, 0.45 if approach_mode else 0.35)
                block_cmd = "COUNTERCLOCKWISE_ROTATION"

        if (
            approach_mode
            and speed_scale_floor is not None
            and speed_scale_floor > 0.0
            and not hard_front_block
        ):
            speed_scale = max(speed_scale, min(1.0, float(speed_scale_floor)))

        sensor_rear, backward_speed_scale = self._rear_clearance_from_sensors(
            lidar_rear_m,
            depth_rear_raw,
        )

        left_winner = "none"
        if sensor_left is not None:
            lidar_ok = ll is not None and self._lidar_valid(ll)
            depth_ok = dl is not None and self._depth_valid(dl)
            if lidar_ok and depth_ok:
                left_winner = (
                    "lidar"
                    if ll <= dl + 1e-4
                    else "depth"
                )
            elif lidar_ok:
                left_winner = "lidar"
            elif depth_ok:
                left_winner = "depth"

        corridor_scale = self._corridor_forward_scale(
            _finite_clearance(sensor_front, float("inf")),
            _finite_clearance(sensor_left, float("inf")),
            _finite_clearance(sensor_right, float("inf")),
            approach_mode=approach_mode,
        )

        source_debug = ObstacleSourceDebug(
            lidar_front=lf,
            lidar_left=ll,
            lidar_right=lr,
            depth_front=depth_front_raw if depth_front_raw is not None else df,
            depth_front_left=depth_front_left_raw,
            depth_left=depth_left_raw,
            depth_front_right=depth_front_right_raw,
            depth_right=depth_right_raw if depth_right_raw is not None else dr,
            depth_rear=depth_rear_raw,
            depth_left_combined=dl_before_filter,
            depth_left_filtered=depth_left_filtered,
            lidar_left_filtered=lidar_left_filtered,
            lidar_right_filtered=lidar_right_filtered,
            left_winner=left_winner,
            corridor_scale=corridor_scale,
        )

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
            sensor_rear_m=_finite_clearance(sensor_rear, float("inf")),
            rear_clearance_m=_finite_clearance(sensor_rear, float("inf")),
            backward_speed_scale=backward_speed_scale,
            source_debug=source_debug,
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

        if not math.isfinite(left_c):
            left_c = float("inf")
        if not math.isfinite(right_c):
            right_c = float("inf")
        side_min = min(left_c, right_c)
        # Unknown side (0 from missing data) — do not hard-block in approach
        if side_min <= 1e-6 and approach_mode:
            return 1.0
        if not math.isfinite(side_min):
            return 1.0

        tight = self.side_stop_m + (0.14 if approach_mode else 0.08)
        if side_min >= tight:
            return 1.0
        if side_min < self.stop_m + 0.02:
            return 0.0

        ratio = side_min / max(tight, 1e-3)
        return max(0.35 if approach_mode else 0.20, min(1.0, ratio))

    def apply_to_wheel_cmd(
        self,
        wheel_cmd: List[float],
        result: ObstacleGuardResult,
        approach_mode: bool = False,
        prefer_visual_yaw: bool = False,
    ) -> List[float]:
        """Scale forward component; damp or blend yaw toward open side."""
        left_w, right_w = float(wheel_cmd[0]), float(wheel_cmd[1])
        fwd = 0.5 * (left_w + right_w)
        yaw = 0.5 * (left_w - right_w)

        left_c = result.left_clearance_m
        right_c = result.right_clearance_m
        front_c = _finite_clearance(result.sensor_front_m, float("inf"))
        if not approach_mode:
            left_c = _finite_clearance(result.sensor_left_m, float("inf"))
            right_c = _finite_clearance(result.sensor_right_m, float("inf"))
            if not math.isfinite(left_c):
                left_c = result.left_clearance_m
            if not math.isfinite(right_c):
                right_c = result.right_clearance_m

        escape_yaw = self._escape_yaw_wheel(result.block_cmd, approach_mode)
        override_visual = self.must_override_visual_yaw(result)

        if escape_yaw != 0.0:
            if prefer_visual_yaw and not override_visual:
                blend = max(0.35, min(0.75, 1.0 - result.speed_scale))
                yaw = (1.0 - blend) * yaw + blend * escape_yaw
            else:
                if result.speed_scale <= 0.10:
                    fwd = 0.0
                else:
                    fwd *= result.speed_scale
                if abs(yaw) < abs(escape_yaw) * 0.45:
                    yaw = escape_yaw
                else:
                    yaw = 0.55 * escape_yaw + 0.45 * yaw

        # 不要往窄側轉（yaw>0 = 右轉，yaw<0 = 左轉）
        if yaw > 0.0 and math.isfinite(right_c) and right_c < self.side_stop_m + 0.10:
            yaw *= max(0.0, right_c / max(self.side_stop_m + 0.10, 1e-3))
        elif yaw < 0.0 and math.isfinite(left_c) and left_c < self.side_stop_m + 0.10:
            yaw *= max(0.0, left_c / max(self.side_stop_m + 0.10, 1e-3))

        if fwd > 0.0:
            if escape_yaw == 0.0 or (prefer_visual_yaw and not override_visual):
                fwd *= result.speed_scale
            corridor_scale = self._corridor_forward_scale(
                front_c if math.isfinite(front_c) else result.front_clearance_m,
                left_c if math.isfinite(left_c) else self.side_stop_m,
                right_c if math.isfinite(right_c) else self.side_stop_m,
                approach_mode=approach_mode,
            )
            fwd *= corridor_scale
        elif fwd < 0.0:
            bscale = max(0.0, min(1.0, float(result.backward_speed_scale)))
            fwd *= bscale
            if bscale <= 0.05:
                fwd = 0.0

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


def get_lidar_rear_minimum(data_processor):
    """Return rear minimum range (m) behind rear-axle plane, or None."""
    try:
        return data_processor.get_lidar_rear_minimum()
    except (AttributeError, TypeError, IndexError):
        return None
