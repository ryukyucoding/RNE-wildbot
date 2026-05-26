#!/usr/bin/env python3
"""Publish fixed-length /scan from /scan_tmp (NaN/inf → range_max)."""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSanitizeNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_sanitize")
        self.declare_parameter("input_topic", "/scan_tmp")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("warmup_scans", 15)
        in_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        out_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self._warmup_target = max(
            5,
            int(
                self.get_parameter("warmup_scans")
                .get_parameter_value()
                .integer_value
            ),
        )
        self._pub = self.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, in_topic, self._cb, qos_profile_sensor_data
        )
        self._warmup_seen = 0
        self._warmup_max = 0
        self._fixed_beam_count: int | None = None
        self.get_logger().info(
            f"Sanitizing {in_topic} → {out_topic} "
            f"(warmup={self._warmup_target} scans before publish)"
        )

    @staticmethod
    def _clean(values, fallback: float):
        out = []
        for v in values:
            if v is None or not math.isfinite(float(v)):
                out.append(float(fallback))
            else:
                out.append(float(v))
        return out

    @staticmethod
    def _expected_beam_count(msg: LaserScan) -> int:
        if msg.angle_increment <= 0.0:
            return len(msg.ranges)
        span = float(msg.angle_max) - float(msg.angle_min)
        if span < 0.0:
            span += 2.0 * math.pi
        return max(1, int(round(span / float(msg.angle_increment))) + 1)

    def _fit_beams(self, msg: LaserScan, expected: int, fallback: float) -> LaserScan:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.angle_max = out.angle_min + (expected - 1) * out.angle_increment

        ranges = self._clean(msg.ranges, fallback)
        if len(ranges) < expected:
            ranges.extend([fallback] * (expected - len(ranges)))
        elif len(ranges) > expected:
            ranges = ranges[:expected]
        out.ranges = ranges

        if msg.intensities:
            intensities = self._clean(msg.intensities, 0.0)
            if len(intensities) < expected:
                intensities.extend([0.0] * (expected - len(intensities)))
            elif len(intensities) > expected:
                intensities = intensities[:expected]
            out.intensities = intensities
        return out

    def _cb(self, msg: LaserScan) -> None:
        fallback = float(msg.range_max) if msg.range_max > 0.0 else 20.0
        candidate = max(len(msg.ranges), self._expected_beam_count(msg))

        if self._fixed_beam_count is None:
            self._warmup_max = max(self._warmup_max, candidate)
            self._warmup_seen += 1
            if self._warmup_seen < self._warmup_target:
                return
            self._fixed_beam_count = self._warmup_max
            self.get_logger().info(
                f"Locked /scan beam count = {self._fixed_beam_count} "
                f"after {self._warmup_seen} warmup scans"
            )

        out = self._fit_beams(msg, self._fixed_beam_count, fallback)
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanSanitizeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
