#!/usr/bin/env python3
from __future__ import annotations

import datetime
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


@dataclass
class TwistSample:
    linear_x: float
    angular_z: float
    stamp_ns: int


class CmdOdomDiff(Node):
    def __init__(self) -> None:
        super().__init__("cmd_odom_diff")

        self.declare_parameter("cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("odom_topic", "/odom_rf2o")
        self.declare_parameter("log_file", "/tmp/cmd_odom_diff.txt")
        self.declare_parameter("log_on_odom_only", True)

        self._cmd_topic = str(self.get_parameter("cmd_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._log_path = Path(str(self.get_parameter("log_file").value))
        self._log_on_odom_only = bool(self.get_parameter("log_on_odom_only").value)

        self._last_cmd: Optional[TwistSample] = None
        self._last_odom: Optional[TwistSample] = None

        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = self._log_path.open("w", encoding="utf-8")
        self._write_csv_header()

        self._cmd_sub = self.create_subscription(
            Twist,
            self._cmd_topic,
            self._on_cmd,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
        )

        mode = "odom_only" if self._log_on_odom_only else "both"
        self.get_logger().info(
            f"CmdOdomDiff listening: cmd={self._cmd_topic} odom={self._odom_topic} log={self._log_path} mode={mode}"
        )

    def _on_cmd(self, msg: Twist) -> None:
        twist = msg
        self._last_cmd = TwistSample(
            linear_x=float(twist.linear.x),
            angular_z=float(twist.angular.z),
            stamp_ns=self.get_clock().now().nanoseconds,
        )
        if not self._log_on_odom_only:
            self._log_update(source="cmd")

    def _on_odom(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        self._last_odom = TwistSample(
            linear_x=float(twist.linear.x),
            angular_z=float(twist.angular.z),
            stamp_ns=int(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec),
        )
        self._log_update(source="odom")

    def _log_update(self, *, source: str) -> None:
        cmd = self._last_cmd
        odom = self._last_odom

        linear_diff = None
        angular_diff = None
        if cmd is not None and odom is not None:
            linear_diff = cmd.linear_x - odom.linear_x
            angular_diff = cmd.angular_z - odom.angular_z

        stamp = datetime.datetime.now(tz=datetime.timezone.utc).isoformat()
        csv_line = (
            f"{stamp},{source},"
            f"{_fmt(cmd.linear_x if cmd else None)},"
            f"{_fmt(cmd.angular_z if cmd else None)},"
            f"{_fmt(odom.linear_x if odom else None)},"
            f"{_fmt(odom.angular_z if odom else None)},"
            f"{_fmt(linear_diff)},"
            f"{_fmt(angular_diff)}"
        )

        self.get_logger().info(csv_line)
        self._log_file.write(csv_line + "\n")
        self._log_file.flush()

    def _write_csv_header(self) -> None:
        header = "timestamp,source,cmd_lin,cmd_ang,odom_lin,odom_ang,diff_lin,diff_ang"
        self._log_file.write(header + "\n")
        self._log_file.flush()

    def destroy_node(self) -> None:
        try:
            self._log_file.close()
        except Exception:
            pass
        super().destroy_node()


def _fmt(value: Optional[float]) -> str:
    if value is None:
        return "nan"
    return f"{value:.4f}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdOdomDiff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
