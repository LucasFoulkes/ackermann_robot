#!/usr/bin/env python3
"""Log map->odom TF age and publish /nav_tf_ready for chaining Nav2 goals."""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class TfHealth(Node):
    def __init__(self):
        super().__init__("tf_health")
        self.ready_max_age_s = float(
            self.declare_parameter("ready_max_age_s", 0.5).value)
        period = float(self.declare_parameter("period_s", 1.0).value)
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ready_pub = self.create_publisher(Bool, "nav_tf_ready", 10)
        self._last_ready = None
        self.create_timer(max(0.5, period), self._tick)
        self.get_logger().info(
            f"tf_health: ready when map->odom age < {self.ready_max_age_s:.1f}s "
            f"(topic /nav_tf_ready); log every {period:.0f}s"
        )

    def _map_odom_age_s(self) -> float | None:
        if not self.tf_buffer.can_transform(
            self.map_frame, self.odom_frame, Time(),
            timeout=Duration(seconds=0.5),
        ):
            return None
        t = self.tf_buffer.lookup_transform(
            self.map_frame, self.odom_frame, Time())
        stamp = Time.from_msg(t.header.stamp)
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _tick(self):
        now = self.get_clock().now()
        lines = []
        ready = False

        try:
            age = self._map_odom_age_s()
            if age is None:
                lines.append("map->odom=MISSING")
            else:
                lines.append(f"map->odom_age={age:.2f}s")
                # Stale when TF stamp is too old; small negative age (future stamp) is OK.
                ready = age < self.ready_max_age_s
        except Exception as exc:  # noqa: BLE001
            lines.append(f"map->odom=ERR({exc})")

        try:
            if self.tf_buffer.can_transform(
                self.odom_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.5),
            ):
                t = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, Time())
                stamp = Time.from_msg(t.header.stamp)
                age = (now - stamp).nanoseconds / 1e9
                lines.append(f"odom->base_age={age:.2f}s")
            else:
                lines.append("odom->base=MISSING")
        except Exception as exc:  # noqa: BLE001
            lines.append(f"odom->base=ERR({exc})")

        status = "NAV_READY" if ready else "NAV_WAIT"
        lines.append(status)
        self.ready_pub.publish(Bool(data=ready))

        if ready != self._last_ready:
            if ready:
                self.get_logger().info(
                    "[tf_health] NAV_READY — OK to send next Nav2 goal")
            else:
                self.get_logger().warn(
                    "[tf_health] NAV_WAIT — map->odom stale; wait before next goal",
                    throttle_duration_sec=2.0,
                )
            self._last_ready = ready
        elif not ready:
            self.get_logger().warn(
                f"[tf_health] {' | '.join(lines)}",
                throttle_duration_sec=3.0,
            )
        else:
            self.get_logger().info(f"[tf_health] {' | '.join(lines)}")


def main():
    rclpy.init()
    node = TfHealth()
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
