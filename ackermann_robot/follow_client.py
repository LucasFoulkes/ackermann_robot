#!/usr/bin/env python3
"""Glue between the person tracker and opennav_following:
  - sends FollowObject(pose_topic=/person_target) whenever a target exists
    and no follow action is active; re-sends after failures
  - relays the server's Twist on /cmd_vel_follow to TwistStamped on
    /cmd_vel_nav (what cmd_vel_to_effort consumes)
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from opennav_following_msgs.action import FollowObject

STATES = {0: "NONE", 1: "INITIAL_PERCEPTION", 2: "CONTROLLING",
          3: "STOPPING", 4: "RETRY"}


class FollowClient(Node):
    def __init__(self):
        super().__init__("follow_client")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.pose_topic = str(p("pose_topic", "/person_target"))
        self.send_max_age_s = float(p("send_max_age_s", 1.5))
        self.retry_backoff_s = float(p("retry_backoff_s", 2.0))

        self.last_target_s = None
        self.goal_handle = None
        self.goal_pending = False
        self.next_send_s = 0.0
        self.last_state = -1
        self.last_state_log_s = 0.0

        self.client = ActionClient(self, FollowObject, "follow_object")
        self.create_subscription(PoseStamped, self.pose_topic,
                                 self.on_target, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, "/cmd_vel_nav", 10)
        self.create_subscription(Twist, "/cmd_vel_follow", self.on_cmd, 10)
        self.create_timer(0.5, self.tick)
        self.create_timer(2.0, self.heartbeat)
        self.get_logger().info(
            f"follow_client: FollowObject(pose_topic={self.pose_topic})")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist = msg
        self.pub_cmd.publish(out)

    def on_target(self, msg):
        self.last_target_s = self.now_s()

    def heartbeat(self):
        age = (None if self.last_target_s is None
               else self.now_s() - self.last_target_s)
        goal = ("ACTIVE" if self.goal_handle is not None
                else "pending" if self.goal_pending else "none")
        self.get_logger().info(
            "[hb] target " + (f"age {age:.1f}s" if age is not None else "never")
            + f" | follow action {goal}")

    def tick(self):
        if self.goal_handle is not None or self.goal_pending:
            return
        if self.last_target_s is None or self.now_s() < self.next_send_s:
            return
        if self.now_s() - self.last_target_s > self.send_max_age_s:
            return
        if not self.client.server_is_ready():
            self.get_logger().warning("follow_object server not ready",
                                      throttle_duration_sec=5.0)
            return
        goal = FollowObject.Goal()
        goal.pose_topic = self.pose_topic
        self.goal_pending = True
        self.get_logger().info("sending FollowObject goal")
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_fb)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        self.goal_pending = False
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warning("FollowObject goal rejected")
            self.next_send_s = self.now_s() + self.retry_backoff_s
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self.on_result)

    def on_fb(self, fb):
        st = fb.feedback.state
        if st != self.last_state and self.now_s() - self.last_state_log_s > 1.0:
            self.get_logger().info(f"follow state: {STATES.get(st, st)}")
            self.last_state = st
            self.last_state_log_s = self.now_s()

    def on_result(self, fut):
        self.goal_handle = None
        try:
            status = fut.result().status
        except Exception:
            status = -1
        names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        self.get_logger().info(
            f"FollowObject ended: {names.get(status, f'status {status}')}")
        self.next_send_s = self.now_s() + self.retry_backoff_s


def main():
    rclpy.init()
    node = FollowClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
