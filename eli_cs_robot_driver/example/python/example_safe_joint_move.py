#!/usr/bin/env python3
# Send a single joint target with a computed duration that respects a max joint velocity.
# Intended for real robot via scaled_joint_trajectory_controller to avoid "External Control speed limit".

import argparse
import math
import sys
from typing import List, Optional
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class SafeJointMove(Node):
    def __init__(self):
        super().__init__("safe_joint_move")

        self.declare_parameter("controller", "scaled_joint_trajectory_controller")
        self.declare_parameter("target")
        self.declare_parameter("max_vel", 0.5)
        self.declare_parameter("min_time", 2.0)

        self._controller = self.get_parameter("controller").value
        self._target = list(self.get_parameter("target").value)
        self._max_vel = self.get_parameter("max_vel").value
        self._min_time = self.get_parameter("min_time").value

        if len(self._target) != 6:
            raise RuntimeError("Parameter 'target' must contain exactly 6 joint values")

        self._current: Optional[List[float]] = None

        self._sub = self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10
        )

        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._controller}/follow_joint_trajectory",
        )

    def _dur(self, sec: float) -> Duration:
        sec_i = int(math.floor(sec))
        nanosec = int((sec - sec_i) * 1e9)
        return Duration(sec=sec_i, nanosec=nanosec)

    def _js_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._current = [float(name_to_pos[j]) for j in JOINTS]
        except KeyError:
            self.get_logger().warn("Joint names mismatch in /joint_states; skipping update")

    def run(self) -> bool:
        # Wait for joint state
        self.get_logger().info("Waiting for /joint_states...")
        for _ in range(50):  # up to ~5s
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._current is not None:
                break
        if self._current is None:
            self.get_logger().error("No /joint_states received; aborting to avoid unsafe motion.")
            return False

        # Compute required duration from max velocity
        deltas = [abs(t - c) for t, c in zip(self._target, self._current)]
        max_delta = max(deltas)
        needed_time = max_delta / self._max_vel if self._max_vel > 0 else self._min_time
        duration = max(needed_time, self._min_time)
        self.get_logger().info(
            f"Max delta {max_delta:.3f} rad, using duration {duration:.3f} s (max_vel {self._max_vel} rad/s)"
        )

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("follow_joint_trajectory action server not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        # Start at current to avoid abrupt jump inside controller
        start = JointTrajectoryPoint()
        start.positions = list(self._current)
        start.time_from_start = self._dur(0.0)
        goal.trajectory.points.append(start)

        point = JointTrajectoryPoint()
        point.positions = list(self._target)
        point.time_from_start = self._dur(duration)
        goal.trajectory.points.append(point)
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("Goal rejected by controller")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != 0:
            self.get_logger().error(f"Controller returned error_code={result.error_code}")
            return False

        self.get_logger().info("Trajectory executed successfully.")
        return True


if __name__ == "__main__":
    rclpy.init()
    node = SafeJointMove()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)