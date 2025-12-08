#!/usr/bin/env python3
# Simple bridge that lets you drive the Gazebo joint_trajectory_controller
# Usage:
#   ros2 run eli_cs_robot_simulation_gz gz_control --example
#   ros2 run eli_cs_robot_simulation_gz gz_control --from-yaml /path/to/waypoints.yaml
import argparse
import math
import sys
from typing import List, Tuple
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml
JointGoal = Tuple[List[float], float]
def _duration_from_seconds(time_s: float) -> Duration:
    sec = int(math.floor(time_s))
    nanosec = int((time_s - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)
class SdkGzBridge(Node):
    def __init__(self, controller: str):
        super().__init__("sdk_gz_bridge")
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{controller}/follow_joint_trajectory",
        )
    def send_goal(self, goals: List[JointGoal]) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("follow_joint_trajectory action server not available")
            return False

        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        elapsed = 0.0
        for positions, duration in goals:
            elapsed += duration
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = _duration_from_seconds(elapsed)
            msg.trajectory.points.append(point)

        self.get_logger().info(
            f"Sending {len(goals)} waypoint(s) to {self._action_client._action_name}"
        )
        send_future = self._action_client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"Controller finished with error_code={result.error_code}")
        return result.error_code == 0

def _load_from_yaml(path: str) -> List[JointGoal]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, list):
        raise ValueError("YAML must contain a list of waypoints")

    goals: List[JointGoal] = []
    for entry in data:
        if not isinstance(entry, dict):
            raise ValueError("Each waypoint must be a dict with positions/duration")
        positions = entry.get("positions")
        duration = float(entry.get("duration", 3.0))
        if positions is None or len(positions) != 6:
            raise ValueError("Each waypoint needs 6 joint positions")
        goals.append((list(map(float, positions)), duration))
    return goals

def _example_goals() -> List[JointGoal]:
    return [
        ([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], 1.5),
        ([0.3, -1.2, 0.2, -1.3, 0.4, 0.0], 2.0),
        ([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], 2.0),
    ]

def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send SDK-like waypoints to Gazebo controller.")
    parser.add_argument(
        "--controller",
        default="joint_trajectory_controller",
        help="Controller name (default: joint_trajectory_controller)",
    )
    parser.add_argument(
        "--from-yaml",
        dest="yaml_path",
        help="YAML file with waypoints: [{'positions': [...6...], 'duration': <seconds>}, ...]",
    )
    parser.add_argument(
        "--example",
        action="store_true",
        help="Send a small built-in demo trajectory.",
    )
    return parser.parse_args(argv)

def main(argv=None):
    args = _parse_args(argv or sys.argv[1:])

    if not args.example and not args.yaml_path:
        print("Please pass --example or --from-yaml <file>", file=sys.stderr)
        return 1

    if args.yaml_path:
        try:
            goals = _load_from_yaml(args.yaml_path)
        except Exception as exc:  # pragma: no cover - CLI helper
            print(f"Failed to load waypoints: {exc}", file=sys.stderr)
            return 1
    else:
        goals = _example_goals()

    rclpy.init()
    node = SdkGzBridge(controller=args.controller)
    success = node.send_goal(goals)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 2

if __name__ == "__main__":
    sys.exit(main())
