#!/usr/bin/env python3
# Plan to a Cartesian pose with MoveIt, then optionally execute it.
# target-pose is x y z rx ry rz: meters + RPY [rad].
#
# Usage examples:
#   # Plan only, keep current orientation, move tool0 +2 cm along base_link X:
#   ros2 run eli_cs_robot_driver example_moveit_pose_goal_py --offset "[0.02,0.0,0.0]"
#
#   # Execute the offset plan:
#   ros2 run eli_cs_robot_driver example_moveit_pose_goal_py --offset "[0.02,0.0,0.0]" --execute
#
#   # Absolute pose target:
#   ros2 run eli_cs_robot_driver example_moveit_pose_goal_py --target-pose "[0.4,0.0,0.523,0.3,-1.0,0.5]" --execute

import argparse
import math
import sys
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    WorkspaceParameters,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Pose().orientation
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy

    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm > 0.0:
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
    return q


class MoveItPoseGoal(Node):
    def __init__(self, args):
        super().__init__("moveit_pose_goal_py")
        self.args = args
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pose_from_target(self) -> Optional[Pose]:
        if self.args.target_pose is None:
            return None
        if len(self.args.target_pose) != 6:
            raise RuntimeError("target_pose must contain 6 values: x y z rx ry rz")

        target = Pose()
        target.position.x = self.args.target_pose[0]
        target.position.y = self.args.target_pose[1]
        target.position.z = self.args.target_pose[2]
        target.orientation = quaternion_from_rpy(
            self.args.target_pose[3], self.args.target_pose[4], self.args.target_pose[5]
        )
        self.get_logger().info(
            "Target RPY=(%.4f, %.4f, %.4f) rad converted to quaternion"
            % (self.args.target_pose[3], self.args.target_pose[4], self.args.target_pose[5])
        )
        return target

    def current_pose(self) -> Pose:
        deadline = self.get_clock().now().nanoseconds + int(5.0 * 1e9)
        last_error = None
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.args.reference_frame,
                    self.args.ee_link,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                return pose
            except TransformException as ex:
                last_error = ex
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"Could not read TF {self.args.reference_frame}->{self.args.ee_link}: {last_error}")

    def target_pose(self) -> Pose:
        target = self.pose_from_target()
        if target is not None:
            return target

        if len(self.args.offset) != 3:
            raise RuntimeError("offset must contain 3 values: dx dy dz")
        current = self.current_pose()
        target = Pose()
        target.position.x = current.position.x + self.args.offset[0]
        target.position.y = current.position.y + self.args.offset[1]
        target.position.z = current.position.z + self.args.offset[2]
        target.orientation = current.orientation
        return target

    def build_constraints(self, target: Pose) -> Constraints:
        constraints = Constraints()
        constraints.name = "pose_goal"

        position = PositionConstraint()
        position.header.frame_id = self.args.reference_frame
        position.link_name = self.args.ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.args.position_tolerance]
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(target)
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.args.reference_frame
        orientation.link_name = self.args.ee_link
        orientation.orientation = target.orientation
        orientation.absolute_x_axis_tolerance = self.args.orientation_tolerance
        orientation.absolute_y_axis_tolerance = self.args.orientation_tolerance
        orientation.absolute_z_axis_tolerance = self.args.orientation_tolerance
        orientation.weight = 1.0

        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        return constraints

    def build_motion_request(self, target: Pose) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self.args.group
        request.num_planning_attempts = self.args.planning_attempts
        request.allowed_planning_time = self.args.planning_time
        request.max_velocity_scaling_factor = self.args.velocity_scale
        request.max_acceleration_scaling_factor = self.args.acceleration_scale
        request.goal_constraints.append(self.build_constraints(target))

        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = self.args.reference_frame
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -2.0
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0
        return request

    def plan(self, target: Pose):
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/move_action server is not available")

        goal = MoveGroup.Goal()
        goal.request = self.build_motion_request(target)
        # Match MoveGroupInterface::plan(): ask move_group to plan only here.
        # If --execute is set, execute the returned trajectory exactly once below.
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True

        self.get_logger().info("Sending MoveIt plan request")
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveGroup goal was rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt planning failed with error_code={result.error_code.val}")
        self.get_logger().info("MoveIt planning succeeded")
        return result.planned_trajectory

    def execute(self, trajectory) -> None:
        if not self.args.execute:
            self.get_logger().warn("execute=false, plan was not sent to the robot. Set --execute to move.")
            return
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/execute_trajectory server is not available")

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        self.get_logger().warn("Executing trajectory on the robot")
        future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("ExecuteTrajectory goal was rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt execution failed with error_code={result.error_code.val}")
        self.get_logger().info("MoveIt execution succeeded")

    def run(self) -> bool:
        target = self.target_pose()
        self.get_logger().info(
            "Planning group=%s, ee_link=%s, reference_frame=%s"
            % (self.args.group, self.args.ee_link, self.args.reference_frame)
        )
        self.get_logger().info(
            "Target pose position=(%.4f, %.4f, %.4f), quaternion=(%.4f, %.4f, %.4f, %.4f)"
            % (
                target.position.x,
                target.position.y,
                target.position.z,
                target.orientation.x,
                target.orientation.y,
                target.orientation.z,
                target.orientation.w,
            )
        )
        trajectory = self.plan(target)
        self.execute(trajectory)
        return True


def parse_float_list(value: str) -> List[float]:
    text = value.strip()
    if text.startswith("[") and text.endswith("]"):
        text = text[1:-1]
    if not text:
        return []
    return [float(item.strip()) for item in text.split(",")]


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Plan and optionally execute a Cartesian pose goal with MoveIt.")
    parser.add_argument("--group", default="cs_manipulator")
    parser.add_argument("--ee-link", default="tool0")
    parser.add_argument("--reference-frame", default="base_link")
    parser.add_argument("--execute", action="store_true", help="Execute the planned trajectory.")
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--velocity-scale", type=float, default=0.1)
    parser.add_argument("--acceleration-scale", type=float, default=0.1)
    parser.add_argument("--position-tolerance", type=float, default=0.001)
    parser.add_argument("--orientation-tolerance", type=float, default=0.01)
    parser.add_argument("--offset", type=parse_float_list, default=[0.02, 0.0, 0.0], help="dx,dy,dz in reference frame.")
    parser.add_argument("--target-pose", type=parse_float_list, help="x,y,z,rx,ry,rz. rx,ry,rz is RPY [rad].")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = MoveItPoseGoal(args)
    try:
        ok = node.run()
    except Exception as ex:  # noqa: BLE001 - report as ROS log and return non-zero.
        node.get_logger().error(str(ex))
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
