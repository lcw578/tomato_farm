#!/usr/bin/env python3
"""仿真采摘 demo（v2：感知闭环版）。

工作流程（--source perception，默认）：
  对每个方位角执行：MoveIt 把手掌(相机)送到观测位姿 → 等 fruit_detector 输出
  /detected_fruits → 合并新检测 → 对可达果实依次执行 预抓取点 → 果实 → 闭合夹爪
  → 撤回。全部方位扫完且无新果 → 结束。

  --source markers 为 v1 真值模式：直接读 markers.json（不依赖感知），用于对照。

用法（四个终端）：
  1) ros2 launch aoc_tomato_farm_gazebo tomato_farm_world_mobile_manipulator_001.launch.py x_pose:=0.6 y_pose:=1.0
  2) ros2 launch aoc_tomato_farm_gazebo control.launch.py
  3) python3 fruit_detector.py                # 感知节点（持续发布 /detected_fruits）
  4) python3 harvest_demo.py --robot-x 0.6 --robot-y 1.0 --sweep 90,130,170
"""

import argparse
import json
import math
import sys
import time
import subprocess
from os import path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, MotionPlanRequest, OrientationConstraint,
                             PositionConstraint)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Image

FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]
GRIPPER_ACTION = "/gripper_trajectory_controller/follow_joint_trajectory"
BASE_Z = 0.333
FRAME = "world"


def farm_to_robot(p, tx, ty, yaw):
    dx, dy = p[0] - tx, p[1] - ty
    c, s = math.cos(-yaw), math.sin(-yaw)
    return [c * dx - s * dy, s * dx + c * dy, p[2]]


def z_align_quat(direction):
    """z 轴对齐 direction 的四元数。"""
    z = np.asarray(direction, dtype=float)
    z = z / np.linalg.norm(z)
    ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(ref, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = np.stack([x, y, z], axis=1)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s_ = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s_
        x_ = (m[2, 1] - m[1, 2]) / s_
        y_ = (m[0, 2] - m[2, 0]) / s_
        z_ = (m[1, 0] - m[0, 1]) / s_
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s_ = math.sqrt(1 + m[i, i] - m[j, j] - m[k, k]) * 2
        q = [0.0, 0.0, 0.0, 0.0]
        q[i] = 0.25 * s_
        q[j] = (m[j, i] + m[i, j]) / s_
        q[k] = (m[k, i] + m[i, k]) / s_
        q[3] = (m[k, j] - m[j, k]) / s_
        x_, y_, z_, w = q
    return x_, y_, z_, w


class Harvester(Node):
    def __init__(self, args):
        super().__init__("harvest_demo")
        self.args = args
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(self, FollowJointTrajectory, GRIPPER_ACTION)
        self.latest_dets = None      # (stamp, [Pose, ...])
        self.latest_rays = None
        self.latest_dbg = None
        qos = rclpy.qos.QoSProfile(depth=5)
        self.create_subscription(PoseArray, "/detected_fruits", self.cb_dets, qos)
        self.create_subscription(PoseArray, "/detected_fruits/rays", self.cb_rays, qos)
        self.create_subscription(Image, "/detected_fruits/image", self.cb_dbg_img, qos)

    def cb_dets(self, m):
        self.latest_dets = m

    def cb_rays(self, m):
        self.latest_rays = m

    def cb_dbg_img(self, m):
        self.latest_dbg = m

    # ---------- MoveIt ----------
    def move_hand(self, xyz, quat=None, pos_tol=0.04, attempts=6):
        """位置(+可选朝向)约束规划并执行，返回 (ok, 错误信息)。"""
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = "panda_hand"
        pc.constraint_region.primitives.append(sphere)
        pose = Pose()
        pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        cons = Constraints()
        cons.position_constraints.append(pc)
        if quat is not None:
            oc = OrientationConstraint()
            oc.header.frame_id = FRAME
            oc.link_name = "panda_hand"
            oc.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]),
                                        z=float(quat[2]), w=float(quat[3]))
            oc.absolute_x_axis_tolerance = 0.25
            oc.absolute_y_axis_tolerance = 0.25
            oc.absolute_z_axis_tolerance = 0.25
            oc.weight = 1.0
            cons.orientation_constraints.append(oc)

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.pipeline_id = "ompl"
        req.group_name = "arm"
        req.num_planning_attempts = attempts
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = 0.15
        req.max_acceleration_scaling_factor = 0.15
        req.goal_constraints.append(cons)
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True

        if not self.move_client.wait_for_server(timeout_sec=2.0):
            return False, "move_group 不可用"
        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False, "目标被拒绝"
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=150.0)
        result = res_fut.result()
        if result is None:
            handle.cancel_goal()
            return False, "执行超时"
        if result.result.error_code.val != 1:
            return False, f"error_code={result.result.error_code.val}"
        return True, None

    def move_gripper(self, width, secs=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = FINGERS
        pt = JointTrajectoryPoint()
        pt.positions = [float(width)] * len(FINGERS)
        pt.time_from_start.sec = int(secs)
        goal.trajectory.points.append(pt)
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            return "夹爪控制器不可用"
        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return "夹爪目标被拒绝"
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=30.0)
        return None

    # ---------- 观测 ----------
    def observe(self, az_deg):
        """把手掌送到方位角 az_deg（机器人系，度）的观测位姿。"""
        a = math.radians(az_deg)
        r_hand, r_look = 0.35, 0.75
        hand = np.array([r_hand * math.cos(a), r_hand * math.sin(a), 0.70])
        look = np.array([r_look * math.cos(a), r_look * math.sin(a), 0.80])
        d = look - hand
        quat = z_align_quat(d)
        ok, err = self.move_hand(hand, quat=quat, pos_tol=0.06)
        if not ok:
            return False, err
        # 等感知节点输出 arrive 之后的新帧
        ref = self.latest_dets
        deadline = time.time() + 8
        got = False
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_dets is not ref:
                got = True
                break
        settle = time.time() + 1.5
        while time.time() < settle:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_dbg is not None:
            import cv2
            from cv_bridge import CvBridge
            cv2.imwrite(f"/tmp/observe_{int(az_deg)}.jpg",
                        CvBridge().imgmsg_to_cv2(self.latest_dbg, "bgr8"))
        print(f"    [观测] 新检测消息: {'是' if got else '否(超时)'}")
        return True, None

    def collect_detections(self, picked, reach):
        """取最新检测，返回可达且未摘过的新果实 [(pos, ray), ...]。"""
        if self.latest_dets is None or self.latest_rays is None:
            return []
        out = []
        for pose, rpose in zip(self.latest_dets.poses, self.latest_rays.poses):
            p = np.array([pose.position.x, pose.position.y, pose.position.z])
            ray = np.array([rpose.orientation.x, rpose.orientation.y, rpose.orientation.z])
            n = np.linalg.norm(ray)
            ray = ray / n if n > 1e-6 else np.array([0, 0, -1.0])
            if any(np.linalg.norm(p - q) < 0.08 for q, _ in picked):
                continue
            if np.linalg.norm(p - [0, 0, BASE_Z]) > reach:
                continue
            out.append((p, ray))
        out.sort(key=lambda t: np.linalg.norm(t[0] - [0, 0, BASE_Z]))
        return out

    # ---------- 采摘 ----------
    def pick(self, pos, ray):
        pre = pos - ray * 0.12
        ok, err = self.move_hand(pre, pos_tol=0.05)
        if not ok:
            return f"预抓取规划失败({err})"
        ok, err = self.move_hand(pos, pos_tol=0.035)
        if not ok:
            return f"抓取点规划失败({err})"
        err = self.move_gripper(self.args.gripper_close)
        if err:
            return f"闭合失败({err})"
        self.move_hand(pre, pos_tol=0.06)
        self.move_gripper(self.args.gripper_open)
        return None


def load_markers_fruits(farm, rx, ry, ryaw, reach):
    markers_path = path.join(
        get_package_share_directory("aoc_tomato_farm_gazebo"), "models", farm, "markers.json")
    with open(markers_path) as f:
        markers = json.load(f)
    out = []
    for m in markers:
        if m["marker_type"] != "FRUIT":
            continue
        pr = farm_to_robot(m["translation"], rx, ry, ryaw)
        d = np.linalg.norm(np.array(pr) - [0, 0, BASE_Z])
        if d <= reach:
            out.append((np.array(pr), None, d))
    out.sort(key=lambda t: t[2])
    return [(p, r) for p, r, _ in out]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=["perception", "markers"], default="perception")
    parser.add_argument("--farm", default="8mx7m")
    parser.add_argument("--robot-x", type=float, default=1.0)
    parser.add_argument("--robot-y", type=float, default=0.0)
    parser.add_argument("--robot-yaw", type=float, default=-1.57)
    parser.add_argument("--reach", type=float, default=0.85)
    parser.add_argument("--sweep", default="90,130,170", help="观测方位角(机器人系,度,逗号分隔)")
    parser.add_argument("--max-fruits", type=int, default=0)
    parser.add_argument("--gripper-open", type=float, default=0.04)
    parser.add_argument("--gripper-close", type=float, default=0.0)
    parser.add_argument("--observe-only", action="store_true", help="只做观测扫描并打印检测，不采摘")
    args = parser.parse_args()

    rclpy.init()
    node = Harvester(args)

    if args.source == "perception":
        # 归位到标准构型，保证约束规划有良好起点
        subprocess.run(["ros2", "run", "franka", "controller"] +
                       [str(j) for j in (2.0, -0.7, 0, -1.3, 0, 1.7, -2.3)],
                       capture_output=True, timeout=60)
        time.sleep(1.0)
        sweep = [float(a) for a in args.sweep.split(",")]
        picked = []
        total_picked = 0
        for round_ in range(3):
            found_any = False
            for az in sweep:
                print(f"—— 观测方位角 {az:.0f}° ——")
                ok, err = node.observe(az)
                if not ok:
                    print(f"    [!] 观测位姿失败: {err}")
                    continue
                if args.observe_only:
                    dets = node.collect_detections([], reach=1e9)
                    for p, _ in dets:
                        print(f"    检测: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
                    continue
                targets = node.collect_detections(picked, args.reach)
                if not targets:
                    print("    无新可达果实")
                    continue
                found_any = True
                for pos, ray in targets:
                    if args.max_fruits and total_picked >= args.max_fruits:
                        break
                    print(f"    采摘目标 ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) …")
                    err = node.pick(pos, ray)
                    if err:
                        print(f"    ✗ {err}")
                    else:
                        print("    ✓ 完成")
                        picked.append((pos, ray))
                        total_picked += 1
            if args.observe_only or not found_any or (args.max_fruits and total_picked >= args.max_fruits):
                break
        if not args.observe_only:
            print(f"完成：共采摘 {total_picked} 个果实")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    # ---------- markers 真值模式（v1 逻辑） ----------
    targets = load_markers_fruits(args.farm, args.robot_x, args.robot_y, args.robot_yaw, args.reach)
    print(f"农场 {args.farm} 可达果实 {len(targets)} 个（真值模式）")
    if not targets:
        return 0
    err = node.move_gripper(args.gripper_open)
    if err:
        print(f"[!] {err}")
    ok = fail = 0
    for i, (pos, ray) in enumerate(targets, 1):
        ray = ray if ray is not None else np.array([0, 0.0, -1.0])
        print(f"[{i}/{len(targets)}] 目标 ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) …")
        err = node.pick(pos, ray)
        if err:
            print(f"    ✗ {err}")
            fail += 1
        else:
            print("    ✓ 完成")
            ok += 1
    print(f"完成：成功 {ok}，失败 {fail}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
