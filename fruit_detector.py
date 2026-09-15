#!/usr/bin/env python3
"""果实感知节点（路线 A：经典视觉）。

对腕部 RGB-D 相机的对齐彩色图做 HSV 红色分割找果实候选，用深度图质心中值
反投影成相机系 3D 点，再经 TF 变换到规划系（world），发布：
  /detected_fruits        geometry_msgs/PoseArray  果实位置（world 系）
  /detected_fruits/rays   geometry_msgs/PoseArray  果实位置 + 朝向=z 轴指向
                                                   果实的接近方向（供预抓取用）
  /detected_fruits/image  sensor_msgs/Image         画框调试图

用法：
  python3 fruit_detector.py                          # 持续检测（约 2Hz）
  python3 fruit_detector.py --once --save out.jpg    # 处理一帧保存调试图后退出
"""

import argparse
import math
from os import path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


# ---------- 四元数/旋转工具（numpy 实现，不依赖 tf_transformations） ----------
def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def z_align_quat(direction):
    """返回使 z 轴对齐 direction 的四元数 (x,y,z,w)。"""
    z = np.asarray(direction, dtype=float)
    z = z / np.linalg.norm(z)
    ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(ref, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = np.stack([x, y, z], axis=1)  # 列为轴
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w, x_, y_, z_ = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1 + m[i, i] - m[j, j] - m[k, k]) * 2
        q = [0.0, 0.0, 0.0, 0.25 * s]
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
        q[3] = (m[k, j] - m[j, k]) / s
        x_, y_, z_, w = q
    return x_, y_, z_, w


class FruitDetector(Node):
    def __init__(self, args):
        super().__init__("fruit_detector")
        self.args = args
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/tip_camera/image_raw", self.cb_color, qos)
        self.create_subscription(Image, "/tip_camera/depth/image_raw", self.cb_depth, qos)
        self.create_subscription(CameraInfo, "/tip_camera/camera_info", self.cb_info, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 相机外参标定：模板 TF 的光学系旋转有误(多次旋转)，用球标定得到的
        # 常量修正矩阵右乘修正（见 camera_extrinsic_calibration.npy）
        import os
        cal_path = path.join(path.dirname(path.abspath(__file__)), "camera_extrinsic_calibration.npy")
        self.C = np.load(cal_path) if os.path.exists(cal_path) else np.eye(3)
        self.pub_pose = self.create_publisher(PoseArray, "/detected_fruits", 10)
        self.pub_ray = self.create_publisher(PoseArray, "/detected_fruits/rays", 10)
        self.pub_img = self.create_publisher(Image, "/detected_fruits/image", 10)

    def cb_color(self, m):
        self.color = m

    def cb_depth(self, m):
        self.depth = m

    def cb_info(self, m):
        self.info = m

    def wait_frames(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.color is not None and self.depth is not None and self.info is not None:
                return True
        return False

    def detect(self, bgr, depth_m, fx, fy, cx, cy):
        """HSV 红色分割 + 连通域 + 深度质心反投影。

        返回 (detections, mask)；每个 detection = dict(x,y,z,u,v,r)。（光学系坐标）
        """
        a = self.args
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0].astype(int)
        red = (h >= a.h_red_lo) | (h <= a.h_red_hi)
        mask = (red & (hsv[:, :, 1] >= a.sat_min) & (hsv[:, :, 2] >= a.val_min)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        dets = []
        for i in range(1, n):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < a.min_area:
                continue
            u, v = centroids[i]
            if not self.args.no_fp_filter and u > 530 and v > 410:
                continue  # 右下角夹爪假阳性区域
            r = math.sqrt(area / math.pi)
            rr = max(3.0, r * 0.6)
            x0, x1 = max(0, int(u - rr)), min(depth_m.shape[1], int(u + rr) + 1)
            y0, y1 = max(0, int(v - rr)), min(depth_m.shape[0], int(v + rr) + 1)
            disc = depth_m[y0:y1, x0:x1]
            valid = disc[np.isfinite(disc) & (disc > 0.3) & (disc < 5.0)]
            if valid.size < 5:
                continue
            z = float(np.median(valid))
            dets.append(dict(
                x=(u - cx) * z / fx, y=(v - cy) * z / fy, z=z,
                u=u, v=v, r=r,
            ))
        return dets, mask

    def lookup_tf(self, stamp):
        """world <- tip_camera_depth_frame 变换（先按时间戳，失败用最新）。"""
        try:
            return self.tf_buffer.lookup_transform(
                "world", "tip_camera_depth_frame", stamp, timeout=Duration(seconds=0.3))
        except Exception:
            return self.tf_buffer.lookup_transform(
                "world", "tip_camera_depth_frame", rclpy.time.Time())

    def process_once(self, save=""):
        if not self.wait_frames():
            print("[!] 相机数据未就绪（/tip_camera/* 话题无数据）")
            return False
        bgr = self.bridge.imgmsg_to_cv2(self.color, "bgr8")
        depth_m = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough").astype(np.float32)
        if self.args.flip:
            bgr = cv2.flip(bgr, 0)
            depth_m = np.flipud(depth_m)
            bgr = np.ascontiguousarray(bgr)
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]

        dets, _ = self.detect(bgr, depth_m, fx, fy, cx, cy)
        try:
            tf = self.lookup_tf(self.depth.header.stamp)
        except Exception as e:
            print(f"[!] TF 查询失败: {e}")
            return False
        R = quat_to_matrix(tf.transform.rotation.x, tf.transform.rotation.y,
                           tf.transform.rotation.z, tf.transform.rotation.w) @ self.C
        tr = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        cam_t = tr  # 相机原点在 world 系

        annotated = bgr.copy()
        pa, rays = PoseArray(), PoseArray()
        pa.header.frame_id = rays.header.frame_id = tf.header.frame_id
        pa.header.stamp = rays.header.stamp = self.depth.header.stamp

        for d in dets:
            p_opt = np.array([d["x"], d["y"], d["z"]])
            pw = R @ p_opt + tr
            d["world"] = pw
            pose, rpose = Pose(), Pose()
            pose.position.x, pose.position.y, pose.position.z = pw
            rpose.position = pose.position
            ray = pw - cam_t
            n = np.linalg.norm(ray)
            if n > 1e-6:
                ray /= n
            qx, qy, qz, qw = z_align_quat(ray)
            rpose.orientation.x, rpose.orientation.y, rpose.orientation.z, rpose.orientation.w = qx, qy, qz, qw
            pa.poses.append(pose)
            rays.poses.append(rpose)
            cv2.circle(annotated, (int(d["u"]), int(d["v"])), int(max(d["r"], 4)), (0, 0, 255), 2)
            cv2.putText(annotated, f"{pw[0]:.2f},{pw[1]:.2f},{pw[2]:.2f}",
                        (int(d["u"]) + 6, int(d["v"]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        self.pub_pose.publish(pa)
        self.pub_ray.publish(rays)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
        if save:
            cv2.imwrite(save, annotated)
        print(f"检测到 {len(dets)} 个果实：")
        for d in dets:
            pw = d["world"]
            print(f"    world 系 ({pw[0]:.3f}, {pw[1]:.3f}, {pw[2]:.3f})  深度 {d['z']:.2f}m  像素({d['u']:.0f},{d['v']:.0f})")
        return True

    def run_continuous(self):
        period = 1.0 / self.args.rate
        last = 0.0
        while rclpy.ok():
            now = time.time()
            if now - last >= period:
                last = now
                try:
                    self.process_once()
                except Exception as e:
                    print(f"[!] 处理异常: {e}")
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--once", action="store_true", help="处理一帧后退出")
    parser.add_argument("--save", default="", help="保存标注图路径")
    parser.add_argument("--h-red-lo", type=int, default=170)
    parser.add_argument("--h-red-hi", type=int, default=10)
    parser.add_argument("--sat-min", type=int, default=90)
    parser.add_argument("--val-min", type=int, default=50)
    parser.add_argument("--min-area", type=int, default=60, help="最小连通域面积 px")
    parser.add_argument("--rate", type=float, default=2.0, help="持续模式频率 Hz")
    parser.add_argument("--flip", action="store_true", help="(弃用，改用标定矩阵)")
    parser.add_argument("--no-fp-filter", action="store_true", help="关闭右下角假阳性过滤")
    args = parser.parse_args()

    rclpy.init()
    node = FruitDetector(args)
    try:
        if args.once:
            ok = node.process_once(save=args.save)
            return 0 if ok else 1
        node.run_continuous()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
