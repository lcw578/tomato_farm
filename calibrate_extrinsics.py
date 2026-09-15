#!/usr/bin/env python3
"""腕部相机外参标定工具。

背景：模板 URDF 的相机光学系存在旋转 bug（TF 与真实渲染差约 90°，且位置也有
偏差）。本工具在若干臂姿态下沿预测视线放置红色标定球，采集 像素↔世界坐标
对应，联合求解常量修正：旋转矩阵 C 与平移偏移 t，使得

    world_dir = R_tf(pose) @ C @ ray_optical
    world_org = tr_tf(pose) + t

结果保存到 camera_extrinsic_calibration.npz（fruit_detector.py 自动加载）。

用法（仿真 + 无需 move_group）：
  python3 calibrate_extrinsics.py
标定球会自动生成/删除。标定后建议重启 fruit_detector。
"""

import json
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from fruit_detector import quat_to_matrix

FX = FY = 565.6
CX, CY = 320.5, 240.5
POSES = [
    [1.2, -0.7, 0, -1.3, 0, 1.7, -2.3],
    [2.3, -0.7, 0, -1.3, 0, 1.7, -2.3],
    [0.2, -0.5, 0, -1.6, 0, 1.6, -2.3],
]
DIST = 0.8
OFFSETS = [(0.0, 0.0), (-0.18, 0.0), (0.0, -0.14)]  # 相机平面偏移 (x,y)


class Calib(Node):
    def __init__(self):
        super().__init__("extrinsic_calib")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.spawn = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete = self.create_client(DeleteEntity, "/delete_entity")

    def tf(self):
        t = self.tf_buffer.lookup_transform("world", "tip_camera_depth_frame", rclpy.time.Time())
        R = quat_to_matrix(t.transform.rotation.x, t.transform.rotation.y,
                           t.transform.rotation.z, t.transform.rotation.w)
        tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        return R, tr

    def ball(self, name, p, r=0.04):
        xml = ('<?xml version="1.0"?><sdf version="1.6"><model name="%s"><static>true</static>'
               '<pose>%.4f %.4f %.4f 0 0 0</pose><link name="l"><visual name="v"><geometry>'
               '<sphere><radius>%.3f</radius></sphere></geometry><material>'
               '<ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse>'
               '</material></visual></link></model></sdf>') % (name, p[0], p[1], p[2], r)
        req = SpawnEntity.Request()
        req.name, req.xml = name, xml
        fut = self.spawn.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)

    def remove(self, name):
        req = DeleteEntity.Request()
        req.name = name
        fut = self.delete.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)

    def detect_pixels(self, timeout=25.0):
        out = subprocess.run(["python3", "-u", "fruit_detector.py", "--once"],
                             capture_output=True, text=True, timeout=timeout).stdout
        pix = []
        for line in out.splitlines():
            if "像素" in line:
                import re
                m = re.search(r"像素\((\d+),(\d+)\)", line)
                pix.append((int(m.group(1)), int(m.group(2))))
        return pix


def main():
    rclpy.init()
    node = Calib()
    # 等待 TF
    end = time.time() + 10
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            node.tf()
            break
        except Exception:
            pass

    corres = []
    for pi, joints in enumerate(POSES):
        subprocess.run(["ros2", "run", "franka", "controller"] + [str(j) for j in joints],
                       capture_output=True, timeout=60)
        time.sleep(1.0)
        R, tr = node.tf()
        C_now = np.eye(3)
        t_now = np.zeros(3)
        try:
            data = np.load("camera_extrinsic_calibration.npz")
            C_now, t_now = data["C"], data["t"]
        except Exception:
            try:
                C_now = np.load("camera_extrinsic_calibration.npy")
            except Exception:
                pass
        Rc = R @ C_now
        axis = Rc[:, 2]
        Xc, Yc = Rc[:, 0], Rc[:, 1]
        center = tr + DIST * axis

        truths = []
        for k, (ox, oy) in enumerate(OFFSETS):
            p = center + ox * Xc + oy * Yc
            p[2] = max(p[2], 0.10)
            node.ball(f"cal_{pi}_{k}", p)
            truths.append(p)
        time.sleep(1.2)
        pix = node.detect_pixels()
        if len(pix) != len(OFFSETS):
            print(f"位姿 {pi}: 检出 {len(pix)}/{len(OFFSETS)}，跳过该位姿")
        else:
            # 按预测投影位置匹配对应关系
            for p in truths:
                pc = Rc @ (p - tr)
                pu = FX * pc[0] / pc[2] + CX
                pv = FY * pc[1] / pc[2] + CY
                u, v = min(pix, key=lambda q: (q[0] - pu) ** 2 + (q[1] - pv) ** 2)
                corres.append((p, u, v, R.copy(), tr.copy()))
            print(f"位姿 {pi}: 采集 {len(pix)} 点")
        for k in range(len(OFFSETS)):
            node.remove(f"cal_{pi}_{k}")
        time.sleep(0.5)

    if len(corres) < 6:
        print("对应点不足，标定失败")
        return 1

    # 联合求解 C(旋转) + t(平移)
    R0 = np.load("camera_extrinsic_calibration.npy") if len(sys.argv) < 2 else np.eye(3)
    rv0 = Rotation.from_matrix(R0).as_rotvec()

    def resid(p):
        C = Rotation.from_rotvec(p[:3]).as_matrix()
        t = p[3:]
        out = []
        for (P, u, v, R, tr) in corres:
            a = np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
            a /= np.linalg.norm(a)
            d = R @ C @ a                       # 世界系视线方向
            o = tr + t                          # 世界系相机原点
            # 点到视线的距离（比角度约束更稳）
            w = P - o
            out.append(np.cross(d, w))
        return np.concatenate(out)

    sol = least_squares(resid, np.concatenate([rv0, [0, 0, 0]]), method="lm", max_nfev=2000)
    C = Rotation.from_rotvec(sol.x[:3]).as_matrix()
    t = sol.x[3:]
    rms = float(np.sqrt(np.mean(np.array(sol.fun) ** 2)))
    print(f"标定完成: RMS 点到视线距离 {rms*100:.1f} cm, 平移偏移 t=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
    np.savez("camera_extrinsic_calibration.npz", C=C, t=t, rms=rms)
    print("已保存 camera_extrinsic_calibration.npz")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
