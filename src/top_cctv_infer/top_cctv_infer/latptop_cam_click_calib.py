#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[노트북에서 실행]
- 이미지 토픽을 구독해서 OpenCV 창에서 4개 픽셀 클릭
- 각 클릭마다 터미널 입력(x y z [phi] [roll] [t])으로 로봇을 움직이게 "서비스 호출"
- ok 입력 시 그때의 (x,y)를 world_points에 저장
- 4개면 Homography(H) 계산 + yaml 저장 + px_points/world_points 문자열 출력

요구:
- Jetank에서 jetank_calib_move_server.py 실행 중이어야 함
  (/jetank1/calib/move_to_xyz 서비스 제공)
- MoveToXYZ.srv 인터페이스가 양쪽(노트북/jetank)에서 빌드되어 있어야 함
"""

import argparse
import os
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from top_cctv_interfaces.srv import MoveToXYZ  # type: ignore


@dataclass
class Sample:
    px: Tuple[float, float]
    world_xy: Tuple[float, float]


class LaptopCalib(Node):
    def __init__(
        self,
        image_topic: str,
        robot_name: str,
        out_yaml: str,
        phi: float,
        move_time: float,
        window_name: str,
    ):
        super().__init__("laptop_cam_click_calib")

        self.image_topic = image_topic
        self.robot_name = robot_name
        self.out_yaml = out_yaml
        self.phi = float(phi)
        self.default_move_time = float(move_time)
        self.window_name = window_name

        self.bridge = CvBridge()
        self.frame = None
        self.samples: List[Sample] = []

        # click state
        self._want_click = False
        self._last_click: Optional[Tuple[int, int]] = None

        self.sub = self.create_subscription(Image, self.image_topic, self._on_image, 10)

        # move service client
        self.srv_name = f"/{robot_name}/calib/move_to_xyz"
        self.cli = self.create_client(MoveToXYZ, self.srv_name)

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._on_mouse)

        self.get_logger().info(f"[Calib] image_topic={self.image_topic}")
        self.get_logger().info(f"[Calib] move_service={self.srv_name}")

    def _on_image(self, msg: Image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")

    def _on_mouse(self, event, x, y, flags, param):
        if not self._want_click:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self._last_click = (int(x), int(y))
            self._want_click = False

    def _render(self):
        if self.frame is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for image...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.imshow(self.window_name, blank)
            return

        disp = self.frame.copy()
        for i, s in enumerate(self.samples, start=1):
            x, y = int(s.px[0]), int(s.px[1])
            cv2.circle(disp, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(disp, f"{i} ({x},{y})", (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        msg = "Click next point" if self._want_click else "Press ESC to quit"
        cv2.putText(disp, msg, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
        cv2.imshow(self.window_name, disp)

    def _wait_for_click(self, idx: int) -> Tuple[int, int]:
        self._last_click = None
        self._want_click = True
        self.get_logger().info(f"[Calib] Point {idx}/4: 화면에서 점을 클릭하세요.")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._render()
            key = cv2.waitKey(30)
            if key == 27:
                raise KeyboardInterrupt
            if self._last_click is not None:
                return self._last_click
        raise KeyboardInterrupt

    def _call_move(self, x: float, y: float, z: float, phi: float, roll: float, t: float) -> bool:
        if not self.cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("move service not available yet...")
            return False

        req = MoveToXYZ.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.phi = float(phi)
        req.roll = float(roll)
        req.move_time = float(t)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(2.0, t + 3.0))
        if future.result() is None:
            self.get_logger().warn("service call failed (timeout or exception)")
            return False
        res = future.result()
        if not res.success:
            self.get_logger().warn(f"move rejected: {res.message}")
        return bool(res.success)

    def _interactive_world_xy(self, px: Tuple[int, int]) -> Tuple[float, float]:
        last_xy: Optional[Tuple[float, float]] = None
        print("\n------------------------------------------------------------")
        print(f"[World Match] px=({px[0]}, {px[1]})")
        print("입력: x y z [phi] [roll] [t]   /  ok  /  q")
        print("예: 150 0 50")
        print("------------------------------------------------------------")
        while True:
            line = input("Target >> ").strip().lower()
            if line in ("q", "quit", "exit"):
                raise KeyboardInterrupt
            if line == "ok":
                if last_xy is None:
                    print("[Warn] 아직 이동을 안 했어요. 좌표를 먼저 입력하세요.")
                    continue
                return last_xy

            parts = line.replace(",", " ").split()
            if len(parts) < 3:
                print("[Error] 최소 3개 필요: x y z")
                continue
            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                phi = float(parts[3]) if len(parts) >= 4 else float(self.phi)
                roll = float(parts[4]) if len(parts) >= 5 else 0.0
                t = float(parts[5]) if len(parts) >= 6 else float(self.default_move_time)
            except ValueError as e:
                print(f"[Error] 파싱 실패: {e}")
                continue

            ok = self._call_move(x, y, z, phi, roll, t)
            if ok:
                last_xy = (x, y)
                time.sleep(max(0.0, t + 0.2))
                print(f">> moved: x={x} y={y} z={z} phi={phi} roll={roll} t={t}")
            else:
                print(">> move 실패(도달 불가/서비스 문제).")

    def run(self):
        # 이미지 대기
        t0 = time.time()
        while rclpy.ok() and self.frame is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._render()
            if time.time() - t0 > 10.0:
                self.get_logger().warn("아직 이미지가 안 들어옵니다. image_topic 확인하세요.")
                t0 = time.time()

        for i in range(1, 5):
            px = self._wait_for_click(i)
            world_xy = self._interactive_world_xy(px)
            self.samples.append(Sample(px=(float(px[0]), float(px[1])), world_xy=(float(world_xy[0]), float(world_xy[1]))))
            self.get_logger().info(f"Saved {i}: px={self.samples[-1].px} -> world={self.samples[-1].world_xy}")

        px_pts = np.array([s.px for s in self.samples], dtype=np.float32)
        world_pts = np.array([s.world_xy for s in self.samples], dtype=np.float32)

        H, _ = cv2.findHomography(px_pts, world_pts)
        if H is None:
            raise RuntimeError("Homography 계산 실패")

        Hinv = np.linalg.inv(H)

        px_points_text = ";".join([f"{int(x)},{int(y)}" for x, y in px_pts.tolist()])
        world_points_text = ";".join([f"{x:.3f},{y:.3f}" for x, y in world_pts.tolist()])

        out_dir = os.path.dirname(os.path.abspath(self.out_yaml))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        data = {
            "camera_world_calib": {
                "ros__parameters": {
                    "image_topic": self.image_topic,
                    "robot_name": self.robot_name,
                    "H": H.reshape(-1).tolist(),
                    "H_inv": Hinv.reshape(-1).tolist(),
                    "px_points": px_points_text,
                    "world_points": world_points_text,
                    "units": "mm",
                }
            }
        }
        with open(self.out_yaml, "w") as f:
            yaml.dump(data, f, sort_keys=False)

        print("\n==================== RESULT ====================")
        print("Saved YAML:", self.out_yaml)
        print(f'px_points: "{px_points_text}"')
        print(f'world_points: "{world_points_text}"')
        print("================================================\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image-topic", default="/jetank/top_cctv1")
    ap.add_argument("--robot-name", default="jetank1")
    ap.add_argument("--out-yaml", default="./camera_world_calib.yaml")
    ap.add_argument("--phi", type=float, default=-90.0)
    ap.add_argument("--move-time", type=float, default=2.0)
    ap.add_argument("--window-name", default="calib")
    args = ap.parse_args()

    rclpy.init()
    node = LaptopCalib(
        image_topic=args.image_topic,
        robot_name=args.robot_name,
        out_yaml=args.out_yaml,
        phi=args.phi,
        move_time=args.move_time,
        window_name=args.window_name,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
