#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[노트북에서 실행]
- 이미지 토픽을 구독해서 OpenCV 창에서 4개 픽셀 클릭
- 4개 클릭하면 px_points 문자열 출력 + (옵션) yaml 저장

사용:
  python3 laptop_click_4pts.py --image-topic /jetank/top_cctv1 --out-yaml ./px_points.yaml
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


@dataclass
class Sample:
    px: Tuple[float, float]


class LaptopClick4Pts(Node):
    def __init__(
        self,
        image_topic: str,
        out_yaml: str,
        window_name: str,
        max_points: int = 4,
    ):
        super().__init__("laptop_click_4pts")

        self.image_topic = image_topic
        self.out_yaml = out_yaml
        self.window_name = window_name
        self.max_points = int(max_points)

        self.bridge = CvBridge()
        self.frame = None
        self.samples: List[Sample] = []

        # click state
        self._want_click = False
        self._last_click: Optional[Tuple[int, int]] = None

        self.sub = self.create_subscription(Image, self.image_topic, self._on_image, 10)

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._on_mouse)

        self.get_logger().info(f"[Click4Pts] image_topic={self.image_topic}")
        self.get_logger().info("[Click4Pts] 창에서 4점을 순서대로 클릭하세요. (ESC: 종료)")

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
            cv2.putText(
                blank,
                "Waiting for image...",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
            )
            cv2.imshow(self.window_name, blank)
            return

        disp = self.frame.copy()

        # 이미 찍은 점 표시
        for i, s in enumerate(self.samples, start=1):
            x, y = int(s.px[0]), int(s.px[1])
            cv2.circle(disp, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(
                disp,
                f"{i} ({x},{y})",
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        # 안내 문구
        remain = self.max_points - len(self.samples)
        msg = f"Click point ({remain} left)" if self._want_click else "Press ESC to quit"
        cv2.putText(disp, msg, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
        cv2.imshow(self.window_name, disp)

    def _wait_for_click(self, idx: int) -> Tuple[int, int]:
        self._last_click = None
        self._want_click = True
        self.get_logger().info(f"[Click4Pts] Point {idx}/{self.max_points}: 화면에서 점을 클릭하세요.")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._render()
            key = cv2.waitKey(30)
            if key == 27:
                raise KeyboardInterrupt
            if self._last_click is not None:
                return self._last_click
        raise KeyboardInterrupt

    def run(self):
        # 이미지 대기
        t0 = time.time()
        while rclpy.ok() and self.frame is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._render()
            if time.time() - t0 > 10.0:
                self.get_logger().warn("아직 이미지가 안 들어옵니다. image_topic 확인하세요.")
                t0 = time.time()

        # 4점만 클릭
        for i in range(1, self.max_points + 1):
            px = self._wait_for_click(i)
            self.samples.append(Sample(px=(float(px[0]), float(px[1]))))
            self.get_logger().info(f"Saved {i}: px={self.samples[-1].px}")

        px_pts = np.array([s.px for s in self.samples], dtype=np.float32)
        px_points_text = ";".join([f"{int(x)},{int(y)}" for x, y in px_pts.tolist()])

        # (옵션) yaml 저장
        if self.out_yaml:
            out_dir = os.path.dirname(os.path.abspath(self.out_yaml))
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)

            data = {
                "camera_click_points": {
                    "ros__parameters": {
                        "image_topic": self.image_topic,
                        "px_points": px_points_text,
                    }
                }
            }
            with open(self.out_yaml, "w") as f:
                yaml.dump(data, f, sort_keys=False)

        print("\n==================== RESULT ====================")
        if self.out_yaml:
            print("Saved YAML:", self.out_yaml)
        print(f'px_points: "{px_points_text}"')
        print("================================================\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image-topic", default="/jetank/top_cctv1")
    ap.add_argument("--out-yaml", default="./px_points.yaml", help="빈 문자열이면 저장 안 함")
    ap.add_argument("--window-name", default="calib")
    ap.add_argument("--max-points", type=int, default=4)
    args = ap.parse_args()

    rclpy.init()
    node = LaptopClick4Pts(
        image_topic=args.image_topic,
        out_yaml=args.out_yaml,
        window_name=args.window_name,
        max_points=args.max_points,
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
