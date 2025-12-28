#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import yaml

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from top_cctv_infer import get_path


class HomographyCalibrator(Node):
    def __init__(self):
        super().__init__("homography_calibrator_stable")

        self.IMAGE_TOPIC = "/jetank/top_cctv1"

        # world 좌표 (고정)
        self.PLATE_VISUAL = [
            [0.30, 0.20],
            [0.50, 0.20],
            [0.50, 0.40],
            [0.30, 0.40],
        ]

        self.YAML_PATH = get_path(file_name="coord.yaml")

        self.bridge = CvBridge()
        self.img = None
        self.display = None
        self.img_pts = []
        self.finished = False

        self.sub = self.create_subscription(
            Image,
            self.IMAGE_TOPIC,
            self.cb_image,
            10,
        )

        cv2.namedWindow("calib")
        cv2.setMouseCallback("calib", self.on_mouse)

        self.get_logger().info("Waiting for image...")

    def cb_image(self, msg):
        if self.img is not None:
            return

        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.display = self.img.copy()
        self.get_logger().info("Image received. Click 4 points.")

    def on_mouse(self, event, x, y, flags, param):
        if self.img is None or self.finished:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.img_pts.append([x, y])

            cv2.circle(self.display, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(
                self.display,
                f"{len(self.img_pts)}: ({x},{y})",
                (x + 6, y - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

            self.get_logger().info(f"Clicked {len(self.img_pts)}: ({x}, {y})")

            if len(self.img_pts) == len(self.PLATE_VISUAL):
                self.compute_homography()

    # ===================== 추가된 부분 =====================
    def print_img_pts_for_copy(self):
        print("\n===== IMAGE POINTS (COPY & PASTE) =====")
        print("img_pts = [")
        for x, y in self.img_pts:
            print(f"    [{x}, {y}],")
        print("]")
        print("======================================\n")
    # ====================================================

    def compute_homography(self):
        self.finished = True

        # 👉 클릭 좌표 복붙용 출력
        self.print_img_pts_for_copy()

        img_pts = np.array(self.img_pts, dtype=np.float32)
        robot_pts = np.array(self.PLATE_VISUAL, dtype=np.float32)

        H, _ = cv2.findHomography(img_pts, robot_pts)

        H_list = H.reshape(-1).tolist()

        yaml_data = {
            "pixel_to_robot": {
                "ros__parameters": {
                    "H": H_list
                }
            }
        }

        os.makedirs(os.path.dirname(self.YAML_PATH), exist_ok=True)
        with open(self.YAML_PATH, "w") as f:
            yaml.dump(yaml_data, f, sort_keys=False)

        print("\n==============================")
        print("Homography Matrix (3x3)")
        print(H)
        print("\nSaved to:")
        print(self.YAML_PATH)
        print("==============================\n")

        self.get_logger().info("Homography saved. Press ESC to exit.")

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.display is not None:
                cv2.imshow("calib", self.display)

            key = cv2.waitKey(30)
            if key == 27:
                break

        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = HomographyCalibrator()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
