#!/usr/bin/env python3
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
from ultralytics import YOLO

from top_cctv_interfaces.srv import GetClosestPose
import os


def get_path(file_name=None):
    p = os.path.dirname(os.path.abspath(__file__)).split("/")
    LIB_PATH = os.path.join("/", *p[1:4], "src", *p[5:6], *p[5:6], "lib", file_name)
    return LIB_PATH

import cv2
import random
import numpy as np
from typing import List, Dict
import os
import sys
import signal
import atexit
import time
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks

from sensor_msgs.msg import Image
from top_cctv_interfaces.srv import GetClosestPose


import os


def get_path(file_name=None):
    p = os.path.dirname(os.path.abspath(__file__)).split("/")
    LIB_PATH = os.path.join("/", *p[1:4], "src", *p[5:6], *p[5:6],  file_name)
    return LIB_PATH


#---TODO-------------------------------------
SUB_TOPIC_NAME = '/jetank/top_cctv1'
PUB_TOPIC_NAME = '/top_cctv/annotated'

#DEVICE = "cpu"
DEVICE = "cuda:0"

MODEL_NAME = "best.pt"

SHOW_IMAGE = True
QUE = 1
CONF = 0.5
#--------------------------------------------

MODEL = get_path(file_name=MODEL_NAME)

            
class OBBInferNode(Node):
    def __init__(
        self,
        sub_topic=SUB_TOPIC_NAME,
        pub_topic=PUB_TOPIC_NAME,
        show_image=SHOW_IMAGE,
        que=QUE,
        model=MODEL,
        device=DEVICE,
        conf=CONF,
    ):
        super().__init__("test_node")
        
        # ===== params =====
        self.declare_parameter("sub_topic", sub_topic)
        self.declare_parameter("pub_topic", pub_topic)
        self.declare_parameter("show_image", show_image)
        self.declare_parameter("que", que)
        self.declare_parameter("model", model)
        self.declare_parameter("device", device)
        self.declare_parameter("conf", conf)
                
        self.sub_topic = self.get_parameter("sub_topic").value
        self.pub_topic = self.get_parameter("pub_topic").value
        self.show_image = bool(self.get_parameter("show_image").value)
        self.que = int(self.get_parameter("que").value)
        self.model_path = self.get_parameter("model").value
        self.device = self.get_parameter("device").value
        self.conf = float(self.get_parameter("conf").value)
               
        self.cv_bridge = CvBridge()
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=self.que,
        )
        
        self.model_path = get_path(self.model_path)
        self.model = YOLO(self.model_path)
        self.model.to(self.device)
        self.model.fuse()
        
        self.sub = self.create_subscription(Image, self.sub_topic, self.image_callback, image_qos)
        self.pub_img = self.create_publisher(Image, self.pub_topic, self.que)
        if self.show_image:
            cv2.namedWindow("top_cctv_obb_infer", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("top_cctv_obb_infer", 640, 360)

        self.cache = {"latest": (False, 0.0, 0.0, 0.0, 0.0)}
        self.srv = self.create_service(GetClosestPose, "/top_cctv/get_closest_pose", self.on_request)        
    
    def _leftmost_index(self, xywhr):
        return int(np.argmin(xywhr[:, 0]))

    def _draw_all_obb(self, vis, xywhr, left_idx):
        for i, (cx, cy, w, h, theta) in enumerate(xywhr):
            rect = (
                (float(cx), float(cy)),
                (float(w), float(h)),
                float(theta) * 180.0 / np.pi,
            )
            box = cv2.boxPoints(rect).astype(int)

            # 가장 왼쪽 것만 빨강
            color = (0, 0, 255) if i == left_idx else (0, 255, 0)
            thickness = 3 if i == left_idx else 2

            cv2.polylines(vis, [box], True, color, thickness)

            # ---- 각도 표시 (rad -> deg) ----
            angle_deg = theta * 180.0 / np.pi

            cv2.putText(
                vis,
                f"{angle_deg:.1f} deg",
                (int(cx) - 30, int(cy) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )


    def image_callback(self, msg: Image):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        vis = frame.copy()

        result = self.model.predict(
            frame,
            conf=CONF,
            device=DEVICE,
            verbose=False
        )[0]

        if result.obb is not None and result.obb.xywhr is not None:
            xywhr = result.obb.xywhr.detach().cpu().numpy()
            conf = (
                result.obb.conf.detach().cpu().numpy()
                if result.obb.conf is not None
                else None
            )

            if len(xywhr) > 0:
                left_idx = self._leftmost_index(xywhr)

                # 🔴 모든 OBB를 한 이미지에 전부 그림
                self._draw_all_obb(vis, xywhr, left_idx)
                
                cx, cy, _, _, theta = xywhr[left_idx]
                c = float(conf[left_idx]) if conf is not None else 0.0
                self.cache["latest"] = (True, float(cx), float(cy), float(theta), c)
            else:
                self.cache["latest"] = (False, 0.0, 0.0, 0.0, 0.0)
        else:
            self.cache["latest"] = (False, 0.0, 0.0, 0.0, 0.0)

        out_msg = self.cv_bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_msg.header = msg.header
        self.pub_img.publish(out_msg)

        if SHOW_IMAGE:
            cv2.imshow("top_cctv_obb_infer", vis)
            cv2.waitKey(1)

    def on_request(self, request, response):
        found, x, y, theta, conf = self.cache["latest"]
        response.found = found
        response.x = x
        response.y = y
        response.theta = theta
        response.conf = conf
        return response

    def cleanup(self):
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = OBBInferNode()

    def shutdown_handler(signum, frame):
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGTSTP, shutdown_handler)

    atexit.register(node.cleanup)

    try:
        rclpy.spin(node)
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
