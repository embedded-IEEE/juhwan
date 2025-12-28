#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Top-CCTV2 Image Publisher
- camera / video / image 디렉토리 입력을 받아 /jetank/top_cctv2 로 sensor_msgs/Image 퍼블리시
- top_cctv1 publisher 스타일을 최대한 유지하면서, 기본 토픽/노드명만 cctv2로 분리
"""

import os
import sys
import cv2

try:
    from role_dds_config import ensure_cyclonedds_env
    ensure_cyclonedds_env()
except Exception:
    pass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def get_path(file_name=None):
    p = os.path.dirname(os.path.abspath(__file__)).split("/")
    # cctv1 publisher가 lib 경로를 쓰는 형태를 유지
    return os.path.join("/", *p[1:4], "src", *p[5:6], *p[5:6], "lib", file_name) if file_name else ""


# --------------- Variable Setting (defaults) ---------------
PUB_TOPIC_NAME = "/jetank/top_cctv2"
DATA_SOURCE = "camera"          # camera | video | image
CAM_NUM = 0                     # /dev/video2 같은 장치 번호(환경에 맞게 조정)
IMAGE_DIRECTORY_PATH = "src/camera_perception_pkg/lib/Collected_Datasets/sample_dataset"
VIDEO_FILE_PATH = get_path("driving_simulation.mp4")
SHOW_IMAGE = False
TIMER = 0.03
# ---------------------------------------------------------


class ImagePublisherNodeCctv2(Node):
    def __init__(
        self,
        data_source: str = DATA_SOURCE,
        cam_num: int = CAM_NUM,
        img_dir: str = IMAGE_DIRECTORY_PATH,
        video_path: str = VIDEO_FILE_PATH,
        pub_topic: str = PUB_TOPIC_NAME,
        logger: bool = SHOW_IMAGE,
        timer: float = TIMER,
        frame_id: str = "top_cctv2_frame",
        width: int = 640,
        height: int = 480,
    ):
        super().__init__("image_publisher_node_cctv2")

        # Declare parameters (cctv1 스타일 유지)
        self.declare_parameter("data_source", data_source)
        self.declare_parameter("cam_num", cam_num)
        self.declare_parameter("img_dir", img_dir)
        self.declare_parameter("video_path", video_path)
        self.declare_parameter("pub_topic", pub_topic)
        self.declare_parameter("logger", logger)
        self.declare_parameter("timer", timer)
        self.declare_parameter("frame_id", frame_id)
        self.declare_parameter("width", width)
        self.declare_parameter("height", height)

        # Get parameters
        self.data_source = self.get_parameter("data_source").get_parameter_value().string_value
        self.cam_num = self.get_parameter("cam_num").get_parameter_value().integer_value
        self.img_dir = self.get_parameter("img_dir").get_parameter_value().string_value
        self.video_path = self.get_parameter("video_path").get_parameter_value().string_value
        self.pub_topic = self.get_parameter("pub_topic").get_parameter_value().string_value
        self.logger = self.get_parameter("logger").get_parameter_value().bool_value
        self.timer_period = self.get_parameter("timer").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)

        # QoS (cctv1 동일)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.br = CvBridge()

        # Select Data Source
        self.cap = None
        self.img_list = []
        self.img_num = 0

        if self.data_source == "camera":
            self.cap = self.open_camera_force(self.cam_num)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))

        elif self.data_source == "video":
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                self.get_logger().error(f"Cannot open video file: {self.video_path}")
                rclpy.shutdown()
                sys.exit(1)

        elif self.data_source == "image":
            if os.path.isdir(self.img_dir):
                self.img_list = sorted(os.listdir(self.img_dir))
                self.img_num = 0
            else:
                self.get_logger().error(f"Not a directory: {self.img_dir}")
                rclpy.shutdown()
                sys.exit(1)

        else:
            self.get_logger().error(f"Invalid data source: {self.data_source}")
            rclpy.shutdown()
            sys.exit(1)

        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(f"Publishing images to: {self.pub_topic} (source={self.data_source})")

    def open_camera_force(self, cam_num: int):
        """CAM_NUM만 무조건 재시도하는 함수(cctv1 로직 유지)."""
        while True:
            self.get_logger().info(f"Trying to open /dev/video{cam_num} ...")
            cap = cv2.VideoCapture(cam_num)

            if cap.isOpened():
                self.get_logger().info(f"Camera /dev/video{cam_num} opened successfully!")
                return cap

            self.get_logger().warn(f"Failed to open /dev/video{cam_num}. Retrying...")

    def _publish_frame(self, frame, src_frame_id: str):
        frame = cv2.resize(frame, (self.width, self.height))
        image_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id if self.frame_id else src_frame_id
        self.publisher.publish(image_msg)

        if self.logger:
            cv2.imshow("top_cctv2", frame)
            cv2.waitKey(1)

    def timer_callback(self):
        if self.data_source == "camera":
            ret, frame = self.cap.read() if self.cap is not None else (False, None)

            # read 실패 시 재연결
            if not ret or frame is None:
                self.get_logger().warn(f"Frame read failed. Reopening /dev/video{self.cam_num} ...")
                try:
                    if self.cap is not None:
                        self.cap.release()
                except Exception:
                    pass
                self.cap = self.open_camera_force(self.cam_num)
                return

            self._publish_frame(frame, "camera_frame")

        elif self.data_source == "image":
            if not self.img_list:
                return
            if self.img_num >= len(self.img_list):
                self.img_num = 0

            img_path = os.path.join(self.img_dir, self.img_list[self.img_num])
            img = cv2.imread(img_path)
            if img is not None:
                self._publish_frame(img, "image_frame")
            self.img_num += 1

        elif self.data_source == "video":
            ret, img = self.cap.read() if self.cap is not None else (False, None)
            if not ret or img is None:
                # 영상 끝이면 처음으로
                if self.cap is not None:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            self._publish_frame(img, "video_frame")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNodeCctv2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if hasattr(node, "cap") and node.cap is not None and node.cap.isOpened():
                node.cap.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
