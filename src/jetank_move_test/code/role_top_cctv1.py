#!/usr/bin/env python3
# top_cctv1 카메라 이미지로 AI 추론을 수행하고
# /top_cctv1/get_closest_pose 서비스를 제공하는 역할.

import argparse

from role_dds_config import ensure_cyclonedds_env

ensure_cyclonedds_env()

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from top_cctv_infer.infer_service_server import InferServiceServer


class WebcamPublisher(Node):
    """노트북 웹캠을 /jetank/top_cctv1 이미지 토픽으로 퍼블리시."""

    def __init__(
        self,
        image_topic: str,
        camera_index: int,
        camera_width: int,
        camera_height: int,
        camera_fps: float,
        frame_id: str,
    ):
        super().__init__("top_cctv1_webcam")
        self.image_topic = image_topic
        self.frame_id = frame_id
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.image_topic, qos_profile_sensor_data)

        self.cap = cv2.VideoCapture(int(camera_index))
        if camera_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(camera_width))
        if camera_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(camera_height))
        if camera_fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, float(camera_fps))

        if not self.cap.isOpened():
            raise RuntimeError(f"camera open failed: index={camera_index}")

        period = 1.0 / camera_fps if camera_fps > 0 else 0.1
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"Webcam publishing to {self.image_topic} (idx={camera_index}, {camera_width}x{camera_height}@{camera_fps})"
        )

    def _on_timer(self) -> None:
        # 웹캠 프레임을 읽어서 ROS Image로 퍼블리시
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self) -> bool:
        # 종료 시 카메라 리소스 해제
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main() -> None:
    """top_cctv1 추론 서비스 노드를 실행."""
    parser = argparse.ArgumentParser(description="Top CCTV1 inference (with optional webcam)")
    parser.add_argument("--use-webcam", action="store_true")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--image-topic", type=str, default="/jetank/top_cctv1")
    parser.add_argument("--service-name", type=str, default="/top_cctv1/get_closest_pose")
    parser.add_argument("--frame-id", type=str, default="top_cctv1")

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    # top_cctv1 전용 서비스 노드 실행
    infer_node = InferServiceServer(
        node_name="top_cctv1_infer_service",
        image_topic=args.image_topic,
        service_name=args.service_name,
    )

    nodes = [infer_node]
    if args.use_webcam:
        cam_node = WebcamPublisher(
            image_topic=args.image_topic,
            camera_index=args.camera_index,
            camera_width=args.camera_width,
            camera_height=args.camera_height,
            camera_fps=args.camera_fps,
            frame_id=args.frame_id,
        )
        nodes.append(cam_node)

    executor = MultiThreadedExecutor(num_threads=len(nodes))
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
