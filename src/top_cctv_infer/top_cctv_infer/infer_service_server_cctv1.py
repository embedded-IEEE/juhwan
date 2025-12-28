#!/usr/bin/env python3
import time
import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
from ultralytics import YOLO

from top_cctv_interfaces.srv import GetClosestPose


def get_path(file_name=None):
    # infer_node.py의 get_path 스타일을 최대한 유지
    p = os.path.dirname(os.path.abspath(__file__)).split("/")
    LIB_PATH = os.path.join("/", *p[1:4], "src", *p[5:6], *p[5:6], file_name)
    return LIB_PATH


class TopCctvInferService(Node):
    """
    infer_node.py 로직 그대로:
      - /jetank/top_cctv1 이미지 구독
      - YOLO OBB로 closest 선택
      - /top_cctv1/annotated 퍼블리시
      - /top_cctv1/closest_pose 퍼블리시(선택, 디버깅 용)
    + 추가:
      - /top_cctv1/get_closest_pose 서비스 제공 (캐시된 closest 결과 반환)
    """

    def __init__(self):
        super().__init__("top_cctv_infer_service_server")

        # ====== basic params ======
        self.declare_parameter("image_topic", "/jetank/top_cctv1")
        self.declare_parameter("annot_topic", "/top_cctv1/annotated")
        self.declare_parameter("pose_topic", "/top_cctv1/closest_pose")
        self.declare_parameter("service_name", "/top_cctv1/get_closest_pose")

        # infer_node.py처럼 get_path 기본 사용 (원하면 절대경로로 바꿔도 됨)
        self.declare_parameter("weights", get_path("best.pt"))
        self.declare_parameter("conf", 0.50)
        self.declare_parameter("device", "cuda:0")  # cpu 가능

        # ====== selection params ======
        self.declare_parameter("ref_mode", "center")  # infer_node 기본
        self.declare_parameter("ref_x", 0.0)
        self.declare_parameter("ref_y", 0.0)

        # ====== ROI params ======
        self.declare_parameter("roi_enabled", True)
        self.declare_parameter("roi_mode", "auto")  # auto|on|off|param
        self.declare_parameter("roi_xmin_ratio", 0.22)
        self.declare_parameter("roi_xmax_ratio", 0.40)
        self.declare_parameter("roi_ymin_ratio", 0.42)
        self.declare_parameter("roi_ymax_ratio", 0.58)

        # ====== request/pose params ======
        self.declare_parameter("theta_unit", "rad")  # rad | deg

        # ====== topics/services ======
        self.image_topic = self.get_parameter("image_topic").value
        self.annot_topic = self.get_parameter("annot_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.service_name = self.get_parameter("service_name").value

        weights = self.get_parameter("weights").value
        self.conf = float(self.get_parameter("conf").value)
        self.device = self.get_parameter("device").value

        self.bridge = CvBridge()
        self.model = YOLO(weights)

        # infer_node.py의 QoS를 그대로 따라감 (RELIABLE + depth=1)
        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos_img)
        self.pub_annot = self.create_publisher(Image, self.annot_topic, 10)
        self.pub_pose = self.create_publisher(Pose2D, self.pose_topic, 10)

        # 서비스 서버
        self.srv = self.create_service(GetClosestPose, self.service_name, self.on_request)

        # 캐시: key=class_id (요청 클래스), value=(found,x,y,theta,conf,last_stamp)
        # -1 은 "전체 중 closest"
        self.cache = {}
        self.cache[-1] = (False, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.last_t = time.time()
        self.cnt = 0

        self.get_logger().info(f"Subscribed: {self.image_topic}")
        self.get_logger().info(f"Annotated pub: {self.annot_topic}")
        self.get_logger().info(f"Pose pub: {self.pose_topic}")
        self.get_logger().info(f"Service: {self.service_name}")
        self.get_logger().info(f"Weights: {weights} / device={self.device} / conf={self.conf}")

    # ===== infer_node.py 로직 최대한 유지 =====

    def _compute_ref(self, frame):
        """Return (ref_x, ref_y) in pixel coordinates."""
        h, w = frame.shape[:2]
        ref_mode = str(self.get_parameter("ref_mode").value)

        if ref_mode == "left_middle":
            return 0.0, h / 2.0
        if ref_mode == "center":
            # infer_node.py 그대로: (w/2, h)  <-- 이게 네 환경에서 잘 됐던 기준일 가능성 큼
            return w / 2.0, h
        if ref_mode == "down_middle":
            return float(w) / 2.0, 0.0
        if ref_mode == "right_middle":
            return float(w), h / 2.0
        if ref_mode == "up_middle":
            return 0.0, float(h) / 2.0

        return float(self.get_parameter("ref_x").value), float(self.get_parameter("ref_y").value)

    def _roi_bounds(self, frame_w: int, frame_h: int):
        xmin_r = float(self.get_parameter("roi_xmin_ratio").value)
        xmax_r = float(self.get_parameter("roi_xmax_ratio").value)
        ymin_r = float(self.get_parameter("roi_ymin_ratio").value)
        ymax_r = float(self.get_parameter("roi_ymax_ratio").value)
        x_min = frame_w * xmin_r
        x_max = frame_w * xmax_r
        y_min = frame_h * ymin_r
        y_max = frame_h * ymax_r
        if x_min > x_max:
            x_min, x_max = x_max, x_min
        if y_min > y_max:
            y_min, y_max = y_max, y_min
        return x_min, x_max, y_min, y_max

    def _roi_center(self, bounds):
        x_min, x_max, y_min, y_max = bounds
        return (x_min + x_max) / 2.0, (y_min + y_max) / 2.0

    def _resolve_roi_enabled(self) -> bool:
        mode = str(self.get_parameter("roi_mode").value).strip().lower()
        if mode == "auto":
            return "top_cctv2" in self.image_topic
        if mode == "on":
            return True
        if mode == "off":
            return False
        return bool(self.get_parameter("roi_enabled").value)

    def _closest_for_class(self, xywhr, cls, conf, target_class, ref_x, ref_y, valid_idxs=None):
        if valid_idxs is None:
            idxs = np.arange(xywhr.shape[0])
        else:
            idxs = np.asarray(valid_idxs, dtype=np.int64)

        if cls is not None and target_class >= 0:
            idxs = idxs[cls[idxs] == target_class]

        if idxs.size == 0:
            return (False, 0.0, 0.0, 0.0, 0.0)

        centers = xywhr[idxs, 0:2]
        d2 = (centers[:, 0] - ref_x) ** 2 + (centers[:, 1] - ref_y) ** 2
        best_i = int(idxs[int(np.argmin(d2))])

        cx, cy, _, _, theta = xywhr[best_i].tolist()
        c = float(conf[best_i]) if conf is not None else 0.0

        theta_out = float(theta)
        if str(self.get_parameter("theta_unit").value) == "deg":
            theta_out = float(theta) * 180.0 / np.pi

        return (True, float(cx), float(cy), float(theta_out), float(c))

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        ref_x, ref_y = self._compute_ref(frame)
        roi_bounds = None
        roi_idxs = None
        roi_enabled = self._resolve_roi_enabled()
        if roi_enabled:
            roi_bounds = self._roi_bounds(frame.shape[1], frame.shape[0])
            ref_x, ref_y = self._roi_center(roi_bounds)

        # ---- inference (OBB) ----
        result = self.model.predict(frame, conf=self.conf, device=self.device, verbose=False)[0]
        obb = getattr(result, "obb", None)
        annotated = frame.copy()

        # ROI 표시 + 기준점 표시
        if roi_bounds is not None:
            x_min, x_max, y_min, y_max = roi_bounds
            cv2.rectangle(annotated, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 255), 2)
        cv2.circle(annotated, (int(ref_x), int(ref_y)), 8, (0, 0, 255), -1)

        now_stamp = float(self.get_clock().now().nanoseconds) * 1e-9

        if obb is None or obb.xywhr is None or len(obb.xywhr) == 0:
            # 캐시 업데이트: 전체(-1)는 found False
            self.cache[-1] = (False, 0.0, 0.0, 0.0, 0.0, now_stamp)
            out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header = msg.header
            self.pub_annot.publish(out)
            return

        xywhr = obb.xywhr.cpu().numpy()  # (N,5) = [cx, cy, w, h, theta]
        cls = obb.cls.cpu().numpy().astype(int) if obb.cls is not None else None
        conf = obb.conf.cpu().numpy() if obb.conf is not None else None

        # ROI 필터 idx 구성
        if roi_bounds is not None:
            x_min, x_max, y_min, y_max = roi_bounds
            cx_arr = xywhr[:, 0]
            cy_arr = xywhr[:, 1]
            roi_mask = (cx_arr >= x_min) & (cx_arr <= x_max) & (cy_arr >= y_min) & (cy_arr <= y_max)
            roi_idxs = np.where(roi_mask)[0]

        # 전체(-1) closest
        found, cx, cy, theta, c = self._closest_for_class(
            xywhr, cls, conf, target_class=-1, ref_x=ref_x, ref_y=ref_y, valid_idxs=roi_idxs
        )
        self.cache[-1] = (found, cx, cy, theta, c, now_stamp)

        # 클래스별 캐시도 같이 채움(요청 target_class 대응)
        if cls is not None:
            for cc in np.unique(cls):
                f2, x2, y2, th2, c2 = self._closest_for_class(
                    xywhr, cls, conf, target_class=int(cc), ref_x=ref_x, ref_y=ref_y, valid_idxs=roi_idxs
                )
                self.cache[int(cc)] = (f2, x2, y2, th2, c2, now_stamp)

        # 디버그: best(-1)만 하이라이트( infer_node 방식 )
        if found and obb.xyxyxyxy is not None and len(obb.xyxyxyxy) > 0:
            # best index를 다시 찾기 위해 -1 기준으로 동일 로직 재계산(가독성 위해 단순화)
            valid = np.arange(xywhr.shape[0]) if roi_idxs is None else np.asarray(roi_idxs, dtype=np.int64)
            centers = xywhr[valid, 0:2]
            d2 = (centers[:, 0] - ref_x) ** 2 + (centers[:, 1] - ref_y) ** 2
            best_i = int(valid[int(np.argmin(d2))])

            poly = obb.xyxyxyxy[best_i].cpu().numpy().reshape(4, 2).astype(np.int32)
            cv2.polylines(annotated, [poly], isClosed=True, color=(0, 255, 255), thickness=3)
            cv2.circle(annotated, (int(cx), int(cy)), 5, (0, 255, 255), -1)

            label = f"conf={c:.2f} cx={cx:.1f} cy={cy:.1f} th={theta:.3f}{self.get_parameter('theta_unit').value}"
            cv2.putText(
                annotated, label, (int(cx) + 10, int(cy) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
            )

            # pose publish (infer_node와 동일하게)
            pose = Pose2D()
            pose.x = float(cx)
            pose.y = float(cy)
            pose.theta = float(theta)
            self.pub_pose.publish(pose)

        # annotated publish
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub_annot.publish(out)

        # FPS log (infer_node 방식)
        self.cnt += 1
        t = time.time()
        if t - self.last_t >= 1.0:
            self.get_logger().info(
                f"FPS: {self.cnt/(t-self.last_t):.1f} | found={found} conf={c:.2f} ({cx:.1f},{cy:.1f})"
            )
            self.last_t = t
            self.cnt = 0

    def on_request(self, request: GetClosestPose.Request, response: GetClosestPose.Response):
        key = int(request.target_class)
        if key not in self.cache:
            key = -1

        found, x, y, theta, conf, stamp = self.cache.get(key, (False, 0.0, 0.0, 0.0, 0.0, 0.0))
        response.found = bool(found)
        response.x = float(x)
        response.y = float(y)
        response.theta = float(theta)
        response.conf = float(conf)
        return response


def main():
    rclpy.init()
    node = TopCctvInferService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
