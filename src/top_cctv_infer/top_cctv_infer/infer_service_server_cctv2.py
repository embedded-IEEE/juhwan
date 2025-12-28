#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Top-CCTV2 Infer Service Server (+ ROI Guard for Conveyor)

top_cctv1의 infer_service_server.py 기능을 기반으로 하되,
top_cctv2 역할(ROI 감지로 컨베이어 ON/OFF + ROI guard enable/disable)을 포함한다.

Top-CCTV2의 핵심 요구(사용자 플로우 반영):
- /jetank/top_cctv2 이미지 구독
- YOLO OBB로 젠가 추론
- ROI 안에 젠가가 "없다"가 일정 프레임 유지되면 -> 컨베이어 ON 요청
- ROI 안에 젠가가 "있다"가 일정 프레임 유지되면 -> 컨베이어 OFF 요청 + (옵션) ROI guard 자동 비활성화
- Jetank2가 다시 탐지를 재개할 수 있게 /top_cctv2/roi_guard_enable(SetBool) 서비스 제공
- Jetank2가 ROI 내 젠가 좌표를 가져갈 수 있게 /top_cctv2/get_closest_pose(GetClosestPose) 서비스 제공
"""

import os
import time
from typing import Dict, Optional, Tuple

import numpy as np
import cv2

try:
    from role_dds_config import ensure_cyclonedds_env
    ensure_cyclonedds_env()
except Exception:
    pass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from ultralytics import YOLO

from top_cctv_interfaces.srv import GetClosestPose


def get_path(file_name=None):
    # top_cctv1 infer_service_server.py의 get_path 스타일을 유지
    p = os.path.dirname(os.path.abspath(__file__)).split("/")
    return os.path.join("/", *p[1:4], "src", *p[5:6], *p[5:6], file_name) if file_name else ""


CacheItem = Tuple[bool, float, float, float, float, float]  # found,x,y,theta,conf,stamp


class TopCctv2InferServiceWithRoiGuard(Node):
    def __init__(self):
        super().__init__("top_cctv2_infer_service_server")

        # ====== basic params ======
        self.declare_parameter("image_topic", "/jetank/top_cctv2")
        self.declare_parameter("annot_topic", "/top_cctv2/annotated")
        self.declare_parameter("pose_topic", "/top_cctv2/closest_pose")
        self.declare_parameter("roi_hit_topic", "/top_cctv2/roi_hit")
        self.declare_parameter("service_name", "/top_cctv2/get_closest_pose")

        # infer_node 스타일 유지
        self.declare_parameter("weights", get_path("best.pt"))
        self.declare_parameter("conf", 0.50)
        self.declare_parameter("device", "cuda:0")  # cpu 가능

        # ====== selection params ======
        self.declare_parameter("ref_mode", "roi_center")  # top_cctv2는 ROI 중심 기준을 권장
        self.declare_parameter("ref_x", 0.0)
        self.declare_parameter("ref_y", 0.0)

        # ====== ROI params (top_cctv2는 기본적으로 ROI 기반 동작) ======
        self.declare_parameter("roi_xmin_ratio", 0.22)
        self.declare_parameter("roi_xmax_ratio", 0.40)
        self.declare_parameter("roi_ymin_ratio", 0.42)
        self.declare_parameter("roi_ymax_ratio", 0.58)

        # ====== theta unit ======
        self.declare_parameter("theta_unit", "rad")  # rad | deg

        # ====== ROI guard(컨베이어 제어) params ======
        self.declare_parameter("roi_guard_enabled", True)
        self.declare_parameter("roi_guard_auto_disable_on_stop", True)

        # "비어있음"이 유지되면 ON, "탐지됨"이 유지되면 OFF
        self.declare_parameter("empty_to_on_frames", 6)   # ROI에 젠가 없음이 연속 N프레임이면 ON
        self.declare_parameter("hit_to_off_frames", 6)    # ROI에 젠가 있음이 연속 N프레임이면 OFF

        self.declare_parameter("power_service", "/conveyor/request_power")  # SetBool
        self.declare_parameter("enable_service", "/top_cctv2/roi_guard_enable")  # SetBool
        self.declare_parameter("min_call_interval_sec", 0.2)

        # ====== topics/services ======
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.annot_topic = str(self.get_parameter("annot_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.roi_hit_topic = str(self.get_parameter("roi_hit_topic").value)
        self.service_name = str(self.get_parameter("service_name").value)

        weights = str(self.get_parameter("weights").value)
        self.conf = float(self.get_parameter("conf").value)
        self.device = str(self.get_parameter("device").value)

        self.bridge = CvBridge()
        self.model = YOLO(weights)

        # QoS (top_cctv1 서버와 동일하게 RELIABLE + depth=1)
        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos_img)
        self.pub_annot = self.create_publisher(Image, self.annot_topic, 10)
        self.pub_pose = self.create_publisher(Pose2D, self.pose_topic, 10)
        self.pub_roi = self.create_publisher(Bool, self.roi_hit_topic, 10)

        # AI 결과 서비스
        self.srv = self.create_service(GetClosestPose, self.service_name, self.on_request)

        # ROI guard enable 서비스 + 컨베이어 power 서비스 클라이언트
        self.power_service = str(self.get_parameter("power_service").value)
        self.enable_service = str(self.get_parameter("enable_service").value)
        self.cli_power = self.create_client(SetBool, self.power_service)
        self.srv_enable = self.create_service(SetBool, self.enable_service, self.on_enable)

        # 캐시: key=class_id (요청 클래스), value=(found,x,y,theta,conf,last_stamp)
        # -1 은 "전체 중 closest"
        self.cache: Dict[int, CacheItem] = {}
        self.cache[-1] = (False, 0.0, 0.0, 0.0, 0.0, 0.0)

        # ROI guard 상태/카운터
        self._guard_enabled = bool(self.get_parameter("roi_guard_enabled").value)
        self._power_on: Optional[bool] = None
        self._last_call_t = 0.0
        self._hit_count = 0
        self._empty_count = 0

        # FPS log
        self._last_t = time.time()
        self._cnt = 0

        self.get_logger().info(f"Subscribed: {self.image_topic}")
        self.get_logger().info(f"Annotated pub: {self.annot_topic}")
        self.get_logger().info(f"Pose pub: {self.pose_topic}")
        self.get_logger().info(f"ROI hit pub: {self.roi_hit_topic}")
        self.get_logger().info(f"AI Service: {self.service_name}")
        self.get_logger().info(f"ROI guard enable service: {self.enable_service}")
        self.get_logger().info(f"Conveyor power service: {self.power_service}")
        self.get_logger().info(f"Weights: {weights} / device={self.device} / conf={self.conf}")

    # ---------------- ROI helpers ----------------

    def _roi_bounds(self, frame_w: int, frame_h: int) -> Tuple[float, float, float, float]:
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

    def _roi_center(self, bounds: Tuple[float, float, float, float]) -> Tuple[float, float]:
        x_min, x_max, y_min, y_max = bounds
        return (x_min + x_max) / 2.0, (y_min + y_max) / 2.0

    def _roi_hit_any(self, bounds: Tuple[float, float, float, float], centers_xy: np.ndarray) -> bool:
        if centers_xy.size == 0:
            return False
        x_min, x_max, y_min, y_max = bounds
        cx = centers_xy[:, 0]
        cy = centers_xy[:, 1]
        hit = (cx >= x_min) & (cx <= x_max) & (cy >= y_min) & (cy <= y_max)
        return bool(hit.any())

    # ---------------- Conveyor power call ----------------

    def _call_conveyor_power(self, on: bool):
        if self._power_on is not None and self._power_on == on:
            return

        now = time.time()
        if now - self._last_call_t < float(self.get_parameter("min_call_interval_sec").value):
            return

        if not self.cli_power.service_is_ready():
            # 준비 안됐으면 조용히 패스(다음 프레임에서 재시도)
            return

        req = SetBool.Request()
        req.data = bool(on)
        fut = self.cli_power.call_async(req)
        self._last_call_t = now

        def _done_cb(f):
            try:
                resp = f.result()
                if resp is not None and resp.success:
                    self._power_on = on
                    self.get_logger().info(f"[ROI Guard] {self.power_service} -> {on} (ok) msg={resp.message}")

                    # OFF 성공이면, 플로우(10)처럼 ROI 탐지 멈춤 옵션
                    if (not on) and bool(self.get_parameter("roi_guard_auto_disable_on_stop").value):
                        self._set_guard_enabled(False, reason="auto-disable after stop")
                else:
                    self.get_logger().warn(
                        f"[ROI Guard] {self.power_service} -> {on} (fail) msg={resp.message if resp else 'None'}"
                    )
            except Exception as exc:
                self.get_logger().error(f"[ROI Guard] service call exception: {exc}")

        fut.add_done_callback(_done_cb)

    def _set_guard_enabled(self, enabled: bool, reason: str):
        self._guard_enabled = bool(enabled)
        self._hit_count = 0
        self._empty_count = 0
        self.get_logger().info(f"[ROI Guard] enabled={self._guard_enabled} ({reason})")

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self._set_guard_enabled(bool(request.data), reason="service")
        response.success = True
        response.message = f"enabled={self._guard_enabled}"
        return response

    # ---------------- Closest selection ----------------

    def _closest_for_class(
        self,
        xywhr: np.ndarray,
        cls: Optional[np.ndarray],
        conf: Optional[np.ndarray],
        target_class: int,
        ref_x: float,
        ref_y: float,
        valid_idxs: Optional[np.ndarray] = None,
    ) -> Tuple[bool, float, float, float, float]:
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
        # ROI guard가 꺼져 있어도 "좌표 서비스"는 계속 갱신(원하면 이 줄을 바꿔도 됨)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        h, w = frame.shape[:2]
        bounds = self._roi_bounds(w, h)
        ref_x, ref_y = self._roi_center(bounds)

        # ---- YOLO 추론 ----
        try:
            results = self.model.predict(frame, conf=self.conf, device=self.device, verbose=False)
        except Exception as e:
            self.get_logger().warn(f"YOLO predict failed: {e}")
            return

        annotated = frame.copy()

        # ROI 영역 시각화(디버그)
        x_min, x_max, y_min, y_max = bounds
        cv2.rectangle(
            annotated,
            (int(x_min), int(y_min)),
            (int(x_max), int(y_max)),
            (255, 255, 0),
            2,
        )

        now_stamp = time.time()

        found = False
        cx = cy = theta = c = 0.0
        centers_xy = np.zeros((0, 2), dtype=np.float32)

        if results and len(results) > 0 and hasattr(results[0], "obb") and results[0].obb is not None:
            obb = results[0].obb
            xywhr = obb.xywhr.cpu().numpy()  # (N,5) = [cx, cy, w, h, theta]
            cls = obb.cls.cpu().numpy().astype(int) if obb.cls is not None else None
            conf = obb.conf.cpu().numpy() if obb.conf is not None else None

            if xywhr is not None and xywhr.size > 0:
                centers_xy = xywhr[:, 0:2].astype(np.float32)

            # ROI 필터 idx 구성 (top_cctv2는 서비스/로직 모두 ROI 기준)
            roi_idxs = None
            if xywhr is not None and xywhr.size > 0:
                cx_arr = xywhr[:, 0]
                cy_arr = xywhr[:, 1]
                roi_mask = (cx_arr >= x_min) & (cx_arr <= x_max) & (cy_arr >= y_min) & (cy_arr <= y_max)
                roi_idxs = np.where(roi_mask)[0]

            # 전체(-1) closest(ROI 내부에서)
            found, cx, cy, theta, c = self._closest_for_class(
                xywhr, cls, conf, target_class=-1, ref_x=ref_x, ref_y=ref_y, valid_idxs=roi_idxs
            )
            self.cache[-1] = (found, cx, cy, theta, c, now_stamp)

            # 클래스별 캐시
            if cls is not None:
                for cc in np.unique(cls):
                    f2, x2, y2, th2, c2 = self._closest_for_class(
                        xywhr, cls, conf, target_class=int(cc), ref_x=ref_x, ref_y=ref_y, valid_idxs=roi_idxs
                    )
                    self.cache[int(cc)] = (f2, x2, y2, th2, c2, now_stamp)

            # 디버그: best(-1) 하이라이트(ROI 내부 best)
            if found and obb.xyxyxyxy is not None and len(obb.xyxyxyxy) > 0:
                valid = np.arange(xywhr.shape[0]) if roi_idxs is None else np.asarray(roi_idxs, dtype=np.int64)
                if valid.size > 0:
                    centers = xywhr[valid, 0:2]
                    d2 = (centers[:, 0] - ref_x) ** 2 + (centers[:, 1] - ref_y) ** 2
                    best_i = int(valid[int(np.argmin(d2))])

                    poly = obb.xyxyxyxy[best_i].cpu().numpy().reshape(4, 2).astype(np.int32)
                    cv2.polylines(annotated, [poly], isClosed=True, color=(0, 255, 255), thickness=3)
                    cv2.circle(annotated, (int(cx), int(cy)), 5, (0, 255, 255), -1)

                    label = f"conf={c:.2f} cx={cx:.1f} cy={cy:.1f} th={theta:.3f}{self.get_parameter('theta_unit').value}"
                    cv2.putText(
                        annotated,
                        label,
                        (int(cx) + 10, int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )

                    # pose publish
                    pose = Pose2D()
                    pose.x = float(cx)
                    pose.y = float(cy)
                    pose.theta = float(theta)
                    self.pub_pose.publish(pose)

        else:
            # 탐지 실패면 캐시 갱신(ROI 내부도 실패)
            self.cache[-1] = (False, 0.0, 0.0, 0.0, 0.0, now_stamp)

        # ---- ROI hit publish ----
        roi_hit = self._roi_hit_any(bounds, centers_xy)
        msg_roi = Bool()
        msg_roi.data = bool(roi_hit)
        self.pub_roi.publish(msg_roi)

        # ---- ROI guard (컨베이어 ON/OFF) ----
        if self._guard_enabled:
            empty_to_on = int(self.get_parameter("empty_to_on_frames").value)
            hit_to_off = int(self.get_parameter("hit_to_off_frames").value)

            if roi_hit:
                self._hit_count += 1
                self._empty_count = 0
                if self._hit_count >= hit_to_off:
                    self._call_conveyor_power(False)
            else:
                self._empty_count += 1
                self._hit_count = 0
                if self._empty_count >= empty_to_on:
                    self._call_conveyor_power(True)

        # annotated publish
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub_annot.publish(out)

        # FPS log
        self._cnt += 1
        t = time.time()
        if t - self._last_t >= 1.0:
            self.get_logger().info(
                f"FPS: {self._cnt/(t-self._last_t):.1f} | roi_hit={roi_hit} | found_roi_best={found} conf={c:.2f}"
            )
            self._last_t = t
            self._cnt = 0

    def on_request(self, request: GetClosestPose.Request, response: GetClosestPose.Response):
        key = int(request.target_class)
        if key not in self.cache:
            key = -1

        found, x, y, theta, conf, _stamp = self.cache.get(key, (False, 0.0, 0.0, 0.0, 0.0, 0.0))
        response.found = bool(found)
        response.x = float(x)
        response.y = float(y)
        response.theta = float(theta)
        response.conf = float(conf)
        return response


def main():
    rclpy.init()
    node = TopCctv2InferServiceWithRoiGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
