import math
import json
import os
import threading
from typing import Dict, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import tf2_ros

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


def yaw_deg_to_quat(yaw_deg: float):
    yaw = math.radians(yaw_deg)
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)


class GoalShellNode(Node):

    def __init__(self):
        super().__init__('nav2_goal_shell')

        # Nav2 Action Client
        self._ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._current_goal_handle = None

        # TF Listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 프레임 설정
        self._global_frame = "map"
        self._base_frames = ["rc_car/base_link"]

        # 저장 파일 경로
        self._points_file = os.path.expanduser("~/.nav2_goal_points.json")

        self._default_points: Dict[str, Tuple[float, float, float, str]] = {

        }

        # 실제 포인트 저장소
        self._points: Dict[str, Tuple[float, float, float, str]] = dict(self._default_points)

        # 시작 시 파일 load 시도
        loaded = self._load_points()
        if loaded:
            self.get_logger().info(f"Load File: {self._points_file}")
        else:
            self.get_logger().warn(f"Failed to Load File: {self._points_file}")

        # 입력 루프는 별도 스레드
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

    # File Save
    def _save_points(self) -> bool:
        data = {}
        for name, (x, y, yaw, frame) in self._points.items():
            data[name] = {"x": x, "y": y, "yaw_deg": yaw, "frame_id": frame}

        tmp_path = self._points_file + ".tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self._points_file)  # 원자적 교체
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to Load File: {e}")
            try:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except Exception:
                pass
            return False

    # File Load
    def _load_points(self) -> bool:
        if not os.path.exists(self._points_file):
            return False

        try:
            with open(self._points_file, "r", encoding="utf-8") as f:
                data = json.load(f)

            # 검증 + 변환
            points: Dict[str, Tuple[float, float, float, str]] = {}
            for name, v in data.items():
                x = float(v["x"])
                y = float(v["y"])
                yaw = float(v["yaw_deg"])
                frame = str(v.get("frame_id", "map"))
                points[str(name)] = (x, y, yaw, frame)

            self._points = points
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to Load File: {e}")
            return False

    # Shell input
    def _input_loop(self):
        while rclpy.ok():
            try:
                line = input("goal-shell> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("exit", "quit"):
                self._save_points()
                rclpy.shutdown()
                return

            if cmd == "list":
                self._print_list()
                continue

            if cmd == "cancel":
                self._cancel_current_goal()
                continue

            if cmd == "set":
                if len(parts) != 2:
                    self.get_logger().warn("help: set <point_name>")
                    continue
                self._set_current_point(parts[1])
                continue

            if cmd == "goal":
                if len(parts) != 2:
                    self.get_logger().warn("help: goal <destination>")
                    continue
                self._send_goal_by_name(parts[1])
                continue

            if cmd == "delete":
                if len(parts) != 2:
                    self.get_logger().warn("help: delete <point_name>")
                    continue
                self._delete_point(parts[1])
                continue

            self.get_logger().warn(f"Unknown Command : {cmd}")


    def _print_list(self):
        print("Goal Point :")
        for name, (x, y, yaw, frame) in sorted(self._points.items(), key=lambda kv: kv[0]):
            print(f"  {name:12s} -> x={x:.3f}, y={y:.3f}, yaw={yaw:.1f}deg, frame={frame}")
        print("")

    def _delete_point(self, name: str):
        if name not in self._points:
            self.get_logger().warn(f"Goal Point not exist: {name}")
            return
        del self._points[name]
        self.get_logger().info(f"Delete : {name}")
        self._save_points()

    # TF -> 좌표 변환
    def _get_current_pose_from_tf(self):
        last_err = None

        for base in self._base_frames:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._global_frame,  # target frame
                    base,                # source frame
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0)
                )

                x = tf.transform.translation.x
                y = tf.transform.translation.y

                q = tf.transform.rotation
                yaw_deg = quat_to_yaw_deg(q.x, q.y, q.z, q.w)

                return (x, y, yaw_deg, self._global_frame, base)

            except Exception as e:
                last_err = e

        self.get_logger().error(
            f"Failed to get TF: {self._global_frame} -> {self._base_frames}"
            f"(Error: {last_err})"
        )
        return None

    def _set_current_point(self, name: str):
        cur = self._get_current_pose_from_tf()
        if cur is None:
            self.get_logger().error("Failed to get current position")
            return

        x, y, yaw_deg, frame_id, base_used = cur
        self._points[name] = (x, y, yaw_deg, frame_id)

        self.get_logger().info(
            f"save : {name} -> x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}deg, "
            f"frame={frame_id} (base={base_used})"
        )

        if self._save_points():
            self.get_logger().info(f"save : {self._points_file}")

    # Nav2 Goal
    def _cancel_current_goal(self):
        if self._current_goal_handle is None:
            self.get_logger().info("Goal Point not exist.")
            return

        self.get_logger().warn("Delete goal...")
        cancel_future = self._current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda f: self.get_logger().info("Success")
        )

    def _send_goal_by_name(self, dest: str):
        if dest not in self._points:
            self.get_logger().error(f"Goal Point not exist: {dest}")
            return

        if not self._ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Can't find action server : /navigate_to_pose")
            return

        x, y, yaw_deg, frame_id = self._points[dest]

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_deg_to_quat(float(yaw_deg))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f"goal '{dest}' 전송: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}deg, frame={frame_id}"
        )

        # 기존 goal 있으면 취소 시도(선택)
        if self._current_goal_handle is not None:
            self.get_logger().warn("Cancel Current Goal")
            self._cancel_current_goal()

        send_future = self._ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal refused")
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted. Start driving")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        res = future.result()
        if res is None:
            self.get_logger().error("Failed to receive nav2 result")
            self._current_goal_handle = None
            return

        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("GOAL SUCCEEDED")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("GOAL ABORTED")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("GOAL CANCELED")
        else:
            self.get_logger().warn(f"UNKNOWN STATUS: {status}")

        self._current_goal_handle = None


def main():
    rclpy.init()
    node = GoalShellNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 저장
        node._save_points()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
