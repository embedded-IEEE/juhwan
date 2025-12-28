import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

WHEEL_BASE = 0.30   # 차량 앞뒤 바퀴 간 거리 (L)
TRACK_WIDTH = 0.20  # 차량 좌우 폭 (T)
WHEEL_RADIUS = 0.07 # 바퀴 반지름 (R) - 이 값이 정확해야 합니다.
# ----------------------------------------------------
# 스케일 / 제한값 (조정해서 과도한 조향/가속을 줄입니다)
MAX_STEER = 0.35          # rad, 조향 최대 각도
MAX_WHEEL_SPEED = 15.0    # rad/s, 각 바퀴 최대 속도
VEL_SCALE = 1.0           # 선형속도 스케일 (0~1), 기본은 1.0배
OMEGA_SCALE = 1.0         # 각속도 스케일 (0~1), 기본은 1.0배
DRIVE_SIGN = 1.0          # +1.0이면 그대로, -1.0이면 앞/뒤 반전
MIN_CURV_LINEAR = 0.05    # 조향 각 계산용 최소 선속도 (0일 때도 핸들이 돌아가도록)
EPS = 1e-3

class TwistToJointCmd(Node):
    def __init__(self):
        super().__init__('twist_to_joint_cmd')

        # rqt_robot_steering 에서 나오는 기본 topic
        self.sub = self.create_subscription(
            Twist, "cmd_vel", self.twist_callback, 10)

        # steering controller (left, right) - 위치(position) 명령
        self.pub_steer = self.create_publisher(
            Float64MultiArray, "steering_controller/commands", 10)

        # wheel controller (4개 바퀴) - 속도(velocity, rad/s) 명령
        self.pub_wheel = self.create_publisher(
            Float64MultiArray, "wheel_controller/commands", 10)

        # drive_sign을 파라미터로 노출 (nav2/Gazebo 좌표계 차이를 맞출 때 사용)
        self.declare_parameter('drive_sign', DRIVE_SIGN)
        self.drive_sign = float(self.get_parameter('drive_sign').value)

        self.get_logger().info("TwistToJointCmd node started. Correcting m/s to rad/s.")

    def twist_callback(self, msg: Twist):
        # 입력 스케일을 적용해 과도한 명령을 줄임
        v = msg.linear.x * VEL_SCALE    # 선형 속도 (v, m/s)
        omega = msg.angular.z * OMEGA_SCALE # 각속도 (omega, rad/s)

        # 각 바퀴의 각속도 (rad/s)를 저장할 변수 초기화
        w_fl = 0.0
        w_fr = 0.0
        w_rl = 0.0
        w_rr = 0.0
        steer_left = 0.0
        steer_right = 0.0

        # 1) 조향 각 계산 (선속도가 0이라도 핸들은 돌아가도록)
        if abs(omega) >= EPS:
            # 조향각 계산용으로 최소 선속도를 강제로 부여해 반경을 안정화
            v_for_curvature = v
            if abs(v_for_curvature) < MIN_CURV_LINEAR:
                v_for_curvature = math.copysign(MIN_CURV_LINEAR, v if abs(v) >= EPS else omega)

            curvature = omega / v_for_curvature  # 곡률 k = omega / v
            radius_abs = 1.0 / max(abs(curvature), EPS)
            radius_abs = max(radius_abs, TRACK_WIDTH / 2.0 + 1e-3)  # 안쪽 반경이 0 이하로 가지 않도록 보호

            inner_angle = math.atan(WHEEL_BASE / (radius_abs - (TRACK_WIDTH / 2.0)))
            outer_angle = math.atan(WHEEL_BASE / (radius_abs + (TRACK_WIDTH / 2.0)))

            if curvature > 0:  # 좌회전 (CCW)
                steer_left = inner_angle
                steer_right = outer_angle
                v_left = v * (radius_abs - (TRACK_WIDTH / 2.0)) / radius_abs   # 안쪽
                v_right = v * (radius_abs + (TRACK_WIDTH / 2.0)) / radius_abs  # 바깥쪽
            else:          # 우회전 (CW)
                steer_left = -outer_angle
                steer_right = -inner_angle
                v_left = v * (radius_abs + (TRACK_WIDTH / 2.0)) / radius_abs   # 바깥쪽
                v_right = v * (radius_abs - (TRACK_WIDTH / 2.0)) / radius_abs  # 안쪽

            steer_left = max(min(steer_left, MAX_STEER), -MAX_STEER)
            steer_right = max(min(steer_right, MAX_STEER), -MAX_STEER)

            # 2) 개별 바퀴 선속도 계산 (좌/우를 내·외측에 맞게 배분)
            v_rl = v_left
            v_rr = v_right
            v_fl = v_left / math.cos(steer_left)
            v_fr = v_right / math.cos(steer_right)

            # 3) 선형 속도 (m/s) → 각속도 (rad/s)
            w_fl = v_fl / WHEEL_RADIUS
            w_fr = v_fr / WHEEL_RADIUS
            w_rl = v_rl / WHEEL_RADIUS
            w_rr = v_rr / WHEEL_RADIUS
        else:
            # 직진 또는 거의 직진
            angular_vel = v / WHEEL_RADIUS
            w_fl = w_fr = w_rl = w_rr = angular_vel
            steer_left = 0.0
            steer_right = 0.0

        # 4. 조향 각도 명령 (Steering Controller)
        steer_msg = Float64MultiArray()
        # 주의: 컨트롤러의 joints 순서에 따라 [steering_joint_left, steering_joint_right] 순서로 전송
        steer_msg.data = [steer_left, steer_right] 
        self.pub_steer.publish(steer_msg)

        # 5. 바퀴 각속도 명령 (Wheel Controller)
        wheel_msg = Float64MultiArray()
        # 주의: 컨트롤러의 joints 순서에 맞게 [FL, FR, RL, RR] 순서로 전송해야 합니다.
        # 속도 제한
        wheel_msg.data = [
            max(min(w_fl, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED) * self.drive_sign,
            max(min(w_fr, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED) * self.drive_sign,
            max(min(w_rl, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED) * self.drive_sign,
            max(min(w_rr, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED) * self.drive_sign,
        ]
        self.pub_wheel.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToJointCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
