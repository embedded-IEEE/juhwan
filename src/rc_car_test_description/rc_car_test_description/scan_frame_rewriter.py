import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ScanFrameRewriter(Node):
    def __init__(self):
        super().__init__('scan_frame_rewriter')

        self.declare_parameter('target_frame', 'lidar_link_1')
        self.declare_parameter('scan_rate', 0.0)  # Hz, 0.0이면 msg.scan_time 기반
        target_frame = self.get_parameter('target_frame').value
        self.scan_rate = float(self.get_parameter('scan_rate').value)

        # 입력(/scan_raw)은 실제 LiDAR QoS(BEST_EFFORT)에 맞추고,
        # 출력(/scan)은 Nav2가 선호하는 RELIABLE로 발행한다.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(LaserScan, '/rc_car/scan', pub_qos)
        self.sub = self.create_subscription(
            LaserScan,
            '/rc_car/scan_raw',   # 기존 Gazebo → ROS 브리지 출력
            self.callback,
            sub_qos)

        self.target_frame = target_frame
        self._last_stamp = None

    def callback(self, msg):
        # frame id 보정 + 시뮬레이션 시간으로 stamp 갱신(중복 시간으로 인한 Cartographer 무시 방지)
        msg.header.frame_id = self.target_frame
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()

        if self._last_stamp is not None:
            min_next = self._last_stamp + Duration(nanoseconds=1)
            if stamp <= min_next:
                # stamp가 역행/중복이면 최소 1ns만 앞으로 보정
                stamp = min_next

        self._last_stamp = stamp
        msg.header.stamp = stamp.to_msg()

        # scan_time/time_increment를 명시적으로 재설정해 중복 subdivision 경고 방지
        if self.scan_rate > 0.0:
            msg.scan_time = 1.0 / self.scan_rate
            if len(msg.ranges) > 0:
                msg.time_increment = msg.scan_time / max(1, len(msg.ranges))

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRewriter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
