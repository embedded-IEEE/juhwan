import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry


class OdomStampRewriter(Node):
    def __init__(self):
        super().__init__('odom_stamp_rewriter')

        self.declare_parameter('min_step_ms', 1.0)
        min_step_ms = float(self.get_parameter('min_step_ms').value)
        self._min_step_ns = max(1, int(min_step_ms * 1e6))

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

        self.pub = self.create_publisher(Odometry, 'odom_out', pub_qos)
        self.sub = self.create_subscription(
            Odometry,
            'odom_in',
            self.callback,
            sub_qos)

        self._last_stamp = None

    def callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()

        if self._last_stamp is not None:
            min_next = self._last_stamp + Duration(nanoseconds=self._min_step_ns)
            if stamp <= min_next:
                # Ensure strictly increasing stamps for Cartographer.
                stamp = min_next

        self._last_stamp = stamp
        msg.header.stamp = stamp.to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomStampRewriter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
