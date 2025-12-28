#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    """Simple bridge that republishes /cmd_vel to a target topic (e.g., /rc_car/cmd_vel)."""

    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('target_topic', '/rc_car/cmd_vel')
        target = self.get_parameter('target_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(Twist, target, 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.get_logger().info(f"Bridging /cmd_vel -> {target}")

    def cb(self, msg: Twist):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
