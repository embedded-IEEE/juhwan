#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class RcCarPoseBridge(Node):
    """Bridge Gazebo rc_car pose to TF: empty_world -> rc_car/odom."""

    def __init__(self):
        super().__init__('rc_car_pose_tf_bridge')
        # 사용자가 use_sim_time을 넘기면 받아서 시뮬레이션 시간에 맞춤
        self.declare_parameter('use_sim_time', True)
        self.frame_world = 'empty_world'
        self.odom_frame = 'rc_car/odom'
        self.sub_pose = self.create_subscription(Pose, '/rc_car/pose', self.pose_cb, 10)
        self.pub = self.create_publisher(TFMessage, '/tf', 10)

    def pose_cb(self, msg: Pose):
        tf_msg = TFMessage()
        tf_odom = TransformStamped()
        tf_odom.header.stamp = self.get_clock().now().to_msg()
        tf_odom.header.frame_id = self.frame_world
        tf_odom.child_frame_id = self.odom_frame
        tf_odom.transform.translation.x = msg.position.x
        tf_odom.transform.translation.y = msg.position.y
        tf_odom.transform.translation.z = msg.position.z
        tf_odom.transform.rotation = msg.orientation
        tf_msg.transforms.append(tf_odom)

        tf_base = TransformStamped()
        tf_base.header.stamp = tf_odom.header.stamp
        tf_base.header.frame_id = self.odom_frame
        tf_base.child_frame_id = 'rc_car/base_link_rot'
        tf_base.transform.rotation.w = 1.0
        tf_msg.transforms.append(tf_base)

        self.pub.publish(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RcCarPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
