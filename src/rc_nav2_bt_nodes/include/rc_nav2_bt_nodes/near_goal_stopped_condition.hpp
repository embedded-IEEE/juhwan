#ifndef RC_NAV2_BT_NODES__NEAR_GOAL_STOPPED_CONDITION_HPP_
#define RC_NAV2_BT_NODES__NEAR_GOAL_STOPPED_CONDITION_HPP_

#include <atomic>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rc_nav2_bt_nodes
{

class NearGoalStoppedCondition : public BT::ConditionNode
{
public:
  NearGoalStoppedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~NearGoalStoppedCondition() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("dist_threshold", 0.01, "Distance threshold to goal (m)"),
      BT::InputPort<double>("vel_threshold", 0.01, "Velocity threshold (m/s or rad/s)"),
      BT::InputPort<double>("hold_time", 1.0, "Time at near-zero velocity (s)"),
      BT::InputPort<std::string>("global_frame", "map", "Global frame"),
      BT::InputPort<std::string>("base_frame", "rc_car/rear_axle", "Robot base frame"),
      BT::InputPort<std::string>("goal_topic", "/goal_pose", "Goal pose topic"),
      BT::InputPort<std::string>("cmd_vel_topic", "/rc_car/cmd_vel", "cmd_vel topic")
    };
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::atomic<bool> has_goal_;
  geometry_msgs::msg::PoseStamped last_goal_;
  rclcpp::Time last_motion_time_;
  std::atomic<double> last_speed_;
};

}  // namespace rc_nav2_bt_nodes

#endif  // RC_NAV2_BT_NODES__NEAR_GOAL_STOPPED_CONDITION_HPP_
