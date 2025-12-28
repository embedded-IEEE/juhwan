#include "rc_nav2_bt_nodes/near_goal_stopped_condition.hpp"

#include <cmath>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "rc_nav2_bt_nodes/compute_approach_goal.hpp"
#include "rc_nav2_bt_nodes/compute_approach_goals.hpp"

namespace rc_nav2_bt_nodes
{

NearGoalStoppedCondition::NearGoalStoppedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  tf_buffer_(rclcpp::Clock::make_shared()),
  tf_listener_(tf_buffer_),
  has_goal_(false),
  last_speed_(0.0)
{
  node_ = rclcpp::Node::make_shared("near_goal_stopped_condition");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opts = rclcpp::SubscriptionOptions();
  sub_opts.callback_group = callback_group_;

  std::string goal_topic;
  getInput("goal_topic", goal_topic);
  std::string cmd_vel_topic;
  getInput("cmd_vel_topic", cmd_vel_topic);

  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, rclcpp::QoS(10),
    std::bind(&NearGoalStoppedCondition::goalCallback, this, std::placeholders::_1),
    sub_opts);

  cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(10),
    std::bind(&NearGoalStoppedCondition::cmdVelCallback, this, std::placeholders::_1),
    sub_opts);

  last_motion_time_ = node_->get_clock()->now();

  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  executor_thread_ = std::thread([this]() {executor_.spin();});
}

NearGoalStoppedCondition::~NearGoalStoppedCondition()
{
  executor_.cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

void NearGoalStoppedCondition::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_goal_ = *msg;
  has_goal_.store(true);
}

void NearGoalStoppedCondition::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double linear = std::hypot(msg->linear.x, msg->linear.y);
  const double angular = std::abs(msg->angular.z);
  const double speed = std::max(linear, angular);
  last_speed_.store(speed);

  double vel_threshold = 0.01;
  getInput("vel_threshold", vel_threshold);

  if (speed > vel_threshold) {
    last_motion_time_ = node_->get_clock()->now();
  }
}

BT::NodeStatus NearGoalStoppedCondition::tick()
{
  if (!has_goal_.load()) {
    return BT::NodeStatus::FAILURE;
  }

  std::string global_frame;
  std::string base_frame;
  double dist_threshold = 0.3;
  double vel_threshold = 0.1;
  double hold_time = 1.0;

  getInput("global_frame", global_frame);
  getInput("base_frame", base_frame);
  getInput("dist_threshold", dist_threshold);
  getInput("vel_threshold", vel_threshold);
  getInput("hold_time", hold_time);

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(global_frame, base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return BT::NodeStatus::FAILURE;
  }

  const double dx = last_goal_.pose.position.x - tf.transform.translation.x;
  const double dy = last_goal_.pose.position.y - tf.transform.translation.y;
  const double dist = std::hypot(dx, dy);

  const auto now = node_->get_clock()->now();
  const double stopped_for =
    (now - last_motion_time_).seconds();

  if (dist <= dist_threshold &&
    last_speed_.load() <= vel_threshold &&
    stopped_for >= hold_time)
  {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rc_nav2_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rc_nav2_bt_nodes::NearGoalStoppedCondition>("NearGoalStopped");
  factory.registerNodeType<rc_nav2_bt_nodes::ComputeApproachGoal>("ComputeApproachGoal");
  factory.registerNodeType<rc_nav2_bt_nodes::ComputeApproachGoals>("ComputeApproachGoals");
}
