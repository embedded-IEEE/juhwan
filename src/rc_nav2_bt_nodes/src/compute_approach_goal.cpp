#include "rc_nav2_bt_nodes/compute_approach_goal.hpp"

#include <cmath>

namespace
{
double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

namespace rc_nav2_bt_nodes
{

ComputeApproachGoal::ComputeApproachGoal(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ComputeApproachGoal::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<double>("approach_distance", 0.5, "Distance to keep straight before goal"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_goal")
  };
}

BT::NodeStatus ComputeApproachGoal::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }

  double approach_distance = 0.5;
  getInput("approach_distance", approach_distance);

  const double yaw = yawFromQuat(goal.pose.orientation);
  geometry_msgs::msg::PoseStamped approach_goal = goal;
  approach_goal.pose.position.x -= approach_distance * std::cos(yaw);
  approach_goal.pose.position.y -= approach_distance * std::sin(yaw);

  setOutput("approach_goal", approach_goal);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rc_nav2_bt_nodes
