#include "rc_nav2_bt_nodes/compute_approach_goals.hpp"

#include <cmath>
#include <utility>

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

ComputeApproachGoals::ComputeApproachGoals(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ComputeApproachGoals::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
    BT::InputPort<double>("approach_distance", 0.5, "Distance to keep straight before final goal"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("approach_goals")
  };
}

BT::NodeStatus ComputeApproachGoals::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (!getInput("goals", goals) || goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  double approach_distance = 0.5;
  getInput("approach_distance", approach_distance);

  const auto & final_goal = goals.back();
  const double yaw = yawFromQuat(final_goal.pose.orientation);
  geometry_msgs::msg::PoseStamped approach_goal = final_goal;
  approach_goal.pose.position.x -= approach_distance * std::cos(yaw);
  approach_goal.pose.position.y -= approach_distance * std::sin(yaw);

  std::vector<geometry_msgs::msg::PoseStamped> approach_goals;
  approach_goals.reserve(goals.size() + 1);
  if (goals.size() > 1) {
    approach_goals.insert(approach_goals.end(), goals.begin(), goals.end() - 1);
  }
  approach_goals.push_back(approach_goal);
  approach_goals.push_back(final_goal);

  setOutput("approach_goals", std::move(approach_goals));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rc_nav2_bt_nodes
