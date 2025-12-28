#ifndef RC_NAV2_BT_NODES__COMPUTE_APPROACH_GOAL_HPP_
#define RC_NAV2_BT_NODES__COMPUTE_APPROACH_GOAL_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rc_nav2_bt_nodes
{

class ComputeApproachGoal : public BT::SyncActionNode
{
public:
  ComputeApproachGoal(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace rc_nav2_bt_nodes

#endif  // RC_NAV2_BT_NODES__COMPUTE_APPROACH_GOAL_HPP_
