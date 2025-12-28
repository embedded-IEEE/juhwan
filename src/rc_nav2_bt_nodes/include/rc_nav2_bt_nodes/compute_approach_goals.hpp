#ifndef RC_NAV2_BT_NODES__COMPUTE_APPROACH_GOALS_HPP_
#define RC_NAV2_BT_NODES__COMPUTE_APPROACH_GOALS_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rc_nav2_bt_nodes
{

class ComputeApproachGoals : public BT::SyncActionNode
{
public:
  ComputeApproachGoals(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace rc_nav2_bt_nodes

#endif  // RC_NAV2_BT_NODES__COMPUTE_APPROACH_GOALS_HPP_
