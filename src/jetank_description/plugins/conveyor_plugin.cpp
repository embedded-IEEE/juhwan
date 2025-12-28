#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>

#include <ignition/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>
#include <string>
#include <cmath>

using namespace ignition;
using namespace ignition::gazebo;

namespace jetank
{

class ConveyorPlugin
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(
    const Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    EntityComponentManager &ecm,
    EventManager &) override
  {
    this->model_ = entity;
    Model model(entity);
    if (!model.Valid(ecm))
    {
      ignerr << "[ConveyorPlugin] Invalid model entity\n";
      return;
    }

    if (sdf && sdf->HasElement("joint_name"))
      this->joint_name_ = sdf->Get<std::string>("joint_name");

    if (sdf && sdf->HasElement("speed"))
      this->belt_speed_ = sdf->Get<double>("speed");

    if (sdf && sdf->HasElement("reset_period"))
      this->reset_period_ = sdf->Get<double>("reset_period");

    this->joint_ = model.JointByName(ecm, this->joint_name_);
    if (this->joint_ == kNullEntity)
    {
      ignerr << "[ConveyorPlugin] Joint not found: " << this->joint_name_ << "\n";
      return;
    }

    if (!ecm.Component<components::JointVelocityCmd>(this->joint_))
      ecm.CreateComponent(
        this->joint_,
        components::JointVelocityCmd({0.0}));

    // ---------------- ROS2 ----------------
    this->ros_context_ = std::make_shared<rclcpp::Context>();
    this->ros_context_->init(0, nullptr);

    rclcpp::NodeOptions opts;
    opts.context(this->ros_context_);

    this->ros_node_ =
      std::make_shared<rclcpp::Node>("conveyor_plugin", opts);

    this->power_srv_ =
      this->ros_node_->create_service<std_srvs::srv::SetBool>(
        "conveyor/power",
        std::bind(&ConveyorPlugin::onSetPower, this,
                  std::placeholders::_1, std::placeholders::_2));

    this->speed_srv_ =
      this->ros_node_->create_service<std_srvs::srv::SetBool>(
        "conveyor/set_speed_fast",
        std::bind(&ConveyorPlugin::onSetSpeedFast, this,
                  std::placeholders::_1, std::placeholders::_2));

    ignmsg << "[ConveyorPlugin] Loaded (ON/OFF + continuous reset)\n";
  }

  void PreUpdate(
    const UpdateInfo &info,
    EntityComponentManager &ecm) override
  {
    if (this->ros_node_)
      rclcpp::spin_some(this->ros_node_);

    if (info.paused || this->joint_ == kNullEntity)
      return;

    const double sim_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(info.simTime).count();

    auto *vel =
      ecm.Component<components::JointVelocityCmd>(this->joint_);

    if (!this->running_)
    {
      vel->Data() = {0.0};
      this->doReset(ecm);
      this->last_reset_time_ = sim_time;
      return;
    }

    // ON 상태 → 계속 회전
    vel->Data() = { this->belt_speed_ };

    // ON 상태에서도 주기적 reset
    if ((sim_time - this->last_reset_time_) >= this->reset_period_)
    {
      this->doReset(ecm);
      this->last_reset_time_ = sim_time;
    }
  }

private:
  void doReset(EntityComponentManager &ecm)
  {
    auto *reset =
      ecm.Component<components::JointPositionReset>(this->joint_);
    if (!reset)
      ecm.CreateComponent(
        this->joint_,
        components::JointPositionReset({0.0}));
    else
      reset->Data() = {0.0};
  }

  void onSetPower(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    this->running_ = req->data;
    res->success = true;
    res->message = this->running_ ? "Conveyor ON" : "Conveyor OFF";
  }

  // 예시: true = 빠름 / false = 느림
  void onSetSpeedFast(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    this->belt_speed_ = req->data ? 1.0 : 0.3;
    res->success = true;
    res->message = "Speed updated";
  }

private:
  Entity model_{kNullEntity};
  Entity joint_{kNullEntity};

  std::string joint_name_{"belt_joint"};

  bool running_{false};
  double belt_speed_{0.5};

  double reset_period_{0.1};     // 토글 주기
  double last_reset_time_{0.0};

  std::shared_ptr<rclcpp::Context> ros_context_;
  std::shared_ptr<rclcpp::Node> ros_node_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr speed_srv_;
};

} // namespace jetank

// ---------------- Plugin registration ----------------

IGNITION_ADD_PLUGIN(
  jetank::ConveyorPlugin,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  jetank::ConveyorPlugin,
  "jetank::ConveyorPlugin")
