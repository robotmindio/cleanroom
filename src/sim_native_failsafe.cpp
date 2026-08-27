// Native Gazebo watchdog for simulation-only actuator topics.
//
// The ROS controllers deliberately publish to public /sim/* topics.  This
// system is the sole forwarder to the native Gazebo controllers, so that a
// dead ROS process or a dead ros_gz_bridge cannot leave the last velocity or
// an autonomous JointTrajectoryController trajectory running indefinitely.

#include <array>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/joint_trajectory.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/transport/Node.hh>

namespace lekiwi_rmf
{
class SimNativeFailsafe final : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    // JointPosition is demand-driven in Gazebo's ECS. Request it explicitly:
    // the watchdog must still have measured positions when the ROS joint-state
    // bridge (or the adapter which normally consumes it) has gone away.
    for (const auto &name : this->armJointNames)
    {
      const auto joint = this->model.JointByName(_ecm, name);
      if (joint != gz::sim::kNullEntity &&
          !_ecm.Component<gz::sim::components::JointPosition>(joint))
      {
        _ecm.CreateComponent(joint, gz::sim::components::JointPosition());
      }
    }
    for (size_t i = 0; i < this->wheelInput.size(); ++i)
    {
      this->wheelPublisher[i] = this->node.Advertise<gz::msgs::Double>(
          this->wheelNative[i]);
      this->node.Subscribe<gz::msgs::Double>(this->wheelInput[i],
          std::function<void(const gz::msgs::Double &)>([this, i](const gz::msgs::Double &_msg)
          {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->wheelPending[i] = _msg;
            ++this->wheelGeneration[i];
          }));
    }
    this->armPublisher = this->node.Advertise<gz::msgs::JointTrajectory>(
        "/sim/arm/native_joint_trajectory");
    this->node.Subscribe<gz::msgs::JointTrajectory>("/sim/arm/joint_trajectory",
        std::function<void(const gz::msgs::JointTrajectory &)>([this](const gz::msgs::JointTrajectory &_msg)
        {
          std::lock_guard<std::mutex> lock(this->mutex);
          this->armPending = _msg;
          ++this->armGeneration;
        }));
    this->node.Subscribe<gz::msgs::Boolean>("/sim/arm/trajectory_heartbeat",
        std::function<void(const gz::msgs::Boolean &)>([this](const gz::msgs::Boolean &_msg)
        {
          if (_msg.data())
          {
            std::lock_guard<std::mutex> lock(this->mutex);
            ++this->heartbeatGeneration;
          }
        }));
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    const auto now = _info.simTime;
    std::array<gz::msgs::Double, 3> wheels;
    gz::msgs::JointTrajectory arm;
    uint64_t wheelGeneration[3];
    uint64_t armGeneration;
    uint64_t heartbeatGeneration;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      wheels = this->wheelPending;
      arm = this->armPending;
      for (size_t i = 0; i < 3; ++i)
        wheelGeneration[i] = this->wheelGeneration[i];
      armGeneration = this->armGeneration;
      heartbeatGeneration = this->heartbeatGeneration;
    }
    for (size_t i = 0; i < 3; ++i)
    {
      if (wheelGeneration[i] != this->seenWheelGeneration[i])
      {
        this->wheelPublisher[i].Publish(wheels[i]);
        this->seenWheelGeneration[i] = wheelGeneration[i];
        this->lastWheel[i] = now;
        this->wheelSeen[i] = true;
      }
      else if (this->wheelSeen[i] && now - this->lastWheel[i] > this->timeout)
      {
        gz::msgs::Double stop;
        stop.set_data(0.0);
        // Continue sending zero rather than relying on a single best-effort
        // transport packet during bridge/controller teardown.
        this->wheelPublisher[i].Publish(stop);
      }
    }
    if (armGeneration != this->seenArmGeneration)
    {
      this->armPublisher.Publish(arm);
      this->seenArmGeneration = armGeneration;
      this->armActive = true;
      this->lastArmHeartbeat = now;
    }
    if (heartbeatGeneration != this->seenHeartbeatGeneration)
    {
      this->seenHeartbeatGeneration = heartbeatGeneration;
      this->lastArmHeartbeat = now;
    }
    if (this->armActive && now - this->lastArmHeartbeat > this->timeout)
    {
      auto hold = this->HoldTrajectory(_ecm);
      if (hold.joint_names_size() == static_cast<int>(this->armJointNames.size()))
      {
        this->armPublisher.Publish(hold);
        this->armActive = false;
      }
    }
  }

  private: gz::msgs::JointTrajectory HoldTrajectory(
      gz::sim::EntityComponentManager &_ecm) const
  {
    gz::msgs::JointTrajectory hold;
    auto *point = hold.add_points();
    point->mutable_time_from_start()->set_nsec(100000000);
    for (const auto &name : this->armJointNames)
    {
      const auto joint = this->model.JointByName(_ecm, name);
      const auto *position = _ecm.Component<gz::sim::components::JointPosition>(joint);
      if (!position || position->Data().empty())
      {
        hold.clear_joint_names();
        hold.clear_points();
        return hold;
      }
      hold.add_joint_names(name);
      point->add_positions(position->Data().front());
    }
    return hold;
  }

  private: const std::chrono::steady_clock::duration timeout{
      std::chrono::milliseconds(250)};
  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::transport::Node node;
  private: std::mutex mutex;
  private: const std::array<std::string, 3> wheelInput{{
      "/sim/sim_base_left_wheel/cmd_vel", "/sim/sim_base_back_wheel/cmd_vel",
      "/sim/sim_base_right_wheel/cmd_vel"}};
  private: const std::array<std::string, 3> wheelNative{{
      "/sim/sim_base_left_wheel/native_cmd_vel", "/sim/sim_base_back_wheel/native_cmd_vel",
      "/sim/sim_base_right_wheel/native_cmd_vel"}};
  private: const std::vector<std::string> armJointNames{
      "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
      "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"};
  private: std::array<gz::transport::Node::Publisher, 3> wheelPublisher;
  private: gz::transport::Node::Publisher armPublisher;
  private: std::array<gz::msgs::Double, 3> wheelPending;
  private: gz::msgs::JointTrajectory armPending;
  private: std::array<uint64_t, 3> wheelGeneration{};
  private: std::array<uint64_t, 3> seenWheelGeneration{};
  private: std::array<std::chrono::steady_clock::duration, 3> lastWheel{};
  private: std::array<bool, 3> wheelSeen{};
  private: uint64_t armGeneration{};
  private: uint64_t seenArmGeneration{};
  private: uint64_t heartbeatGeneration{};
  private: uint64_t seenHeartbeatGeneration{};
  private: std::chrono::steady_clock::duration lastArmHeartbeat{};
  private: bool armActive{false};
};
}

GZ_ADD_PLUGIN(lekiwi_rmf::SimNativeFailsafe,
    gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(lekiwi_rmf::SimNativeFailsafe,
    "lekiwi_rmf::SimNativeFailsafe")
