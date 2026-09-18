// Native Gazebo watchdog for simulation-only actuator topics.
//
// The ROS controllers deliberately publish to public /sim/* topics.  This
// system is the sole forwarder to the native Gazebo controllers, so that a
// dead ROS process or a dead ros_gz_bridge cannot leave the last actuator
// command running indefinitely.

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
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
    for (size_t i = 0; i < this->armJointNames.size(); ++i)
    {
      this->armPublisher[i] = this->node.Advertise<gz::msgs::Double>(
          this->armNative[i]);
    }
    this->node.Subscribe<gz::msgs::JointTrajectory>("/sim/arm/joint_positions",
        std::function<void(const gz::msgs::JointTrajectory &)>([this](
            const gz::msgs::JointTrajectory &_msg)
        {
          if (_msg.points_size() != 1 ||
              _msg.points(0).positions_size() != _msg.joint_names_size())
            return;
          std::array<double, 6> positions{};
          std::array<bool, 6> present{};
          for (int j = 0; j < _msg.joint_names_size(); ++j)
          {
            size_t i = 0;
            while (i < this->armJointNames.size() &&
                   this->armJointNames[i] != _msg.joint_names(j))
              ++i;
            const auto position = _msg.points(0).positions(j);
            if (i == this->armJointNames.size() || present[i] ||
                !std::isfinite(position))
              return;
            positions[i] = position;
            present[i] = true;
          }
          std::lock_guard<std::mutex> lock(this->mutex);
          for (size_t i = 0; i < present.size(); ++i)
          {
            if (!present[i])
              continue;
            this->armPending[i].set_data(positions[i]);
            ++this->armGeneration[i];
          }
        }));
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    // ROS publishers and transport loss are wall-clock events. Gazebo can run
    // faster or slower than real time, so simulation time is not a safe lease.
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    std::array<gz::msgs::Double, 3> wheels;
    uint64_t wheelGeneration[3];
    std::array<gz::msgs::Double, 6> arm;
    uint64_t armGeneration[6];
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      wheels = this->wheelPending;
      for (size_t i = 0; i < 3; ++i)
        wheelGeneration[i] = this->wheelGeneration[i];
      arm = this->armPending;
      for (size_t i = 0; i < 6; ++i)
        armGeneration[i] = this->armGeneration[i];
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
    for (size_t i = 0; i < 6; ++i)
    {
      if (armGeneration[i] != this->seenArmGeneration[i])
      {
        this->armPublisher[i].Publish(arm[i]);
        this->seenArmGeneration[i] = armGeneration[i];
        this->lastArm[i] = now;
        this->armSeen[i] = true;
        this->armHolding[i] = false;
      }
      else if (this->armSeen[i] && now - this->lastArm[i] > this->timeout)
      {
        const auto joint = this->model.JointByName(_ecm, this->armJointNames[i]);
        const auto *position = _ecm.Component<gz::sim::components::JointPosition>(joint);
        if (position && !position->Data().empty())
        {
          if (!this->armHolding[i])
          {
            this->armHold[i] = position->Data().front();
            this->armHolding[i] = true;
          }
          gz::msgs::Double hold;
          hold.set_data(this->armHold[i]);
          this->armPublisher[i].Publish(hold);
        }
      }
    }
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
  private: const std::array<std::string, 6> armJointNames{{
      "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
      "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"}};
  private: const std::array<std::string, 6> armNative{{
      "/sim/arm/arm_shoulder_pan/native_cmd_pos", "/sim/arm/arm_shoulder_lift/native_cmd_pos",
      "/sim/arm/arm_elbow_flex/native_cmd_pos", "/sim/arm/arm_wrist_flex/native_cmd_pos",
      "/sim/arm/arm_wrist_roll/native_cmd_pos", "/sim/arm/arm_gripper/native_cmd_pos"}};
  private: std::array<gz::transport::Node::Publisher, 3> wheelPublisher;
  private: std::array<gz::transport::Node::Publisher, 6> armPublisher;
  private: std::array<gz::msgs::Double, 3> wheelPending;
  private: std::array<gz::msgs::Double, 6> armPending;
  private: std::array<uint64_t, 3> wheelGeneration{};
  private: std::array<uint64_t, 3> seenWheelGeneration{};
  private: std::array<std::chrono::steady_clock::duration, 3> lastWheel{};
  private: std::array<bool, 3> wheelSeen{};
  private: std::array<uint64_t, 6> armGeneration{};
  private: std::array<uint64_t, 6> seenArmGeneration{};
  private: std::array<std::chrono::steady_clock::duration, 6> lastArm{};
  private: std::array<bool, 6> armSeen{};
  private: std::array<bool, 6> armHolding{};
  private: std::array<double, 6> armHold{};
};
}

GZ_ADD_PLUGIN(lekiwi_rmf::SimNativeFailsafe,
    gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(lekiwi_rmf::SimNativeFailsafe,
    "lekiwi_rmf::SimNativeFailsafe")
