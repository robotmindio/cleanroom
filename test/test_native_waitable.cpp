#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waitable_destruction_check");
  auto group = node->get_node_base_interface()->get_default_callback_group();
  const auto before = group->size();
  auto client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node, "unused");
  auto live_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node, "live");
  client.reset();
  const bool kept_live = group->size() == before + 1;
  live_client.reset();
  const bool removed = kept_live && group->size() == before;
  node.reset();
  group.reset();
  rclcpp::shutdown();
  return removed ? 0 : 1;
}
