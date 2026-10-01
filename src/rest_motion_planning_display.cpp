#include <moveit/motion_planning_rviz_plugin/motion_planning_display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace lekiwi_rmf
{
class RestMotionPlanningDisplay : public moveit_rviz_plugin::MotionPlanningDisplay
{
protected:
  void onInitialize() override
  {
    // RViz does not declare parameter overrides automatically. MoveIt's loader
    // checks has_parameter(), so an undeclared override silently leaves FCL active.
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    if (!node->has_parameter("collision_detector"))
      node->declare_parameter<std::string>("collision_detector", "lekiwi_rmf/RestFCL");
    MotionPlanningDisplay::onInitialize();
  }
};
}  // namespace lekiwi_rmf
PLUGINLIB_EXPORT_CLASS(lekiwi_rmf::RestMotionPlanningDisplay, rviz_common::Display)
