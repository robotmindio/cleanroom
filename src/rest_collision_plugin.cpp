#include "rest_collision_env.hpp"
#include <moveit/collision_detection/collision_plugin.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace lekiwi_rmf
{
class RestCollisionPlugin : public collision_detection::CollisionPlugin
{
public:
  bool initialize(const planning_scene::PlanningScenePtr& scene) const override
  {
    scene->allocateCollisionDetector(RestCollisionAllocator::create());
    return true;
  }
};
}  // namespace lekiwi_rmf
PLUGINLIB_EXPORT_CLASS(lekiwi_rmf::RestCollisionPlugin, collision_detection::CollisionPlugin)
