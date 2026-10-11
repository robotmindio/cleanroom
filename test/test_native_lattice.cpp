#include <array>
#include <cmath>
#include <iostream>
#include <nav2_smac_planner/node_lattice.hpp>
#include <nav2_smac_planner/collision_checker.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lattice_collision_check");
  nav2_costmap_2d::Costmap2D map(80, 80, 0.05, -2.0, -2.0, 0);
  nav2_smac_planner::GridCollisionChecker checker(nullptr, 72, node);
  checker.setCostmap(&map);
  const std::array<std::array<double, 2>, 4> corners = {{
    {0.24, 0.22}, {0.24, -0.22}, {-0.22, -0.22}, {-0.22, 0.22}}};
  nav2_costmap_2d::Footprint footprint(4);
  for (size_t i = 0; i < corners.size(); ++i) {
    footprint[i].x = corners[i][0];
    footprint[i].y = corners[i][1];
  }
  checker.setFootprint(footprint, false, 1.0);
  map.setCost(34, 30, nav2_costmap_2d::NO_INFORMATION);
  nav2_smac_planner::NodeLattice::motion_table.lattice_metadata.heading_angles = {
    0.0f, float(M_PI_2), float(M_PI), float(3 * M_PI_2)};
  nav2_smac_planner::NodeLattice state(0);
  state.setPose({39.4f, 31.4f, 3.0f});
  const bool clear = state.isNodeValid(false, &checker);
  // The same grid index now overlaps the unknown cell by only 1 cm.
  state.setPose({39.2f, 31.4f, 3.0f});
  const bool blocked = !state.isNodeValid(false, &checker);
  const bool allowed_unknown = state.isNodeValid(true, &checker);
  // Clear endpoints cannot excuse an overlapping intermediate primitive pose.
  nav2_smac_planner::MotionPrimitive primitive;
  primitive.start_angle = primitive.end_angle = 3;
  const auto direction = nav2_smac_planner::TurnDirection::UNKNOWN;
  primitive.poses = {
    {0.0f, 0.0f, float(3 * M_PI_2), direction},
    {-0.01f, 0.0f, float(3 * M_PI_2), direction},
    {0.0f, 0.0f, float(3 * M_PI_2), direction}};
  nav2_smac_planner::NodeLattice::motion_table.lattice_metadata.grid_resolution = 0.05f;
  state.reset();
  state.setPose({39.4f, 31.4f, 3.0f});
  const bool middle_blocked = !state.isNodeValid(false, &checker, &primitive, false);
  // A fractional start must not be checked at the shifted cell centre.
  map.setCost(34, 30, nav2_costmap_2d::FREE_SPACE);
  map.setCost(45, 40, nav2_costmap_2d::NO_INFORMATION);
  map.setCost(40, 40, 100);  // A soft inflation cost keeps the old outline check active.
  state.reset();
  state.setPose({40.01f, 40.01f, 0.0f});
  const bool fractional_clear = state.isNodeValid(false, &checker);
  const bool outside_blocked = checker.inCollision(80.0f, 40.0f, 0.0f, false);
  const bool passed = clear && blocked && allowed_unknown && middle_blocked && fractional_clear && outside_blocked;
  if (!passed) {
    std::cerr << "clear=" << clear << " blocked=" << blocked
              << " allowed_unknown=" << allowed_unknown
              << " middle_blocked=" << middle_blocked
              << " fractional_clear=" << fractional_clear
              << " outside_blocked=" << outside_blocked << '\n';
  }
  node.reset();
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
