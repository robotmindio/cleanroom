#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <nav2_smac_planner/node_lattice.hpp>
#include <nav2_smac_planner/collision_checker.hpp>
#include <nav2_smac_planner/smac_planner_lattice.hpp>

bool subcell_goals(const std::string & lattice)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("subcell_goal_test");
  node->declare_parameter("test.lattice_filepath", lattice);
  node->declare_parameter("test.allow_unknown", false);
  node->declare_parameter("test.smooth_path", false);
  node->declare_parameter("test.tolerance", 0.0);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("subcell_costmap");
  costmap->set_parameter(rclcpp::Parameter("footprint",
    "[[0.24,0.22],[0.24,-0.22],[-0.22,-0.22],[-0.22,0.22]]"));
  costmap->on_configure(rclcpp_lifecycle::State());
  costmap->getCostmap()->resizeMap(40, 40, 0.05, -1.0, -1.0);
  auto * grid = costmap->getCostmap();
  for (unsigned int y = 0; y < 40; ++y) {
    for (unsigned int x = 0; x < 40; ++x) {grid->setCost(x, y, 0);}
  }
  nav2_smac_planner::SmacPlannerLattice planner;
  planner.configure(node, "test", nullptr, costmap);
  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = start.pose.position.y = 0.001;
  start.pose.orientation.w = 1.0;
  goal = start;
  goal.pose.position.x = goal.pose.position.y = 0.049;
  auto canceled = []() {return false;};
  const auto plan = planner.createPlan(start, goal, canceled);
  const bool exact = plan.poses.size() >= 2 &&
    std::abs(plan.poses.back().pose.position.x - goal.pose.position.x) < 1e-8 &&
    std::abs(plan.poses.back().pose.position.y - goal.pose.position.y) < 1e-8;
  grid->setCost(25, 25, nav2_costmap_2d::LETHAL_OBSTACLE);
  bool endpoint_blocked = false, rotation_blocked = false;
  try {planner.createPlan(start, goal, canceled);} catch (const nav2_core::PlannerException &) {
    endpoint_blocked = true;
  }
  grid->setCost(25, 25, 0);
  grid->setCost(26, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  goal = start;
  goal.pose.orientation.z = goal.pose.orientation.w = std::sqrt(0.5);
  try {planner.createPlan(start, goal, canceled);} catch (const nav2_core::PlannerException &) {
    rotation_blocked = true;
  }
  grid->setCost(26, 20, 0);
  const auto rotation = planner.createPlan(start, goal, canceled);
  const bool rotation_exact = !rotation.poses.empty() &&
    std::abs(rotation.poses.back().pose.orientation.z - goal.pose.orientation.z) < 1e-8;
  bool cancellation_honored = false;
  try {planner.createPlan(start, goal, []() {return true;});}
  catch (const nav2_core::PlannerCancelled &) {cancellation_honored = true;}
  goal = start;
  goal.pose.position.x = 0.123;
  goal.pose.position.y = 0.073;
  goal.pose.orientation.z = std::sin(0.137 / 2.0);
  goal.pose.orientation.w = std::cos(0.137 / 2.0);
  const auto general = planner.createPlan(start, goal, canceled);
  const auto & endpoint = general.poses.back().pose;
  const bool general_exact =
    std::abs(endpoint.position.x - goal.pose.position.x) < 1e-8 &&
    std::abs(endpoint.position.y - goal.pose.position.y) < 1e-8 &&
    std::abs(endpoint.orientation.z - goal.pose.orientation.z) < 1e-8 &&
    std::abs(endpoint.orientation.w - goal.pose.orientation.w) < 1e-8;
  bool short_direct = true;
  start.pose.position.x = 0.044;
  start.pose.position.y = -0.394;
  const double heading = -0.5;
  start.pose.orientation.z = std::sin(heading / 2.0);
  start.pose.orientation.w = std::cos(heading / 2.0);
  // Replanning across a cell edge must not turn centimetres into a lattice detour.
  for (const auto & delta : std::array<std::array<double, 3>, 7>{{
      {.05, 0, 0}, {.08, 0, 0}, {.10, 0, 0}, {-.05, 0, 0}, {0, .05, 0}, {0, -.05, 0}, {.05, 0, .137}}})
  {
    goal = start;
    goal.pose.position.x += delta[0] * std::cos(heading) - delta[1] * std::sin(heading);
    goal.pose.position.y += delta[0] * std::sin(heading) + delta[1] * std::cos(heading);
    goal.pose.orientation.z = std::sin((heading + delta[2]) / 2.0);
    goal.pose.orientation.w = std::cos((heading + delta[2]) / 2.0);
    const auto short_plan = planner.createPlan(start, goal, canceled);
    double length = 0.0;
    for (size_t i = 1; i < short_plan.poses.size(); ++i) {
      length += std::hypot(
        short_plan.poses[i].pose.position.x - short_plan.poses[i - 1].pose.position.x,
        short_plan.poses[i].pose.position.y - short_plan.poses[i - 1].pose.position.y);
    }
    short_direct &= short_plan.poses.size() >= 2 &&
      std::abs(length - std::hypot(delta[0], delta[1])) < 1e-6 &&
      std::abs(short_plan.poses.front().pose.position.x - start.pose.position.x) < 1e-8 &&
      std::abs(short_plan.poses.back().pose.position.x - goal.pose.position.x) < 1e-8;
  }
  bool short_cancellation = false;
  int polls = 0;
  try {planner.createPlan(start, goal, [&polls]() {return ++polls > 3;});}
  catch (const nav2_core::PlannerCancelled &) {short_cancellation = true;}
  start.pose.position.x = start.pose.position.y = 0.001;
  start.pose.orientation.z = 0.0;
  start.pose.orientation.w = 1.0;
  goal = start;
  goal.pose.position.x = 0.061;
  goal.pose.orientation.z = goal.pose.orientation.w = std::sqrt(0.5);
  grid->setCost(26, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  bool blocked_short_fallback = false;
  try {
    const auto detour = planner.createPlan(start, goal, canceled);
    blocked_short_fallback = !detour.poses.empty();
  } catch (const nav2_core::NoValidPathCouldBeFound &) {
    blocked_short_fallback = false;
  }
  planner.cleanup();
  costmap->on_cleanup(rclcpp_lifecycle::State());
  std::cout << "exact_subcell_goal=" << exact << " endpoint_blocked=" << endpoint_blocked
            << " swept_rotation_blocked=" << rotation_blocked << " rotation_exact=" << rotation_exact
            << " cancellation_honored=" << cancellation_honored
            << " general_exact=" << general_exact << " short_direct=" << short_direct
            << " short_cancellation=" << short_cancellation
            << " blocked_short_fallback=" << blocked_short_fallback << '\n';
  return exact && endpoint_blocked && rotation_blocked && rotation_exact && cancellation_honored &&
         general_exact && short_direct && short_cancellation && blocked_short_fallback;
}

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
  const bool passed = clear && blocked && allowed_unknown && middle_blocked && fractional_clear && outside_blocked &&
    argc == 2 && subcell_goals(argv[1]);
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
