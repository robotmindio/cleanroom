#include "../src/rest_collision_env.hpp"
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <iostream>

int main(int argc, char** argv)
{
  if (argc != 3) return 2;
  std::ifstream input(argv[1]);
  std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  auto urdf = urdf::parseURDF(xml);
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initFile(*urdf, argv[2])) return 3;
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  lekiwi_rmf::RestCollisionEnv detector(model);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  if (!state.setToDefaultValues(model->getJointModelGroup("arm"), "travel_stow")) return 4;
  state.update();
  collision_detection::AllowedCollisionMatrix only_rest(model->getLinkModelNames(), true);
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  detector.checkSelfCollision(request, result, state, only_rest);
  if (result.collision) throw std::runtime_error("folded resting contact was rejected");
  // Roll changes no forearm/shoulder geometry: their contact still exists,
  // but this pose has left the resting neighborhood and must be rejected.
  state.setVariablePosition("arm_wrist_roll", state.getVariablePosition("arm_wrist_roll") + 0.15);
  state.update();
  result.clear();
  detector.checkSelfCollision(request, result, state, only_rest);
  if (!result.collision) throw std::runtime_error("outside-fold contact was silently allowed");
  state.setToDefaultValues(model->getJointModelGroup("arm"), "travel_stow");
  const double lift = state.getVariablePosition("arm_shoulder_lift");
  const double elbow = state.getVariablePosition("arm_elbow_flex");
  const double wrist = state.getVariablePosition("arm_wrist_flex");
  for (int i = 1; i <= 20; ++i)
  {
    const double t = i / 20.0;
    state.setVariablePosition("arm_shoulder_lift", lift + t * (-1.73 - lift));
    state.setVariablePosition("arm_elbow_flex", elbow + t * (1.52 - elbow));
    state.setVariablePosition("arm_wrist_flex", wrist + t * (1.12 - wrist));
    state.update();
    result.clear();
    detector.checkSelfCollision(request, result, state, only_rest);
    if (result.collision) throw std::runtime_error("measured short release path was blocked");
  }
  std::cout << "fold accepted; outside-fold contact rejected\n";
}
