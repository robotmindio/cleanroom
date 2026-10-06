#include "../src/rest_collision_env.hpp"
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <iostream>
#include <array>

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
  // Roll cannot change forearm/shoulder contact. Only pairs containing the
  // rolling part leave their permitted relative orientation.
  state.setVariablePosition("arm_wrist_roll", state.getVariablePosition("arm_wrist_roll") + 0.15);
  state.update();
  auto guarded = detector.guardedMatrix(state, only_rest);
  collision_detection::AllowedCollision::Type type;
  if (!guarded.getEntry("shoulder_motor_collision_proxy", "forearm_link_collision_proxy", type) ||
      type != collision_detection::AllowedCollision::ALWAYS)
    throw std::runtime_error("unrelated wrist roll revoked forearm resting contact");
  if (!guarded.getEntry("shoulder_holder_collision_proxy", "roll_holder_collision_proxy", type) ||
      type != collision_detection::AllowedCollision::NEVER)
    throw std::runtime_error("excessive relative roll retained resting permission");
  // Both observed loaded poses are physically valid. Joint-space gating
  // rejected them despite the parts remaining near their resting placement.
  for (const auto& pose : {std::array<double, 3>{-1.672447371141819, 1.4944621463230567, 1.2336217306403876},
                          std::array<double, 3>{-1.6709130157554506, 1.5466302294595906, 1.2336217306403876}})
  {
    state.setToDefaultValues(model->getJointModelGroup("arm"), "travel_stow");
    state.setVariablePosition("arm_shoulder_lift", pose[0]);
    state.setVariablePosition("arm_elbow_flex", pose[1]);
    state.setVariablePosition("arm_wrist_flex", pose[2]);
    state.setVariablePosition("arm_wrist_roll", -0.013809198477317652);
    state.setVariablePosition("arm_gripper", 0.21214683802333567);
    state.update();
    result.clear();
    detector.checkSelfCollision(request, result, state, only_rest);
    if (result.collision) throw std::runtime_error("confirmed loaded release pose was rejected");
    result.clear();
    detector.checkSelfCollision(request, result, state,
        collision_detection::AllowedCollisionMatrix(*srdf));
    if (result.collision) throw std::runtime_error("loaded pose failed complete self-collision check");
  }
  state.setToDefaultValues(model->getJointModelGroup("arm"), "travel_stow");
  state.setVariablePosition("arm_shoulder_lift", state.getVariablePosition("arm_shoulder_lift") + 0.4);
  state.update();
  guarded = detector.guardedMatrix(state, only_rest);
  if (!guarded.getEntry("shoulder_motor_collision_proxy", "forearm_link_collision_proxy", type) ||
      type != collision_detection::AllowedCollision::NEVER)
    throw std::runtime_error("distant contact retained resting permission");
  collision_detection::AllowedCollisionMatrix forbidden(model->getLinkModelNames(), false);
  guarded = detector.guardedMatrix(state, forbidden);
  if (!guarded.getEntry("arm_ground_keepout_proxy", "roll_holder_collision_proxy", type) ||
      type != collision_detection::AllowedCollision::NEVER)
    throw std::runtime_error("floor collision permission changed");
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
  std::cout << "fold/release accepted; unrelated roll isolated; distant contact and floor checked\n";
}
