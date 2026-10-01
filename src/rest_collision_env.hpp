#pragma once

#include <moveit/collision_detection/collision_detector_allocator.hpp>
#include <moveit/collision_detection_fcl/collision_env_fcl.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <cmath>
#include <stdexcept>

namespace lekiwi_rmf
{
// Keep the measured resting contact and its short release motion. Every other
// part and every contact outside this 0.10 rad neighborhood remains checked.
class RestCollisionEnv : public collision_detection::CollisionEnvFCL
{
public:
  using CollisionEnvFCL::CollisionEnvFCL;

  collision_detection::AllowedCollisionMatrix guardedMatrix(
      const moveit::core::RobotState& state,
      const collision_detection::AllowedCollisionMatrix& original) const
  {
    moveit::core::RobotState rest(state.getRobotModel());
    const auto* arm = state.getRobotModel()->getJointModelGroup("arm");
    if (!arm || !rest.setToDefaultValues(arm, "travel_stow"))
      throw std::runtime_error("MechanicalRest requires the named arm travel_stow pose");
    bool folded = true;
    for (const auto& name : arm->getVariableNames())
      if (name != "arm_shoulder_pan" &&
          std::abs(state.getVariablePosition(name) - rest.getVariablePosition(name)) > 0.10)
        folded = false;
    auto guarded = original;
    for (const auto& pair : state.getRobotModel()->getSRDF()->getDisabledCollisionPairs())
      if (pair.reason_ == "MechanicalRest")
        guarded.setEntry(pair.link1_, pair.link2_, folded);
    return guarded;
  }

  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                         collision_detection::CollisionResult& res,
                         const moveit::core::RobotState& state,
                         const collision_detection::AllowedCollisionMatrix& acm) const override
  {
    CollisionEnvFCL::checkSelfCollision(req, res, state, guardedMatrix(state, acm));
  }

  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                         collision_detection::CollisionResult& res,
                         const moveit::core::RobotState& state) const override
  {
    checkSelfCollision(req, res, state,
                       collision_detection::AllowedCollisionMatrix(*state.getRobotModel()->getSRDF()));
  }

  void distanceSelf(const collision_detection::DistanceRequest& req,
                    collision_detection::DistanceResult& res,
                    const moveit::core::RobotState& state) const override
  {
    auto acm = guardedMatrix(state, req.acm ? *req.acm :
        collision_detection::AllowedCollisionMatrix(*state.getRobotModel()->getSRDF()));
    auto guarded = req;
    guarded.acm = &acm;
    CollisionEnvFCL::distanceSelf(guarded, res, state);
  }
};

class RestCollisionAllocator : public collision_detection::CollisionDetectorAllocatorTemplate<
    RestCollisionEnv, RestCollisionAllocator>
{
public:
  inline static const std::string NAME = "LeKiwiRestFCL";
};
}  // namespace lekiwi_rmf
