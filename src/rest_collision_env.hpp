#pragma once

#include <moveit/collision_detection/collision_detector_allocator.hpp>
#include <moveit/collision_detection_fcl/collision_env_fcl.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lekiwi_rmf
{
// Resting contact follows the relative placement of the two physical parts.
// A coupled release can move several joints while leaving those parts together.
class RestCollisionEnv : public collision_detection::CollisionEnvFCL
{
public:
  using CollisionEnvFCL::CollisionEnvFCL;

  collision_detection::AllowedCollisionMatrix guardedMatrix(
      const moveit::core::RobotState& state,
      const collision_detection::AllowedCollisionMatrix& original) const
  {
    std::array<moveit::core::RobotState, 2> rests{
        moveit::core::RobotState(state.getRobotModel()), moveit::core::RobotState(state.getRobotModel())};
    const auto* arm = state.getRobotModel()->getJointModelGroup("arm");
    const std::array<const char*, 2> names{"travel_stow", "supported_rest"};
    for (std::size_t i = 0; i < rests.size(); ++i)
    {
      if (!arm || !rests[i].setToDefaultValues(arm, names[i]))
        throw std::runtime_error(std::string("MechanicalRest requires the named arm pose ") + names[i]);
      rests[i].update();
    }
    auto guarded = original;
    for (const auto& pair : state.getRobotModel()->getSRDF()->getDisabledCollisionPairs())
      if (pair.reason_ == "MechanicalRest")
      {
        const Eigen::Isometry3d actual = state.getGlobalLinkTransform(pair.link1_).inverse() *
                                        state.getGlobalLinkTransform(pair.link2_);
        bool resting = false;
        for (const auto& rest : rests)
        {
          const Eigen::Isometry3d expected = rest.getGlobalLinkTransform(pair.link1_).inverse() *
                                            rest.getGlobalLinkTransform(pair.link2_);
          // Each measured support keeps the same relative-placement bounds.
          if ((actual.translation() - expected.translation()).norm() <= 0.025 &&
              Eigen::AngleAxisd(expected.linear().transpose() * actual.linear()).angle() <= 0.10)
          {
            resting = true;
            break;
          }
        }
        guarded.setEntry(pair.link1_, pair.link2_, resting);
      }
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
