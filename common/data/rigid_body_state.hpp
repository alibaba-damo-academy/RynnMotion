#pragma once

#include <Eigen/Dense>
#include <string>

#include "data_base.hpp"
#include "pose.hpp"

namespace data {

/**
 * @brief Unified rigid body state representation
 *
 * Replaces FrameSensor, Object, and BodyFeedback with a single type.
 * Used for: end-effectors, objects, body links, camera frames, etc.
 */
struct RigidBodyState : public Pose {
  std::string name;

  Eigen::Matrix<double, 6, 1> velocity;     // [linear_xyz, angular_xyz]
  Eigen::Matrix<double, 6, 1> acceleration; // [linear_xyz, angular_xyz]

  RigidBodyState(const std::string &name = "") :
      Pose(),
      name(name),
      velocity(Eigen::Matrix<double, 6, 1>::Zero()),
      acceleration(Eigen::Matrix<double, 6, 1>::Zero()) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<RigidBodyState>(*this);
  }

  RigidBodyState &operator=(const RigidBodyState &other) {
    if (this != &other) {
      Pose::operator=(other);
      name = other.name;
      velocity = other.velocity;
      acceleration = other.acceleration;
    }
    return *this;
  }

  RigidBodyState(const RigidBodyState &other) :
      Pose(other),
      name(other.name),
      velocity(other.velocity),
      acceleration(other.acceleration) {
  }
};

/**
 * @brief Body trajectory plan (pose + spatial velocity)
 */
struct BodyPlan : public Pose {
  Eigen::Matrix<double, 6, 1> velocity; // 6x1 spatial velocity (linear + angular)

  BodyPlan() :
      Pose(),
      velocity(Eigen::Matrix<double, 6, 1>::Zero()) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<BodyPlan>(*this);
  }

  BodyPlan &operator=(const BodyPlan &other) {
    if (this != &other) {
      Pose::operator=(other);
      velocity = other.velocity;
    }
    return *this;
  }

  BodyPlan(const BodyPlan &other) :
      Pose(other), velocity(other.velocity) {
  }
};

// Semantic aliases for clarity
using FrameSensor = RigidBodyState;  // End-effector frames
using ObjectState = RigidBodyState;   // Manipulated objects
using BodyFeedback = RigidBodyState;  // Body/link feedback
using Object = RigidBodyState;        // Objects (backward compatibility)

} // namespace data
