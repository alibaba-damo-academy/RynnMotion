#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

enum class JointConstraintMode {
  PositionVelocity,
  PositionVelocityAcc,
  PositionVelocityAccJerk
};

template <size_t DOFs>
class TrajGen {
public:
  explicit TrajGen(double dt);
  virtual ~TrajGen() = default;

  /// @brief Set start state
  /// @param pos Start position
  /// @param vel Start velocity
  /// @param acc Start acceleration
  virtual void setStartState(const Eigen::VectorXd &pos,
                             const Eigen::VectorXd &vel = Eigen::VectorXd::Zero(DOFs),
                             const Eigen::VectorXd &acc = Eigen::VectorXd::Zero(DOFs)) {
    input_.current_position = pos;
    input_.current_velocity = vel.size() == DOFs ? vel : (pos - lastPlanPos_) / dt_;
    input_.current_acceleration = acc.size() == DOFs ? acc : (input_.current_velocity - lastPlanVel_) / dt_;
  }

  /// @brief Set target state
  /// @param pos Target position
  /// @param vel Target velocity
  /// @param acc Target acceleration
  virtual void setTargetState(const Eigen::VectorXd &pos,
                              const Eigen::VectorXd &vel = Eigen::VectorXd::Zero(DOFs),
                              const Eigen::VectorXd &acc = Eigen::VectorXd::Zero(DOFs)) {
    input_.target_position = pos;
    input_.target_velocity = vel.size() == DOFs ? vel : (pos - lastTargetPos_) / dt_;
    input_.target_acceleration = acc.size() == DOFs ? acc : (input_.target_velocity - lastTargetVel_) / dt_;
  }

  /// @brief Update trajectory calculation
  /// @return Trajectory status
  Result update();

  void passToInput();
  double getTime() const;
  double getDuration() const;

  const InputParameter<DOFs, EigenVector> &getInput() const;
  const OutputParameter<DOFs, EigenVector> &getOutput() const;
  void reset();

protected:
  std::unique_ptr<Ruckig<DOFs, EigenVector>> otg_;
  InputParameter<DOFs, EigenVector> input_;
  OutputParameter<DOFs, EigenVector> output_;

  Eigen::Matrix<double, DOFs, 1> lastTargetPos_;
  Eigen::Matrix<double, DOFs, 1> lastTargetVel_;
  Eigen::Matrix<double, DOFs, 1> lastTargetAcc_;
  Eigen::Matrix<double, DOFs, 1> lastPlanPos_;
  Eigen::Matrix<double, DOFs, 1> lastPlanVel_;
  Eigen::Matrix<double, DOFs, 1> lastPlanAcc_;

  double dt_;

  virtual void applySaturation();
  void updateStateHistory();
};

template <size_t DOFs>
class JointTrajGen : public TrajGen<DOFs> {
public:
  explicit JointTrajGen(double dt) :
      TrajGen<DOFs>(dt) {
    qUpperLimits_.setConstant(1e3);
    qLowerLimits_.setConstant(-1e3);
    qdMax_.setConstant(1e3);
    qddMax_.setConstant(1e3);
    qdddMax_.setConstant(1e3);
  }

  void setStartState(const Eigen::VectorXd &pos,
                     const Eigen::VectorXd &vel = Eigen::VectorXd::Zero(DOFs),
                     const Eigen::VectorXd &acc = Eigen::VectorXd::Zero(DOFs)) override {
    assert(pos.size() == DOFs);
    this->input_.current_position = pos;
    if (vel.size() == DOFs) {
      this->input_.current_velocity = vel;
    } else {
      this->input_.current_velocity = (pos - this->lastPlanPos_) / this->dt_;
    }
    if (acc.size() == DOFs) {
      this->input_.current_acceleration = acc;
    } else {
      this->input_.current_acceleration = (this->input_.current_velocity - this->lastPlanVel_) / this->dt_;
    }
  }

  void setTargetState(const Eigen::VectorXd &pos,
                      const Eigen::VectorXd &vel = Eigen::VectorXd::Zero(DOFs),
                      const Eigen::VectorXd &acc = Eigen::VectorXd::Zero(DOFs)) override {
    assert(pos.size() == DOFs);
    this->input_.target_position = pos;
    if (vel.size() == DOFs) {
      this->input_.target_velocity = vel;
    } else {
      this->input_.target_velocity = (pos - this->lastTargetPos_) / this->dt_;
    }
    if (acc.size() == DOFs) {
      this->input_.target_acceleration = acc;
    } else {
      this->input_.target_acceleration = (this->input_.target_velocity - this->lastTargetVel_) / this->dt_;
    }
  }

  /// @brief Set joint velocity limits
  /// @param qdMax Maximum joint velocities
  void setJointMotionLimits(const Eigen::VectorXd &qdMax);

  /// @brief Set joint velocity and acceleration limits
  /// @param qdMax Maximum joint velocities
  /// @param qddMax Maximum joint accelerations
  void setJointMotionLimits(const Eigen::VectorXd &qdMax, const Eigen::VectorXd &qddMax);

  /// @brief Set joint velocity, acceleration and jerk limits
  /// @param qdMax Maximum joint velocities
  /// @param qddMax Maximum joint accelerations
  /// @param qdddMax Maximum joint jerks
  void setJointMotionLimits(const Eigen::VectorXd &qdMax,
                            const Eigen::VectorXd &qddMax,
                            const Eigen::VectorXd &qdddMax);

  /// @brief Set joint position limits
  /// @param qUpperLimits Upper position limits
  /// @param qLowerLimits Lower position limits
  void setJointPosLimits(const Eigen::VectorXd &qUpperLimits,
                         const Eigen::VectorXd &qLowerLimits);

  void clearJointLimits();
  void clearPositionLimits();

  Eigen::VectorXd getCurrPos() const {
    return this->output_.new_position;
  }

  Eigen::VectorXd getCurrVel() const {
    return this->output_.new_velocity;
  }

  Eigen::VectorXd getCurrAcc() const {
    return this->output_.new_acceleration;
  }

protected:
  void applySaturation() override;

private:
  JointConstraintMode constraintMode_ = JointConstraintMode::PositionVelocity;

  Eigen::Matrix<double, DOFs, 1> qUpperLimits_;
  Eigen::Matrix<double, DOFs, 1> qLowerLimits_;
  Eigen::Matrix<double, DOFs, 1> qdMax_;
  Eigen::Matrix<double, DOFs, 1> qddMax_;
  Eigen::Matrix<double, DOFs, 1> qdddMax_;
};

class EEPoseTrajGen : public TrajGen<4> {
public:
  explicit EEPoseTrajGen(double dt);

  // Bring base class methods into scope to avoid hiding warnings
  using TrajGen<4>::setStartState;
  using TrajGen<4>::setTargetState;

  /// @brief Set start state (pose interface)
  /// @param pos0 Start position
  /// @param quat0 Start orientation (quaternion)
  /// @param vel 4DOF velocity vector
  /// @param acc 4DOF acceleration vector
  void setStartState(const Eigen::Vector3d &pos0, const Eigen::Quaterniond &quat0,
                     const Eigen::Vector4d &vel = Eigen::Vector4d::Zero(),
                     const Eigen::Vector4d &acc = Eigen::Vector4d::Zero());

  /// @brief Set target state (pose interface)
  /// @param pos1 Target position
  /// @param quat1 Target orientation (quaternion)
  /// @param vel 4DOF velocity vector
  /// @param acc 4DOF acceleration vector
  void setTargetState(const Eigen::Vector3d &pos1, const Eigen::Quaterniond &quat1,
                      const Eigen::Vector4d &vel = Eigen::Vector4d::Zero(),
                      const Eigen::Vector4d &acc = Eigen::Vector4d::Zero());

  /// @brief Set TCP limits
  /// @param maxSpeed Maximum TCP speed (m/s)
  /// @param maxAcc Maximum TCP acceleration (m/s²)
  void setTcpLimits(double maxSpeed, double maxAcc = std::numeric_limits<double>::max());

  void clearTcpLimits();

  /// @brief Get current TCP speed
  /// @return Current TCP speed (m/s)
  double getCurrTcpSpeed() const;

  /// @brief Get current TCP acceleration
  /// @return Current TCP acceleration (m/s²)
  double getCurrTcpAcc() const;

  /// @brief Get current pose
  /// @return Current position and orientation
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> getCurrPose() const;

  /// @brief Get current velocity
  /// @return Current linear velocity
  Eigen::Vector3d getCurrVel() const;

  /// @brief Get current acceleration
  /// @return Current linear acceleration
  Eigen::Vector3d getCurrAcc() const;

  Result update();

protected:
  void applySaturation() override;

private:
  double maxTcpSpeed_;
  double maxTcpAcc_;

  Eigen::Quaterniond currQuat_;
  Eigen::Quaterniond targetQuat_;
  Eigen::Vector3d actionAxis_;
  double totalActionAngle_;
};

template <size_t DOFs>
TrajGen<DOFs>::TrajGen(double dt) :
    otg_(std::make_unique<Ruckig<DOFs, EigenVector>>(dt)), dt_(dt) {
  lastTargetPos_.setZero();
  lastTargetVel_.setZero();
  lastTargetAcc_.setZero();
  lastPlanPos_.setZero();
  lastPlanVel_.setZero();
  lastPlanAcc_.setZero();
}

template <size_t DOFs>
Result TrajGen<DOFs>::update() {
  applySaturation();

  auto currPos = input_.current_position;
  auto currVel = input_.current_velocity;
  auto currAcc = input_.current_acceleration;

  auto result = otg_->update(input_, output_);

  if (result != Result::Working && result != Result::Finished) {
    auto originalTargetAcc = input_.target_acceleration;
    input_.target_acceleration.setZero();

    result = otg_->update(input_, output_);

    if (result == Result::Working || result == Result::Finished) {
    } else {
      input_.target_acceleration = originalTargetAcc;
      output_.new_position = currPos;
      output_.new_velocity = currVel;
      output_.new_acceleration = currAcc;
    }
  }

  updateStateHistory();

  return result;
}

template <size_t DOFs>
void TrajGen<DOFs>::passToInput() {
  output_.pass_to_input(input_);
}

template <size_t DOFs>
double TrajGen<DOFs>::getTime() const {
  return output_.time;
}

template <size_t DOFs>
double TrajGen<DOFs>::getDuration() const {
  return output_.trajectory.get_duration();
}

template <size_t DOFs>
const InputParameter<DOFs, EigenVector> &TrajGen<DOFs>::getInput() const {
  return input_;
}

template <size_t DOFs>
const OutputParameter<DOFs, EigenVector> &TrajGen<DOFs>::getOutput() const {
  return output_;
}

template <size_t DOFs>
void TrajGen<DOFs>::reset() {
  output_.time = 0.0;
  lastTargetPos_.setZero();
  lastTargetVel_.setZero();
  lastTargetAcc_.setZero();
  lastPlanPos_.setZero();
  lastPlanVel_.setZero();
  lastPlanAcc_.setZero();
}

template <size_t DOFs>
void TrajGen<DOFs>::applySaturation() {
}

template <size_t DOFs>
void TrajGen<DOFs>::updateStateHistory() {
  lastTargetPos_ = input_.target_position;
  lastTargetVel_ = input_.target_velocity;
  lastTargetAcc_ = input_.target_acceleration;
  lastPlanPos_ = output_.new_position;
  lastPlanVel_ = output_.new_velocity;
  lastPlanAcc_ = output_.new_acceleration;
}

template <size_t DOFs>
void JointTrajGen<DOFs>::setJointMotionLimits(const Eigen::VectorXd &qdMax) {
  assert(qdMax.size() == DOFs);

  qdMax_ = qdMax;

  this->input_.max_velocity = qdMax;

  this->input_.max_acceleration.setConstant(1e3);
  this->input_.max_jerk.setConstant(1e3);
  qddMax_.setConstant(1e3);
  qdddMax_.setConstant(1e3);

  constraintMode_ = JointConstraintMode::PositionVelocity;
}

template <size_t DOFs>
void JointTrajGen<DOFs>::setJointMotionLimits(const Eigen::VectorXd &qdMax, const Eigen::VectorXd &qddMax) {
  assert(qdMax.size() == DOFs);
  assert(qddMax.size() == DOFs);

  qdMax_ = qdMax;
  qddMax_ = qddMax;

  this->input_.max_velocity = qdMax;
  this->input_.max_acceleration = qddMax;

  this->input_.max_jerk.setConstant(1e3);
  qdddMax_.setConstant(1e3);

  constraintMode_ = JointConstraintMode::PositionVelocityAcc;
}

template <size_t DOFs>
void JointTrajGen<DOFs>::setJointMotionLimits(const Eigen::VectorXd &qdMax,
                                              const Eigen::VectorXd &qddMax,
                                              const Eigen::VectorXd &qdddMax) {
  assert(qdMax.size() == DOFs);
  assert(qddMax.size() == DOFs);
  assert(qdddMax.size() == DOFs);

  qdMax_ = qdMax;
  qddMax_ = qddMax;
  qdddMax_ = qdddMax;

  this->input_.max_velocity = qdMax;
  this->input_.max_acceleration = qddMax;
  this->input_.max_jerk = qdddMax;

  constraintMode_ = JointConstraintMode::PositionVelocityAccJerk;
}

template <size_t DOFs>
void JointTrajGen<DOFs>::setJointPosLimits(const Eigen::VectorXd &qUpperLimits,
                                           const Eigen::VectorXd &qLowerLimits) {
  assert(qUpperLimits.size() == DOFs);
  assert(qLowerLimits.size() == DOFs);

  qUpperLimits_ = qUpperLimits;
  qLowerLimits_ = qLowerLimits;

  // Position limits don't change constraint mode - velocity must always be set
}

template <size_t DOFs>
void JointTrajGen<DOFs>::clearJointLimits() {
  constraintMode_ = JointConstraintMode::PositionVelocity;
  qUpperLimits_.setConstant(1e3);
  qLowerLimits_.setConstant(-1e3);
  qdMax_.setConstant(1e3);
  qddMax_.setConstant(1e3);
  qdddMax_.setConstant(1e3);

  // Reset Ruckig limits to defaults
  this->input_.max_velocity.setConstant(1e3);
  this->input_.max_acceleration.setConstant(1e3);
  this->input_.max_jerk.setConstant(1e3);
}

template <size_t DOFs>
void JointTrajGen<DOFs>::clearPositionLimits() {
  qUpperLimits_.setConstant(1e3);
  qLowerLimits_.setConstant(-1e3);
  // Note: Don't change constraintMode_ here as velocity/acceleration limits might still be set
}

template <size_t DOFs>
void JointTrajGen<DOFs>::applySaturation() {
  switch (constraintMode_) {
  case JointConstraintMode::PositionVelocity: {
    for (size_t i = 0; i < DOFs; ++i) {
      if (this->input_.target_position[i] > qUpperLimits_[i] || this->input_.target_position[i] < qLowerLimits_[i]) {
        this->input_.target_position[i] = std::clamp(this->input_.target_position[i],
                                                     qLowerLimits_[i], qUpperLimits_[i]);
        // Zero velocity and acceleration only for this joint
        this->input_.target_velocity[i] = 0.0;
        this->input_.target_acceleration[i] = 0.0;
      }
    }

    // Apply velocity limits per joint
    for (size_t i = 0; i < DOFs; ++i) {
      if (std::abs(this->input_.target_velocity[i]) > qdMax_[i]) {
        this->input_.target_velocity[i] = std::clamp(this->input_.target_velocity[i],
                                                     -qdMax_[i], qdMax_[i]);
        // Zero acceleration only for this joint
        this->input_.target_acceleration[i] = 0.0;
      }
      // Set Ruckig internal limits
      this->input_.max_velocity[i] = qdMax_[i];
      this->input_.max_acceleration[i] = qddMax_[i];
      this->input_.max_jerk[i] = qdddMax_[i];
    }
  } break;

  case JointConstraintMode::PositionVelocityAcc:
    // Apply position, velocity, and acceleration constraints
    {
      // Apply position limits per joint
      for (size_t i = 0; i < DOFs; ++i) {
        if (this->input_.target_position[i] > qUpperLimits_[i] || this->input_.target_position[i] < qLowerLimits_[i]) {
          this->input_.target_position[i] = std::clamp(this->input_.target_position[i],
                                                       qLowerLimits_[i], qUpperLimits_[i]);
          // Zero velocity and acceleration only for this joint
          this->input_.target_velocity[i] = 0.0;
          this->input_.target_acceleration[i] = 0.0;
        }
      }

      // Apply velocity limits per joint
      for (size_t i = 0; i < DOFs; ++i) {
        if (std::abs(this->input_.target_velocity[i]) > qdMax_[i]) {
          this->input_.target_velocity[i] = std::clamp(this->input_.target_velocity[i],
                                                       -qdMax_[i], qdMax_[i]);
          // Zero acceleration only for this joint
          this->input_.target_acceleration[i] = 0.0;
        }
        // Set Ruckig internal limits
        this->input_.max_velocity[i] = qdMax_[i];
      }

      // Apply acceleration limits per joint
      for (size_t i = 0; i < DOFs; ++i) {
        this->input_.target_acceleration[i] = std::clamp(this->input_.target_acceleration[i],
                                                         -qddMax_[i], qddMax_[i]);
        // Set Ruckig internal limits
        this->input_.max_acceleration[i] = qddMax_[i];
        this->input_.max_jerk[i] = qdddMax_[i];
      }
    }
    break;

  case JointConstraintMode::PositionVelocityAccJerk:
    // Apply all constraints and ensure Ruckig limits are properly set
    {
      // Apply position limits per joint
      for (size_t i = 0; i < DOFs; ++i) {
        if (this->input_.target_position[i] > qUpperLimits_[i] || this->input_.target_position[i] < qLowerLimits_[i]) {
          this->input_.target_position[i] = std::clamp(this->input_.target_position[i],
                                                       qLowerLimits_[i], qUpperLimits_[i]);
          // Zero velocity and acceleration only for this joint
          this->input_.target_velocity[i] = 0.0;
          this->input_.target_acceleration[i] = 0.0;
        }
      }

      // Apply velocity limits per joint
      for (size_t i = 0; i < DOFs; ++i) {
        if (std::abs(this->input_.target_velocity[i]) > qdMax_[i]) {
          this->input_.target_velocity[i] = std::clamp(this->input_.target_velocity[i],
                                                       -qdMax_[i], qdMax_[i]);
          // Zero acceleration only for this joint
          this->input_.target_acceleration[i] = 0.0;
        }
        // Set Ruckig internal limits
        this->input_.max_velocity[i] = std::min(this->input_.max_velocity[i], qdMax_[i]);
      }

      // Apply acceleration limits per joint
      for (size_t i = 0; i < DOFs; ++i) {
        this->input_.target_acceleration[i] = std::clamp(this->input_.target_acceleration[i],
                                                         -qddMax_[i], qddMax_[i]);
        // Set Ruckig internal limits
        this->input_.max_acceleration[i] = std::min(this->input_.max_acceleration[i], qddMax_[i]);
      }

      // Apply jerk limits (only affects Ruckig internal limits)
      for (size_t i = 0; i < DOFs; ++i) {
        this->input_.max_jerk[i] = std::min(this->input_.max_jerk[i], qdddMax_[i]);
      }
    }
    break;
  }
}
