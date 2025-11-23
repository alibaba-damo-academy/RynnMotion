#include "trajgen.hpp"

EEPoseTrajGen::EEPoseTrajGen(double dt) :
    TrajGen<4>(dt) {
  maxTcpSpeed_ = std::numeric_limits<double>::max();
  maxTcpAcc_ = std::numeric_limits<double>::max();

  input_.max_velocity << 2.0, 2.0, 2.0, 20.0;      // [x, y, z, angle] - 10x scaling for angle
  input_.max_acceleration << 5.0, 5.0, 5.0, 50.0;  // [x, y, z, angle] - 10x scaling for angle
  input_.max_jerk << 20.0, 20.0, 20.0, 200.0;      // [x, y, z, angle] - 10x scaling for angle

  currQuat_ = Eigen::Quaterniond::Identity();
  targetQuat_ = Eigen::Quaterniond::Identity();
  actionAxis_ = Eigen::Vector3d::UnitZ();
  totalActionAngle_ = 0.0;
}

void EEPoseTrajGen::setTcpLimits(double maxSpeed, double maxAcc) {
  maxTcpSpeed_ = maxSpeed;
  if (maxAcc < std::numeric_limits<double>::max()) {
    maxTcpAcc_ = maxAcc;
  }
  // Note: Actual limit setting is handled in applySaturation() which is called during update()
}

void EEPoseTrajGen::clearTcpLimits() {
  maxTcpSpeed_ = std::numeric_limits<double>::max();
  maxTcpAcc_ = std::numeric_limits<double>::max();
}

double EEPoseTrajGen::getCurrTcpSpeed() const {
  return output_.new_velocity.head<3>().norm();
}

double EEPoseTrajGen::getCurrTcpAcc() const {
  return output_.new_acceleration.head<3>().norm();
}

void EEPoseTrajGen::setStartState(const Eigen::Vector3d &pos0, const Eigen::Quaterniond &quat0,
                                  const Eigen::Vector4d &vel,
                                  const Eigen::Vector4d &acc) {
  currQuat_ = quat0;

  // Set 4DOF state: [x, y, z, angle] - angle starts at 0
  Eigen::Vector4d state4d;
  state4d.head<3>() = pos0;
  state4d(3) = 0.0; // Start angle at 0
  input_.current_position = state4d;

  // Set 4DOF velocity and acceleration
  input_.current_velocity = vel;
  input_.current_acceleration = acc;
}

void EEPoseTrajGen::setTargetState(const Eigen::Vector3d &pos1, const Eigen::Quaterniond &quat1,
                                   const Eigen::Vector4d &vel,
                                   const Eigen::Vector4d &acc) {
  targetQuat_ = quat1;

  // Calculate action quaternion and extract angle-axis
  Eigen::Quaterniond actionQuat = currQuat_.inverse() * quat1;
  Eigen::AngleAxisd actionAxisAngle(actionQuat);
  actionAxis_ = actionAxisAngle.axis();
  totalActionAngle_ = actionAxisAngle.angle();

  // Set 4DOF target state: [x, y, z, total_angle]
  Eigen::Vector4d state4d;
  state4d.head<3>() = pos1;
  state4d(3) = totalActionAngle_; // Target angle is the total rotation
  input_.target_position = state4d;

  // Set 4DOF target velocity and acceleration
  input_.target_velocity = vel;
  input_.target_acceleration = acc;
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> EEPoseTrajGen::getCurrPose() const {
  // Get current position from 4DOF output (first 3 components)
  Eigen::Vector3d pos = output_.new_position.head<3>();

  // Get current angle from 4DOF output (4th component)
  double currentAngle = output_.new_position(3);

  // Create action quaternion from current angle and stored axis
  Eigen::AngleAxisd currentAction(currentAngle, actionAxis_);
  Eigen::Quaterniond actionQuat(currentAction);

  // Apply action to original quaternion
  Eigen::Quaterniond quat = currQuat_ * actionQuat;

  return std::make_pair(pos, quat);
}

Eigen::Vector3d EEPoseTrajGen::getCurrVel() const {
  return output_.new_velocity.head<3>();
}

Eigen::Vector3d EEPoseTrajGen::getCurrAcc() const {
  return output_.new_acceleration.head<3>();
}

Result EEPoseTrajGen::update() {
  Result result = TrajGen<4>::update();
  return result;
}

void EEPoseTrajGen::applySaturation() {
  // Always ensure angular axis has reasonable limits relative to linear axes
  // The constructor sets defaults: vel=[2,2,2,1], acc=[5,5,5,5], jerk=[20,20,20,20]
  // But angular should have higher limits to not constrain rotational motion
  
  // Set reasonable angular limits (10x scaling factor)
  double angular_vel_scale = 10.0;
  double angular_acc_scale = 10.0;
  double angular_jerk_scale = 10.0;
  
  if (maxTcpSpeed_ < std::numeric_limits<double>::max()) {
    // Set TCP speed limit for first 3 axes (x, y, z)
    input_.max_velocity.head<3>().setConstant(maxTcpSpeed_);
    // Set reasonable angular velocity limit for 4th axis (angle)
    input_.max_velocity(3) = maxTcpSpeed_ * angular_vel_scale;
  } else {
    // Use default linear velocity but scaled angular velocity
    input_.max_velocity(3) = input_.max_velocity(0) * angular_vel_scale;
  }
  
  if (maxTcpAcc_ < std::numeric_limits<double>::max()) {
    // Set TCP acceleration limit for first 3 axes (x, y, z)
    input_.max_acceleration.head<3>().setConstant(maxTcpAcc_);
    // Set reasonable angular acceleration limit for 4th axis (angle)
    input_.max_acceleration(3) = maxTcpAcc_ * angular_acc_scale;
  } else {
    // Use default linear acceleration but scaled angular acceleration
    input_.max_acceleration(3) = input_.max_acceleration(0) * angular_acc_scale;
  }
  
  // Always set reasonable jerk limits for angular motion
  // Use the current linear jerk limit as base
  input_.max_jerk(3) = input_.max_jerk(0) * angular_jerk_scale;
}
