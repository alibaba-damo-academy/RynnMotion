#include "mj_actuator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>

#include "debug_config.hpp"

namespace mujoco {

MujocoActuator::MujocoActuator(MujocoInterface &mj) :
    mj_(mj) {
}

void MujocoActuator::initActuatorSystem() {
  std::vector<int> motorIds;
  std::vector<std::string> motorNames;

  auto &robotManager = *mj_.robotManager;

  mdof_ = robotManager.getMotionDOF();
  adof_ = robotManager.getActionDOF();
  numEE_ = robotManager.getNumEndEffectors();
  jointIndices_ = robotManager.getJointIndices();
  eeIndices_ = robotManager.getEEIndices();

  int nActuators = mdof_ + numEE_ * adof_;

  std::cout << "Actuator validation: nu=" << nActuators << ", mdof=" << mdof_
            << ", adof=" << adof_ << ", numEE=" << numEE_
            << ", nu==mdof+numEE*adof " << (nActuators == mdof_ + numEE_ * adof_ ? "✓" : "✗")
            << std::endl;

  auto simKp = robotManager.getSimKp();
  auto simKd = robotManager.getSimKd();
  for (int i = 0; i < mdof_; ++i) {
    mj_.runtimeData_.kp[i] = simKp[i];
    mj_.runtimeData_.kd[i] = simKd[i];
  }

  for (int i = 0; i < nActuators; ++i) {
    if (const char *name = mj_id2name(mj_.mjModel_, mjOBJ_ACTUATOR, i)) {
      motorNames.emplace_back(name);
      motorIds.emplace_back(i);
    }
  }

  actuatorModes_ = robotManager.getActuatorModes();
  if (actuatorModes_.size() != static_cast<size_t>(nActuators)) {
    std::cerr << "Warning: Actuator modes size mismatch. Using default modes." << std::endl;
    actuatorModes_.resize(nActuators, rynn::ActuatorMode::kGeneral);
  }

  validateMapping();

  if (utils::DebugConfig::getInstance().isVerbose()) {
    printMapping();
  }

  initFromKeyframe(0);
}

void MujocoActuator::initFromKeyframe(int keyframeIndex) {
  const auto &robotManager = *mj_.robotManager;

  if (robotManager.getNumKeyframes() <= keyframeIndex) {
    return;
  }

  Eigen::VectorXd qHome = robotManager.getKeyframe(keyframeIndex);
  DEBUG_LOG("Initializing from keyframe " << keyframeIndex << ": qHome = " << qHome.transpose());

  for (int i = 0; i < mdof_; i++) {
    int actuatorIdx = jointIndices_[i];
    int jntAdr = mj_.mjModel_->actuator_trnid[2 * actuatorIdx];

    if (jntAdr < 0 || jntAdr >= mj_.mjModel_->nq) {
      continue;
    }

    mj_.mjData_->qpos[jntAdr] = qHome[i];
    mj_.mjData_->qvel[jntAdr] = 0.0;
    mj_.mjData_->qacc[jntAdr] = 0.0;
  }

  for (int i = 0; i < mdof_; i++) {
    int actuatorIdx = jointIndices_[i];
    mj_.mjData_->ctrl[actuatorIdx] = qHome[i];
  }

  for (int i = 0; i < numEE_ * adof_; i++) {
    int actuatorIdx = eeIndices_[i];
    int eeIdx = i / adof_;
    double defaultGripperCmd = robotManager.normalizeEECommand(eeIdx, 0.0);
    mj_.mjData_->ctrl[actuatorIdx] = defaultGripperCmd;
  }

  DEBUG_LOG("Actuator initialization complete: " << mdof_ << " joints, " << numEE_ << " end-effectors");
}

void MujocoActuator::update() {
  updateJointsFeedback();
  updateEEFeedback();
}

void MujocoActuator::updateJointsFeedback() {
  auto &ds = mj_.runtimeData_;

  for (int i = 0; i < mdof_; ++i) {
    int actuatorIdx = jointIndices_[i];
    int jntAdr = mj_.mjModel_->actuator_trnid[2 * actuatorIdx];

    if (actuatorIdx < 0 || actuatorIdx >= mj_.mjModel_->nu || jntAdr < 0 || jntAdr >= mj_.mjModel_->nq) {
      std::cerr << "ERROR: Invalid joint actuator index " << actuatorIdx << " or jntAdr " << jntAdr << std::endl;
      continue;
    }

    ds.qFb[i] = mj_.mjData_->qpos[jntAdr];
    ds.qdFb[i] = mj_.mjData_->qvel[jntAdr];
    ds.qtauFb[i] = mj_.mjData_->actuator_force[actuatorIdx];
  }
}

void MujocoActuator::updateEEFeedback() {
  auto &ds = mj_.runtimeData_;
  const auto &robotManager = *mj_.robotManager;
  auto robotType = robotManager.getRobotType();

  if (numEE_ == 0) return;

  if (ds.gripperFeedbacks.size() != static_cast<size_t>(numEE_)) {
    ds.gripperFeedbacks.resize(numEE_);
  }

  for (int i = 0; i < numEE_ * adof_; ++i) {
    int actuatorIdx = eeIndices_[i];
    int eeIdx = i / adof_;

    if (actuatorIdx < 0 || actuatorIdx >= mj_.mjModel_->nu) {
      std::cerr << "ERROR: Invalid EE actuator index " << actuatorIdx << std::endl;
      continue;
    }

    auto &gripperFb = ds.gripperFeedbacks[eeIdx];

    if (robotType == rynn::RobotType::fr3 || robotType == rynn::RobotType::dual_fr3) {
      double tendonLength = mj_.mjData_->actuator_length[actuatorIdx];
      double maxGripperDistance = 0.04;
      gripperFb.posFb = tendonLength / maxGripperDistance;
      gripperFb.posFb = std::max(0.0, std::min(1.0, gripperFb.posFb));
      gripperFb.tauFb = mj_.mjData_->actuator_force[actuatorIdx];
    } else {
      int jntAdr = mj_.mjModel_->actuator_trnid[2 * actuatorIdx];
      if (jntAdr >= 0 && jntAdr < mj_.mjModel_->nq) {
        gripperFb.posFb = mj_.mjData_->qpos[jntAdr];
      }
      gripperFb.tauFb = mj_.mjData_->actuator_force[actuatorIdx];
    }

    if (mj_._show_gripper_debug) {
      std::cout << "EE[" << eeIdx << "] feedback - "
                << "actuator " << actuatorIdx << ": "
                << "posFb=" << gripperFb.posFb << ", "
                << "tauFb=" << gripperFb.tauFb << std::endl;
    }
  }
}

void MujocoActuator::setCommand() {
  setJointsCommand();
  setEECommand();

  if (mj_._show_motor_qCmd) {
    Eigen::VectorXd qCmd, qdCmd_unused, qtauCmd_unused;
    mj_.runtimeData_.getJointsCommand(qCmd, qdCmd_unused, qtauCmd_unused);
    std::cout << "qCmd: " << qCmd.transpose() << std::endl;
  }
}

void MujocoActuator::setJointsCommand() {
  Eigen::VectorXd qCmd, qdCmd, qtauCmd;
  Eigen::VectorXd qFb, qdFb, qtauFb;
  mj_.runtimeData_.getJointsCommand(qCmd, qdCmd, qtauCmd);
  mj_.runtimeData_.getJointsFeedback(qFb, qdFb, qtauFb);

  for (int i = 0; i < mdof_; ++i) {
    int actuatorIdx = jointIndices_[i];

    if (actuatorIdx < 0 || actuatorIdx >= mj_.mjModel_->nu) {
      std::cerr << "ERROR: Invalid joint actuator index " << actuatorIdx << std::endl;
      continue;
    }

    if (actuatorIdx < static_cast<int>(actuatorModes_.size())) {
      switch (actuatorModes_[actuatorIdx]) {
      case rynn::ActuatorMode::kTorque:
        mj_.mjData_->ctrl[actuatorIdx] = pvtpidTorque(i, qCmd, qdCmd, qtauCmd, qFb, qdFb);
        break;
      case rynn::ActuatorMode::kVelocity:
        mj_.mjData_->ctrl[actuatorIdx] = qdCmd(i);
        break;
      case rynn::ActuatorMode::kIntVelocity:
        mj_.mjData_->ctrl[actuatorIdx] = qdCmd(i);
        break;
      case rynn::ActuatorMode::kPosition:
      case rynn::ActuatorMode::kGeneral:
      case rynn::ActuatorMode::kDamper:
      case rynn::ActuatorMode::kCylinder:
      case rynn::ActuatorMode::kMuscle:
      case rynn::ActuatorMode::kAdhesion:
      case rynn::ActuatorMode::kPlugin:
      default:
        mj_.mjData_->ctrl[actuatorIdx] = qCmd(i);
        break;
      }
    } else {
      mj_.mjData_->ctrl[actuatorIdx] = qCmd(i);
    }
  }
}

void MujocoActuator::setEECommand() {
  auto &ds = mj_.runtimeData_;
  const auto &robotManager = *mj_.robotManager;

  if (numEE_ == 0) return;

  if (ds.gripperCommands.size() != static_cast<size_t>(numEE_)) {
    ds.gripperCommands.resize(numEE_);
  }

  for (int i = 0; i < numEE_ * adof_; ++i) {
    int actuatorIdx = eeIndices_[i];
    int eeIdx = i / adof_;

    if (actuatorIdx < 0 || actuatorIdx >= mj_.mjModel_->nu) {
      std::cerr << "ERROR: Invalid EE actuator index " << actuatorIdx << std::endl;
      continue;
    }

    auto &gripperCmd = ds.gripperCommands[eeIdx];

    double actualGripperControl = robotManager.normalizeEECommand(eeIdx, gripperCmd.posCmd);
    mj_.mjData_->ctrl[actuatorIdx] = actualGripperControl;

    if (mj_._show_gripper_debug) {
      std::cout << "EE[" << eeIdx << "] command - "
                << "actuator " << actuatorIdx << ": "
                << "posCmd=" << gripperCmd.posCmd << " → "
                << "ctrl=" << actualGripperControl << ", "
                << "qtauCmd=" << gripperCmd.qtauCmd << std::endl;
    }
  }
}

double MujocoActuator::pvtpidTorque(int motorIndex,
                                    const Eigen::VectorXd &qCmd,
                                    const Eigen::VectorXd &qdCmd,
                                    const Eigen::VectorXd &qtauCmd,
                                    const Eigen::VectorXd &qFb,
                                    const Eigen::VectorXd &qdFb) {
  double qError = qCmd(motorIndex) - qFb(motorIndex);
  double qdError = qdCmd(motorIndex) - qdFb(motorIndex);
  double fftorque = qtauCmd(motorIndex);

  double posTorque = mj_.runtimeData_.kp[motorIndex] * qError;
  double velTorque = mj_.runtimeData_.kd[motorIndex] * qdError;

  return fftorque + posTorque + velTorque;
}

void MujocoActuator::validateMapping() {
  int nu = mj_.mjModel_->nu;

  for (size_t i = 0; i < jointIndices_.size(); ++i) {
    int idx = jointIndices_[i];
    if (idx < 0 || idx >= nu) {
      throw std::runtime_error("Invalid joint actuator index: " + std::to_string(idx) + " at position " + std::to_string(i));
    }
  }

  for (size_t i = 0; i < eeIndices_.size(); ++i) {
    int idx = eeIndices_[i];
    if (idx < 0 || idx >= nu) {
      throw std::runtime_error("Invalid EE actuator index: " + std::to_string(idx) + " at position " + std::to_string(i));
    }
  }

  std::set<int> allIndices(jointIndices_.begin(), jointIndices_.end());
  for (int idx : eeIndices_) {
    if (allIndices.count(idx)) {
      throw std::runtime_error("Actuator " + std::to_string(idx) + " appears in both joint and EE indices!");
    }
  }

  std::cout << "✓ Actuator mapping validation passed" << std::endl;
}

void MujocoActuator::printMapping() {
  std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
  std::cout << "║         Actuator Index Mapping Table              ║" << std::endl;
  std::cout << "╠════════════════════════════════════════════════════╣" << std::endl;
  std::cout << "║ Total actuators (nu): " << mj_.mjModel_->nu << std::endl;
  std::cout << "║ Motion DOF (mdof):    " << mdof_ << std::endl;
  std::cout << "║ End-effectors (numEE): " << numEE_ << std::endl;
  std::cout << "║ Action DOF (adof):    " << adof_ << std::endl;
  std::cout << "╠════════════════════════════════════════════════════╣" << std::endl;

  std::cout << "║ Joint Actuators:" << std::endl;
  for (size_t i = 0; i < jointIndices_.size(); ++i) {
    int actId = jointIndices_[i];
    const char *name = mj_id2name(mj_.mjModel_, mjOBJ_ACTUATOR, actId);
    std::cout << "║   [" << i << "] → actuator " << actId
              << " (" << (name ? name : "unnamed") << ")" << std::endl;
  }

  std::cout << "║ End-Effector Actuators:" << std::endl;
  for (size_t i = 0; i < eeIndices_.size(); ++i) {
    int actId = eeIndices_[i];
    const char *name = mj_id2name(mj_.mjModel_, mjOBJ_ACTUATOR, actId);
    int eeIdx = i / adof_;
    std::cout << "║   EE[" << eeIdx << "] → actuator " << actId
              << " (" << (name ? name : "unnamed") << ")" << std::endl;
  }

  std::cout << "╚════════════════════════════════════════════════════╝\n"
            << std::endl;
}

} // namespace mujoco
