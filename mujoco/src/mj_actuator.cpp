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

  DEBUG_LOG("Actuator validation: nu=" << nActuators << ", mdof=" << mdof_
            << ", adof=" << adof_ << ", numEE=" << numEE_
            << ", nu==mdof+numEE*adof " << (nActuators == mdof_ + numEE_ * adof_ ? "OK" : "FAIL"));

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

  initJointSensors();
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

void MujocoActuator::initJointSensors() {
  jointPosSensorAdr_.resize(mdof_, -1);
  jointVelSensorAdr_.resize(mdof_, -1);
  jointFrcSensorAdr_.resize(mdof_, -1);

  for (int i = 0; i < mj_.mjModel_->nsensor; i++) {
    int type = mj_.mjModel_->sensor_type[i];
    int adr = mj_.mjModel_->sensor_adr[i];
    int objid = mj_.mjModel_->sensor_objid[i];

    if (type == mjSENS_JOINTPOS) {
      // Match sensor's joint ID to our joint actuators
      for (int j = 0; j < mdof_; j++) {
        int actuatorIdx = jointIndices_[j];
        int jntId = mj_.mjModel_->actuator_trnid[2 * actuatorIdx];
        if (objid == jntId) {
          jointPosSensorAdr_[j] = adr;
          break;
        }
      }
    } else if (type == mjSENS_JOINTVEL) {
      for (int j = 0; j < mdof_; j++) {
        int actuatorIdx = jointIndices_[j];
        int jntId = mj_.mjModel_->actuator_trnid[2 * actuatorIdx];
        if (objid == jntId) {
          jointVelSensorAdr_[j] = adr;
          break;
        }
      }
    } else if (type == mjSENS_ACTUATORFRC) {
      // actuatorfrc sensor's objid is actuator ID directly
      for (int j = 0; j < mdof_; j++) {
        if (objid == jointIndices_[j]) {
          jointFrcSensorAdr_[j] = adr;
          break;
        }
      }
    }
  }

  // Validate sensor discovery and report status
  int foundPos = 0, foundVel = 0, foundFrc = 0;
  for (int j = 0; j < mdof_; j++) {
    if (jointPosSensorAdr_[j] >= 0) foundPos++;
    if (jointVelSensorAdr_[j] >= 0) foundVel++;
    if (jointFrcSensorAdr_[j] >= 0) foundFrc++;
  }

  bool allFound = (foundPos == mdof_ && foundVel == mdof_ && foundFrc == mdof_);

  if (useSensorFeedback_) {
    if (allFound) {
      DEBUG_LOG("Sensor feedback enabled: " << mdof_ << " joints with pos/vel/frc sensors");
      for (int j = 0; j < mdof_; j++) {
        DEBUG_LOG("  Joint " << j << ": sensordata[pos=" << jointPosSensorAdr_[j]
                  << ", vel=" << jointVelSensorAdr_[j]
                  << ", frc=" << jointFrcSensorAdr_[j] << "]");
      }
    } else {
      std::cerr << "[MujocoActuator] Warning: Not all joint sensors found (pos:" << foundPos
                << "/" << mdof_ << ", vel:" << foundVel << "/" << mdof_
                << ", frc:" << foundFrc << "/" << mdof_ << "), falling back to direct state" << std::endl;
      useSensorFeedback_ = false;
    }
  } else {
    if (utils::DebugConfig::getInstance().isVerbose()) {
      std::cout << "[MujocoActuator] Sensor feedback disabled, using direct state. "
                << "Found sensors: pos:" << foundPos << "/" << mdof_
                << ", vel:" << foundVel << "/" << mdof_
                << ", frc:" << foundFrc << "/" << mdof_ << std::endl;
    }
  }
}

void MujocoActuator::update() {
  updateJointsFeedback();
  updateEEFeedback();
}

void MujocoActuator::updateJointsFeedbackFromSensors() {
  auto &ds = mj_.runtimeData_;

  for (int i = 0; i < mdof_; ++i) {
    ds.qFb[i] = mj_.mjData_->sensordata[jointPosSensorAdr_[i]];
    ds.qdFb[i] = mj_.mjData_->sensordata[jointVelSensorAdr_[i]];
    ds.qtauFb[i] = mj_.mjData_->sensordata[jointFrcSensorAdr_[i]];
  }

  // Debug: Compare sensor vs ground truth (print once per second)
  static double lastPrintTime = -1.0;
  if (mj_.mjData_->time - lastPrintTime > 1.0) {
    lastPrintTime = mj_.mjData_->time;
    int actuatorIdx = jointIndices_[0];
    int jntAdr = mj_.mjModel_->actuator_trnid[2 * actuatorIdx];
    double sensorPos = mj_.mjData_->sensordata[jointPosSensorAdr_[0]];
    double truePos = mj_.mjData_->qpos[jntAdr];
    double diff = sensorPos - truePos;
    std::cout << "[SensorFB] Joint0: sensor=" << std::fixed << std::setprecision(4) << sensorPos
              << ", true=" << truePos << ", diff=" << diff << " rad" << std::endl;
  }
}

void MujocoActuator::updateJointsFeedback() {
  if (useSensorFeedback_) {
    updateJointsFeedbackFromSensors();
    return;
  }

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

  DEBUG_LOG("Actuator mapping validation passed");
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
