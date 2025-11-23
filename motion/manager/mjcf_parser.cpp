#include "mjcf_parser.hpp"

#ifdef __APPLE__
#include "mujoco.h"
#else
#include "mujoco/mujoco.h"
#endif

#include <algorithm>
#include <cctype>
#include <iostream>

#include "debug_config.hpp"

namespace rynn {

MjcfParser::MjcfParser(RobotManager &rm) : rm_(rm), robotModel_(nullptr), pinoModel_(nullptr) {
  parseAndPopulate();
}

MjcfParser::~MjcfParser() {
  if (pinoModel_ && pinoModel_ != robotModel_) {
    mj_deleteModel(pinoModel_);
    pinoModel_ = nullptr;
  }

  if (robotModel_) {
    mj_deleteModel(robotModel_);
    robotModel_ = nullptr;
  }
}

void MjcfParser::parseAndPopulate() {
  // Load robotMJCF (*_robot.xml)
  robotModel_ = mj_loadXML(rm_.getRobotMJCF().c_str(), 0, nullptr, 0);
  if (!robotModel_) return;

  // Load pinoMJCF (*_pinocchio.xml) or fallback to robotModel_
  pinoModel_ = mj_loadXML(rm_.getPinoMJCF().c_str(), 0, nullptr, 0);
  if (!pinoModel_) pinoModel_ = robotModel_;

  parseRobotModel();
}

void MjcfParser::parseRobotModel() {
  if (!robotModel_ || !pinoModel_) return;

  detectEndEffectors();
  computeEEParentJoints();
  extractJointLimits();
  detectActuatorModes();
  extractActuatorGains();
  extractKeyframes();
  extractSiteNames();
}

void MjcfParser::detectEndEffectors() {
  std::vector<int> jointIndices, eeIndices;
  std::vector<std::pair<double, double>> eeRanges;

  std::vector<std::string> gripperPatterns = {
      "grip",
      "gripper",
      "finger",
      "hand"};

  for (int i = 0; i < robotModel_->nu; i++) {
    bool isGripper = false;

    const char *actuatorName = mj_id2name(robotModel_, mjOBJ_ACTUATOR, i);
    if (!actuatorName) continue;

    std::string name(actuatorName);
    std::string baseName = name;

    size_t underscorePos = name.find_last_of('_');
    if (underscorePos != std::string::npos) {
      std::string suffix = name.substr(underscorePos + 1);
      if (!suffix.empty() && std::all_of(suffix.begin(), suffix.end(), ::isdigit)) {
        baseName = name.substr(0, underscorePos);
      }
    }

    std::string lowerName = baseName;
    std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(), ::tolower);

    for (const auto &pattern : gripperPatterns) {
      if (lowerName == pattern || lowerName.find(pattern) != std::string::npos) {
        isGripper = true;
        break;
      }
    }

    if (robotModel_->actuator_trntype[i] == mjTRN_TENDON) {
      isGripper = true;
    }

    if (isGripper) {
      eeIndices.push_back(i);
      double minRange = robotModel_->actuator_ctrlrange[2 * i];
      double maxRange = robotModel_->actuator_ctrlrange[2 * i + 1];
      eeRanges.emplace_back(minRange, maxRange);

      DEBUG_LOG("Detected end-effector: actuator[" << i << "] = \""
                                                   << name << "\", range=[" << minRange << ", "
                                                   << maxRange << "]");
    } else {
      jointIndices.push_back(i);
    }
  }

  int mdof = jointIndices.size();
  int adof = eeIndices.empty() ? 0 : 1; // Assume 1 DOF per end effector for now

  rm_.setMotionDOF(mdof);
  rm_.setActionDOF(adof);
  rm_.setActuatorIndices(jointIndices, eeIndices);
  rm_.setEndEffectorConfig(adof, {}, eeRanges);
}

void MjcfParser::computeEEParentJoints() {
  std::vector<int> jointIndices = rm_.getJointIndices();
  std::vector<int> eeIndices = rm_.getEEIndices();
  std::vector<int> eeJointIndices;

  DEBUG_LOG("computeEEParentJoints:");
  DEBUG_LOG("  jointIndices (" << jointIndices.size() << "): [");
  for (size_t i = 0; i < jointIndices.size(); ++i) {
    DEBUG_LOG("    [" << i << "] = actuator_id " << jointIndices[i]);
  }
  DEBUG_LOG("  ]");
  DEBUG_LOG("  eeIndices (" << eeIndices.size() << "): [");
  for (size_t i = 0; i < eeIndices.size(); ++i) {
    DEBUG_LOG("    [" << i << "] = actuator_id " << eeIndices[i]);
  }
  DEBUG_LOG("  ]");

  if (eeIndices.empty()) {
    return;
  }

  for (int eeActuatorIdx : eeIndices) {
    int parentJointIdx = -1;
    int parentJointActuatorId = -1;

    for (size_t i = 0; i < jointIndices.size(); ++i) {
      if (jointIndices[i] < eeActuatorIdx) {
        parentJointIdx = static_cast<int>(i);
        parentJointActuatorId = jointIndices[i];
      } else {
        break;
      }
    }

    if (parentJointIdx == -1) {
      std::cerr << "Warning: Could not find parent joint for EE actuator " << eeActuatorIdx << std::endl;
      parentJointIdx = static_cast<int>(jointIndices.size()) - 1; // Default to last joint
      parentJointActuatorId = jointIndices[parentJointIdx];
    }

    eeJointIndices.push_back(parentJointActuatorId);

    DEBUG_LOG("    EE actuator " << eeActuatorIdx << " -> parent joint actuator " << parentJointActuatorId
                                 << " (jointIndices[" << parentJointIdx << "])");
  }

  DEBUG_LOG("  eeJointIndices (" << eeJointIndices.size() << "): [");
  for (size_t i = 0; i < eeJointIndices.size(); ++i) {
    DEBUG_LOG("    [" << i << "] = actuator_id " << eeJointIndices[i]);
  }
  DEBUG_LOG("  ]");

  std::vector<std::pair<double, double>> eeRanges = rm_.getEERanges();
  rm_.setEndEffectorConfig(rm_.getActionDOF(), eeJointIndices, eeRanges);
}

void MjcfParser::extractJointLimits() {
  int mdof = rm_.getMotionDOF();

  DEBUG_LOG("extractJointLimits:");
  DEBUG_LOG("  Motion DOF: " << mdof);

  Eigen::VectorXd qPosMin = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd qPosMax = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd qVelMax = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd qTorqueMax = Eigen::VectorXd::Zero(mdof);

  for (int i = 0; i < mdof; i++) {
    int joint_id = pinoModel_->actuator_trnid[2 * i];
    int joint_type = pinoModel_->actuator_trntype[i];

    if (joint_type == mjTRN_JOINT && joint_id >= 0 && joint_id < pinoModel_->njnt) {
      if (pinoModel_->jnt_limited[joint_id]) {
        qPosMin[i] = pinoModel_->jnt_range[2 * joint_id];
        qPosMax[i] = pinoModel_->jnt_range[2 * joint_id + 1];
      } else {
        qPosMin[i] = -1e6;
        qPosMax[i] = 1e6;
      }
    } else {
      qPosMin[i] = -1e6;
      qPosMax[i] = 1e6;
    }

    if (pinoModel_->actuator_dyntype[i] != mjDYN_NONE) {
      qVelMax[i] = 1e6;
    } else {
      qVelMax[i] = 1e6;
    }

    double ctrlRange = std::max(std::abs(pinoModel_->actuator_ctrlrange[2 * i]),
                                std::abs(pinoModel_->actuator_ctrlrange[2 * i + 1]));
    qTorqueMax[i] = ctrlRange;

    DEBUG_LOG("  joint" << i << " Position: [" << qPosMin[i] << ", " << qPosMax[i] << "]");
    DEBUG_LOG("  joint" << i << " Velocity: " << qVelMax[i]);
    DEBUG_LOG("  joint" << i << " Torque: " << qTorqueMax[i]);
  }

  rm_.setJointLimits(qPosMin, qPosMax, qVelMax, qTorqueMax);
}

void MjcfParser::detectActuatorModes() {
  int nMotor = pinoModel_->nu;
  std::vector<ActuatorMode> actuatorModes(nMotor, ActuatorMode::kGeneral);

  for (int i = 0; i < nMotor; ++i) {
    const int dy = pinoModel_->actuator_dyntype[i];
    const int ga = pinoModel_->actuator_gaintype[i];
    const int bi = pinoModel_->actuator_biastype[i];

    [[maybe_unused]] auto dynParam0 = [this](int id) { return pinoModel_->actuator_dynprm[id * mjNDYN]; };
    auto gainParam0 = [this](int id) { return pinoModel_->actuator_gainprm[id * mjNGAIN]; };
    auto biasParam2 = [this](int id) { return pinoModel_->actuator_biasprm[id * mjNBIAS + 2]; };

    // === Direct torque (motor) =====================================
    if (ga == mjGAIN_FIXED && bi == mjBIAS_NONE && dy == mjDYN_NONE) {
      actuatorModes[i] = ActuatorMode::kTorque;
      continue;
    }

    // === Velocity servo ===========================================
    if (ga == mjGAIN_FIXED && bi == mjBIAS_AFFINE && dy == mjDYN_NONE) {
      double kv = gainParam0(i);
      if (biasParam2(i) == -kv) {
        actuatorModes[i] = ActuatorMode::kVelocity;
        continue;
      }
    }

    // === Position servo ===========================================
    if (ga == mjGAIN_FIXED && bi == mjBIAS_AFFINE && (dy == mjDYN_NONE || dy == mjDYN_FILTEREXACT)) {
      [[maybe_unused]] double kp = gainParam0(i);
      actuatorModes[i] = ActuatorMode::kPosition;
      continue;
    }

    // === Integrated‑velocity (intvelocity) =========================
    if (ga == mjGAIN_FIXED && bi == mjBIAS_AFFINE && dy == mjDYN_INTEGRATOR) {
      actuatorModes[i] = ActuatorMode::kIntVelocity;
      continue;
    }

    // Additional mappings (muscle, damper, cylinder, plugin) ------
    if (ga == mjGAIN_MUSCLE || bi == mjBIAS_MUSCLE || dy == mjDYN_MUSCLE) {
      actuatorModes[i] = ActuatorMode::kMuscle;
      continue;
    }

    actuatorModes[i] = ActuatorMode::kGeneral;
  }

  std::vector<int> positionActuators, velocityActuators, torqueActuators;

  for (int i = 0; i < nMotor; ++i) {
    switch (actuatorModes[i]) {
    case ActuatorMode::kPosition:
      positionActuators.push_back(i);
      break;
    case ActuatorMode::kVelocity:
      velocityActuators.push_back(i);
      break;
    case ActuatorMode::kTorque:
      torqueActuators.push_back(i);
      break;
    default:
      break;
    }
  }

  rm_.setActuatorModes(actuatorModes);
}

void MjcfParser::extractActuatorGains() {
  if (!pinoModel_) {
    std::cerr << "Error: NULL pinoModel_ pointer in extractActuatorGains" << std::endl;
    return;
  }

  int mdof = rm_.getMotionDOF();
  int adof = rm_.getActionDOF();
  int numEE = rm_.getNumEndEffectors();
  int nActuators = mdof + numEE * adof;

  Eigen::VectorXd simKp = Eigen::VectorXd::Zero(nActuators);
  Eigen::VectorXd simKd = Eigen::VectorXd::Zero(nActuators);

  std::vector<ActuatorMode> actuatorModes = rm_.getActuatorModes();

  for (int i = 0; i < nActuators; ++i) {
    const char *actuatorName = mj_id2name(pinoModel_, mjOBJ_ACTUATOR, i);
    std::string name = actuatorName ? actuatorName : "unknown";

    ActuatorMode mode = (i < static_cast<int>(actuatorModes.size())) ? actuatorModes[i] : ActuatorMode::kGeneral;

    switch (mode) {
    case ActuatorMode::kPosition: {
      simKp[i] = pinoModel_->actuator_gainprm[i * mjNGAIN];

      double kv = pinoModel_->actuator_biasprm[i * mjNBIAS + 2];
      if (kv < 0) {
        simKd[i] = -kv;
      } else {
        simKd[i] = simKp[i] * 0.05;
      }
      break;
    }

    case ActuatorMode::kVelocity: {
      simKp[i] = pinoModel_->actuator_gainprm[i * mjNGAIN];
      simKd[i] = 0.0;
      break;
    }

    case ActuatorMode::kTorque: {
      simKp[i] = 1.0;
      simKd[i] = 0.0;
      break;
    }

    default: {
      double gain = pinoModel_->actuator_gainprm[i * mjNGAIN];
      simKp[i] = (gain != 0.0) ? gain : 10.0;
      simKd[i] = simKp[i] * 0.05;
      break;
    }
    }
  }

  rm_._simKp = simKp;
  rm_._simKd = simKd;
}

void MjcfParser::extractKeyframes() {
  mjData *data = mj_makeData(pinoModel_);
  if (!data) {
    std::cerr << "Warning: Failed to create mjData for keyframe extraction" << std::endl;
    return;
  }

  int mdof = rm_.getMotionDOF();

  std::vector<std::string> universalKeyframeNames = {"home", "standby1", "standby2"};

  RobotManager::EigenVecXd keyframesList;
  std::vector<std::string> namesList;

  for (const auto &expectedName : universalKeyframeNames) {
    int key_id = mj_name2id(pinoModel_, mjOBJ_KEY, expectedName.c_str());
    if (key_id != -1) {
      mj_resetDataKeyframe(pinoModel_, data, key_id);
      Eigen::VectorXd qKeyframe = Eigen::VectorXd::Zero(mdof);
      for (int i = 0; i < mdof; i++) {
        qKeyframe[i] = data->qpos[i];
      }
      keyframesList.push_back(qKeyframe);
      namesList.push_back(expectedName);
    }
  }

  for (int key_idx = 0; key_idx < pinoModel_->nkey; ++key_idx) {
    const char *key_name_cstr = mj_id2name(pinoModel_, mjOBJ_KEY, key_idx);
    if (key_name_cstr) {
      std::string key_name(key_name_cstr);

      bool isUniversal = false;
      for (const auto &uName : universalKeyframeNames) {
        if (key_name == uName) {
          isUniversal = true;
          break;
        }
      }

      if (!isUniversal) {
        mj_resetDataKeyframe(pinoModel_, data, key_idx);
        Eigen::VectorXd qKeyframe = Eigen::VectorXd::Zero(mdof);
        for (int i = 0; i < mdof; i++) {
          qKeyframe[i] = data->qpos[i];
        }
        keyframesList.push_back(qKeyframe);
        namesList.push_back(key_name);
      }
    }
  }

  rm_.setKeyframes(keyframesList, namesList);
  mj_deleteData(data);
}

void MjcfParser::extractSiteNames() {
  if (!pinoModel_) {
    std::cerr << "Error: NULL pinoModel_ pointer in extractSiteNames" << std::endl;
    return;
  }

  std::vector<std::string> siteNames;
  siteNames.reserve(pinoModel_->nsite);

  for (int i = 0; i < pinoModel_->nsite; ++i) {
    const char *siteName = mj_id2name(pinoModel_, mjOBJ_SITE, i);
    if (siteName && siteName[0] != '\0') {
      siteNames.push_back(std::string(siteName));
    }
  }

  DEBUG_LOG("Extracted " << siteNames.size() << " site(s) from Pinocchio MJCF");
  for (const auto &name : siteNames) {
    DEBUG_LOG("  - " << name);
  }

  rm_.setSiteNames(siteNames);
}

RobotManager::EigenVecXd MjcfParser::extractKeyframesFromModel(mjModel *model, int mdof) {
  RobotManager::EigenVecXd keyframes;

  if (!model || model->nkey == 0) {
    return keyframes;
  }

  for (int key_id = 0; key_id < model->nkey; ++key_id) {
    Eigen::VectorXd q(mdof);

    for (int j = 0; j < mdof; ++j) {
      q[j] = model->key_qpos[key_id * model->nq + j];
    }

    keyframes.push_back(q);
  }

  return keyframes;
}

} // namespace rynn
