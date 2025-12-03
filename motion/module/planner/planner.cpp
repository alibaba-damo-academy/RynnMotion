#include "planner.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>

#include "debug_config.hpp"
#include "orient_tools.hpp"

namespace rynn {

CPlanner::CPlanner(const YAML::Node &yamlNode) :
    CModuleBase(yamlNode) {
  loadYaml();
}

void CPlanner::loadYaml() {
  if (_yamlNode["module"]["planner"] && _yamlNode["module"]["planner"]["frequency"]) {
    freq_ = _yamlNode["module"]["planner"]["frequency"].as<double>();
  }
}

void CPlanner::initModule() {
  if (!runtimeData_) {
    throw std::runtime_error("CPlanner: RuntimeData not set");
  }

  _numEE = robotManager->getNumEndEffectors();
  if (_numEE == 0) {
    std::cerr << "Warning: No end-effectors found, defaulting to 1" << std::endl;
    _numEE = 1;
  }

  for (int i = 0; i < _numEE; i++) {
    runtimeData_->addBodyPlanner();
  }

  _circleStates.resize(_numEE);

  if (utils::DebugConfig::getInstance().isVerbose()) {
    std::cout << "[CPlanner] Configured for " << _numEE
              << " end-effector(s)" << std::endl;
  }
}

void CPlanner::update() {
  _t = runtimeData_->simTime;

  if (fsmManager_->isOnEntry()) {
    Eigen::VectorXd qdFb_unused, qtauFb_unused;
    runtimeData_->getJointsFeedback(qInit_, qdFb_unused, qtauFb_unused);

    if (fsmManager_->getCurrentState() == StateID::GoStand1) {
      setStandPos();
    }
  }

  switch (fsmManager_->getCurrentState()) {
  case StateID::Init:
    break;

  case StateID::GoStand1:
    move2Stand();
    break;

  case StateID::Action1:
    eePlanner();
    break;

  case StateID::GoHome:
    move2Home();
    break;

  default:
    move2Stand();
    break;
  }
}

void CPlanner::setStandPos() {
  qStand_ = robotManager->getQStandby();
}

void CPlanner::move2Stand() {
  float progress = std::min(1.0f, fsmManager_->getStateTime());

  int mdof = robotManager->getMotionDOF();
  Eigen::VectorXd qCmd = utils::lerp(qInit_, qStand_, progress);
  Eigen::VectorXd qdCmd = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(mdof);
  runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);
}

void CPlanner::twoLinkPlanner() {
  double circleTime = static_cast<double>(fsmManager_->getStateTime());

  double omega = 2.0 * M_PI * freq_;
  double angle = circleTime * omega;
  double ry = 0.2;
  double rz = 0.1;

  Eigen::Vector3d eePosDes = Eigen::Vector3d::Zero();
  eePosDes.y() = -ry * sin(angle);
  eePosDes.z() = 0.4 + rz - rz * cos(angle);

  Eigen::Vector3d eeVelDes;
  eeVelDes.x() = 0;
  eeVelDes.y() = -ry * cos(angle) * omega;
  eeVelDes.z() = +rz * sin(angle) * omega;

  runtimeData_->bodyPlans[0].pos = eePosDes;
  runtimeData_->bodyPlans[0].velocity.head<3>() = eeVelDes;
}

void CPlanner::eePlanner() {
  _sceneNumber = sceneManager->getSceneNumber();
  _sceneType = sceneManager->getSceneType();

  if (_sceneType == SceneType::kUI) {
    followObject();
    return;
  }

  if (_sceneType == SceneType::kTracking) {
    predefinedMotion();
    return;
  }

  if (_sceneType == SceneType::kPickPlace) {
    pickAndPlace();
    return;
  }

  predefinedMotion();
}

void CPlanner::move2Home() {
  auto robotType = robotManager->getRobotType();

  if (robotType == RobotType::fr3) {
    move2Stand();
    return;
  }

  move2Stand();
}

void CPlanner::predefinedMotion() {
  static double radius = 0.08;
  static double frequency = 0.5;
  static bool initialized = false;

  if (!initialized) {
    for (int i = 0; i < _numEE; i++) {
      pinKine_->update(qStand_);
      _circleStates[i].center = pinKine_->getEEPos(i);
      _circleStates[i].phaseOffset = i * M_PI;
      _circleStates[i].initialized = true;
    }
    initialized = true;
  }

  double omega = 2.0 * M_PI * frequency;

  for (int i = 0; i < _numEE; i++) {
    double theta = omega * (_t - 1.0) + _circleStates[i].phaseOffset;

    Eigen::Vector3d eePosDes;
    eePosDes.x() = _circleStates[i].center.x() + radius * sin(theta);
    eePosDes.y() = _circleStates[i].center.y() - radius * cos(theta);
    eePosDes.z() = _circleStates[i].center.z();

    Eigen::VectorXd eeVelDes = Eigen::VectorXd::Zero(6);
    eeVelDes << radius * omega * cos(theta),
        radius * omega * sin(theta),
        0, 0, 0, 0;

    pinKine_->update(qStand_);
    Eigen::Vector4d eeQuatVec = pinKine_->getEEQuat(i);
    Eigen::Quaterniond eeQuatDes(eeQuatVec(3), eeQuatVec(0), eeQuatVec(1), eeQuatVec(2));

    runtimeData_->bodyPlans[i].pos = eePosDes;
    runtimeData_->bodyPlans[i].quat = eeQuatDes;
    runtimeData_->bodyPlans[i].velocity = eeVelDes;
  }
}

void CPlanner::followObject() {
  auto cubeObject = runtimeData_->getObjectByName("cube");
  if (!cubeObject) {
    std::cerr << "[CPlanner] Error: Cube object not found" << std::endl;
    return;
  }

  for (int i = 0; i < _numEE; i++) {
    runtimeData_->bodyPlans[i].pos = cubeObject->pos;

    Eigen::Vector3d deltaRPY;
    if (_numEE == 1) {
      deltaRPY = Eigen::Vector3d::Zero();
    } else {
      if (i == 0) {
        deltaRPY = Eigen::Vector3d(-M_PI / 2.0, M_PI / 4.0, 0.0);
      } else {
        deltaRPY = Eigen::Vector3d(M_PI / 2.0, -M_PI / 4.0, 0.0);
      }
    }

    Eigen::Quaterniond deltaQuat = utils::EulerZYX2Quaternion(deltaRPY);
    runtimeData_->bodyPlans[i].quat = cubeObject->quat * deltaQuat;
    runtimeData_->bodyPlans[i].velocity = Eigen::VectorXd::Zero(6);
  }

  if (robotManager->getNumEndEffectors() > 0) {
    static std::vector<bool> gripperOpenStates(_numEE, true);

    if (gripperOpenStates.size() != static_cast<size_t>(_numEE)) {
      gripperOpenStates.resize(_numEE, true);
    }

    for (int i = 0; i < _numEE; i++) {
      if (runtimeData_->gripperCommands[i].toggleGripper) {
        gripperOpenStates[i] = !gripperOpenStates[i];
        runtimeData_->gripperCommands[i].toggleGripper = false;
        std::cout << "EE[" << i << "] Gripper toggled: "
                  << (gripperOpenStates[i] ? "OPEN" : "CLOSED") << std::endl;
      }

      if (gripperOpenStates[i]) {
        runtimeData_->openGripper(i);
      } else {
        runtimeData_->closeGripper(i);
      }
    }
  }
}

std::vector<data::Pose> CPlanner::getOfflineGraspPose(const data::Pose &eePose) {
  std::vector<data::Pose> graspPoses;
  Eigen::Quaterniond graspQuat(eePose.quat.toRotationMatrix());

  size_t objectCount = runtimeData_->getObjectCount();
  for (size_t i = 0; i < objectCount; ++i) {
    auto &obj = runtimeData_->objectStates[i];

    if (obj.name.find("cube") == std::string::npos && obj.name.find("Cube") == std::string::npos) {
      continue;
    }

    data::Pose wayPointPose;
    wayPointPose.pos = obj.pos - sceneManager->getOriginOffset();
    wayPointPose.pos.z() = 0.01;
    wayPointPose.quat = graspQuat;
    graspPoses.push_back(wayPointPose);
  }
  return graspPoses;
}

std::pair<std::vector<data::Pose>, std::vector<data::Pose>>
CPlanner::assignCubesToArms(const std::vector<data::Pose> &allCubes,
                            const Eigen::Vector3d &leftArmPos,
                            const Eigen::Vector3d &rightArmPos) {
  std::vector<data::Pose> leftCubes, rightCubes;

  if (allCubes.empty()) {
    return {leftCubes, rightCubes};
  }

  struct CubeDistance {
    data::Pose pose;
    double distToLeft;
    double distToRight;
    size_t index;
  };

  std::vector<CubeDistance> cubesWithDist;
  for (size_t i = 0; i < allCubes.size(); i++) {
    CubeDistance cd;
    cd.pose = allCubes[i];
    cd.distToLeft = (allCubes[i].pos - leftArmPos).norm();
    cd.distToRight = (allCubes[i].pos - rightArmPos).norm();
    cd.index = i;
    cubesWithDist.push_back(cd);
  }

  std::sort(cubesWithDist.begin(), cubesWithDist.end(),
            [](const CubeDistance &a, const CubeDistance &b) {
              return a.distToLeft < b.distToLeft;
            });

  for (size_t i = 0; i < 3 && i < cubesWithDist.size(); i++) {
    leftCubes.push_back(cubesWithDist[i].pose);
  }

  std::sort(cubesWithDist.begin(), cubesWithDist.end(),
            [](const CubeDistance &a, const CubeDistance &b) {
              return a.distToRight < b.distToRight;
            });

  size_t numRight = std::min(size_t(3), cubesWithDist.size());
  for (int i = numRight - 1; i >= 0; i--) {
    rightCubes.push_back(cubesWithDist[i].pose);
  }

  return {leftCubes, rightCubes};
}

void CPlanner::pickAndPlace() {
  if (_numEE == 1) {
    pickAndPlaceSingleArm();
    return;
  }

  if (_numEE != 2) {
    std::cerr << "[CPlanner] Error: Only 1 or 2 end-effectors supported" << std::endl;
    predefinedMotion();
    return;
  }

  bool needsInit = !_pickPlaceControllers[0] && !_pickPlaceControllers[1];

  if (needsInit) {
    data::Pose standPose[2];
    for (int i = 0; i < 2; i++) {
      standPose[i].pos = runtimeData_->bodyStates[i].pos;
      standPose[i].quat = runtimeData_->bodyStates[i].quat;
    }

    auto mugObject = runtimeData_->getObjectByName("mug");
    if (!mugObject) {
      std::cerr << "[CPlanner] Error: Mug object not found" << std::endl;
      return;
    }

    std::vector<data::Pose> allCubePoses = getOfflineGraspPose(standPose[0]);
    if (allCubePoses.empty()) {
      std::cerr << "[CPlanner] Error: No cubes found" << std::endl;
      return;
    }

    auto [leftCubes, rightCubes] = assignCubesToArms(allCubePoses, standPose[0].pos, standPose[1].pos);

    if (leftCubes.empty() || rightCubes.empty()) {
      std::cerr << "[CPlanner] Warning: Uneven cube distribution - "
                << leftCubes.size() << " left, " << rightCubes.size() << " right" << std::endl;
    }

    for (int i = 0; i < 2; i++) {
      auto robotType = robotManager->getRobotType();
      _pickPlaceControllers[i] = std::make_unique<PickPlaceController>(robotType, i);

      auto mugObject = runtimeData_->getObjectByName("mug");
      if (!mugObject) {
        std::cerr << "[CPlanner] Error: mug object not found" << std::endl;
        return;
      }

      data::Pose dropPose;
      dropPose.pos = mugObject->pos;
      if (i == 1) {
        dropPose.pos.y() = -dropPose.pos.y();
      }

      dropPose.pos.z() += _pickPlaceControllers[i]->getConfig().dropHeight;
      dropPose.quat = standPose[i].quat;

      const auto &cubesForArm = (i == 0) ? leftCubes : rightCubes;

      if (!_pickPlaceControllers[i]->initialize(standPose[i], dropPose, cubesForArm)) {
        std::cerr << "[CPlanner] Error: Failed to initialize controller for arm "
                  << i << std::endl;
        _pickPlaceControllers[i].reset();
        return;
      }

      _lastPickPlaceUpdateTime[i] = runtimeData_->simTime;
      _pickPlaceInitialized[i] = true;
    }

    return;
  }

  double currentTime = runtimeData_->simTime;

  bool allCompleted = true;
  for (int i = 0; i < 2; i++) {
    if (!_pickPlaceControllers[i]) continue;

    double dt = currentTime - _lastPickPlaceUpdateTime[i];
    _lastPickPlaceUpdateTime[i] = currentTime;

    auto cmd = _pickPlaceControllers[i]->update(dt);

    runtimeData_->bodyPlans[i].pos = cmd.targetPose.pos;
    runtimeData_->bodyPlans[i].quat = cmd.targetPose.quat;
    runtimeData_->bodyPlans[i].velocity = Eigen::VectorXd::Zero(6);

    if (cmd.openGripper) {
      runtimeData_->openGripper(i);
    } else {
      runtimeData_->closeGripper(i);
    }

    if (!cmd.isCompleted) {
      allCompleted = false;
    }
  }

  static int lastPrintTime = 0;
  int currentSecond = static_cast<int>(currentTime);
  if (currentSecond > lastPrintTime) {
    lastPrintTime = currentSecond;
  }

  if (allCompleted) {
    static bool printedOnce = false;
    if (!printedOnce) {
      std::cout << "[CPlanner] Dual-arm pick-and-place sequence completed!" << std::endl;
      printedOnce = true;
    }
  }
}

void CPlanner::pickAndPlaceSingleArm() {
  if (!_pickPlaceControllers[0]) {
    auto robotType = robotManager->getRobotType();
    _pickPlaceControllers[0] = std::make_unique<PickPlaceController>(robotType);

    pinKine_->update(qStand_);
    data::Pose standPose;
    standPose.pos = pinKine_->getEEPos(0);
    standPose.quat = pinKine_->getEEQuat(0);

    auto mugObject = runtimeData_->getObjectByName("mug");
    if (!mugObject) {
      std::cerr << "[CPlanner] Error: Mug object not found" << std::endl;
      return;
    }
    data::Pose dropPose;
    dropPose.pos = mugObject->pos - sceneManager->getOriginOffset();
    dropPose.pos.z() += _pickPlaceControllers[0]->getConfig().dropHeight;
    dropPose.quat = standPose.quat;

    std::vector<data::Pose> objectPoses = getOfflineGraspPose(standPose);

    if (!_pickPlaceControllers[0]->initialize(standPose, dropPose, objectPoses)) {
      std::cerr << "[CPlanner] Error: Failed to initialize PickPlaceController" << std::endl;
      _pickPlaceControllers[0].reset();
      return;
    }
    _lastPickPlaceUpdateTime[0] = runtimeData_->simTime;
    return;
  }

  double currentTime = runtimeData_->simTime;
  double dt = currentTime - _lastPickPlaceUpdateTime[0];
  _lastPickPlaceUpdateTime[0] = currentTime;

  auto cmd = _pickPlaceControllers[0]->update(dt);

  runtimeData_->bodyPlans[0].pos = cmd.targetPose.pos;
  runtimeData_->bodyPlans[0].quat = cmd.targetPose.quat;
  runtimeData_->bodyPlans[0].velocity = Eigen::VectorXd::Zero(6);

  if (cmd.openGripper) {
    runtimeData_->openGripper(0);
  } else {
    runtimeData_->closeGripper(0);
  }

  if (cmd.isCompleted) {
    static bool printedOnce = false;
    if (!printedOnce) {
      std::cout << "PickPlace sequence completed!" << std::endl;
      printedOnce = true;
    }
  }
}

bool CPlanner::resetModule() {
  for (int i = 0; i < 2; i++) {
    if (_pickPlaceControllers[i]) {
      _pickPlaceControllers[i]->reset();
      _pickPlaceControllers[i].reset();
    }
    _lastPickPlaceUpdateTime[i] = 0.0;
    _pickPlaceInitialized[i] = false;
  }
  return true;
}

} // namespace rynn
