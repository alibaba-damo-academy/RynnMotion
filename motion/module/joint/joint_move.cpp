#include "joint_move.hpp"

#include <iostream>

namespace rynn {

CJointMove::CJointMove(const YAML::Node &yamlNode) :
    CModuleBase(yamlNode),
    duration_(1.0),
    amp_(M_PI) {
}

void CJointMove::loadYaml() {
}

void CJointMove::initModule() {
  // Create trajectory generators based on robot type
  auto robotType = robotManager->getRobotType();

  if (robotType == RobotType::onelink) {
    initSingleActuator();
  }
}

void CJointMove::update() {
  double time = runtimeData_->simTime;
  auto robotType = robotManager->getRobotType();
  _sceneType = sceneManager->getSceneType();

  // Capture initial joint position on FSM entry (needs sensor feedback)
  if (fsmManager_->isOnEntry()) {
    Eigen::VectorXd qFb, qdFb, qtauFb;
    runtimeData_->getJointsFeedback(qFb, qdFb, qtauFb);
    _qFb0 = qFb;
  }

  if (_sceneType == SceneType::kKeyframeCycle) {
    keyframeWobbling(time);
    return;
  }
  switch (robotType) {
  case RobotType::onelink:
    oneLinkMotion(time);
    break;

  case RobotType::diffcar:
  case RobotType::diffmobile:
  case RobotType::mobile_fr3:
    break;

  default:
    generalJointMove(time);
    break;
  }
}

void CJointMove::oneLinkMotion(double time) {
  Eigen::VectorXd state = source_[0]->getOutput(time);

  Eigen::VectorXd singleJointPos(1), singleJointVel(1), singleJointAcc(1);
  singleJointPos << state(0);
  singleJointVel << state(1);
  singleJointAcc << state(2);

  Eigen::VectorXd jointTorque = pinDynamics->computeID(singleJointPos, singleJointVel, singleJointAcc);

  Eigen::VectorXd qCmd(1), qdCmd(1), qtauCmd(1);
  qCmd << state(0);
  qdCmd << state(1);
  qtauCmd << jointTorque(0);
  runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);
}

void CJointMove::generalJointMove(double time) {
  int mdof = robotManager->getMotionDOF();

  Eigen::VectorXd qPosMin = robotManager->getJointPosMin();
  Eigen::VectorXd qPosMax = robotManager->getJointPosMax();
  Eigen::VectorXd qStandby = robotManager->getQStandby();

  double fbToStandDuration = 1.0;
  double limitTestDuration = 2.0;

  Eigen::VectorXd qCmd = qStandby;
  Eigen::VectorXd qdCmd = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(mdof);

  if (time < fbToStandDuration) {
    double t = time / fbToStandDuration;
    qCmd = utils::lerp(_qFb0, qStandby, t);
  } else {
    double jointTestTime = time - fbToStandDuration;

    for (int i = 0; i < mdof; ++i) {
      double jointStartTime = i * limitTestDuration;
      double jointEndTime = (i + 1) * limitTestDuration;

      if (jointTestTime >= jointStartTime && jointTestTime < jointEndTime) {
        double jointPhaseTime = jointTestTime - jointStartTime;
        double phaseProgress = jointPhaseTime / limitTestDuration;

        if (phaseProgress < 0.25) {
          double t = phaseProgress / 0.25;
          qCmd(i) = utils::lerp(qStandby[i], qPosMin[i], t);
        } else if (phaseProgress < 0.75) {
          double t = (phaseProgress - 0.25) / 0.5;
          qCmd(i) = utils::lerp(qPosMin[i], qPosMax[i], t);
        } else {
          double t = (phaseProgress - 0.75) / 0.25;
          qCmd(i) = utils::lerp(qPosMax[i], qStandby[i], t);
        }
        break;
      }
    }
  }

  runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);

  int numEE = robotManager->getNumEndEffectors();
  if (numEE > 0) {
    double gripperCycleTime = 4.0;
    double gripperPhase = fmod(time, gripperCycleTime);

    for (int i = 0; i < numEE; ++i) {
      if (gripperPhase < 2.0) {
        runtimeData_->openGripper(i);
      } else {
        runtimeData_->closeGripper(i);
      }
    }
  }
}

void CJointMove::keyframeWobbling(double time) {
  int mdof = robotManager->getMotionDOF();
  auto keyframes = robotManager->getAllKeyframes();

  if (keyframes.empty()) {
    return;
  }

  int numKeyframes = keyframes.size();
  double totalCycleDuration = duration_ * 2 * numKeyframes;

  if (time >= totalCycleDuration) {
    Eigen::VectorXd qCmd = keyframes[0];
    Eigen::VectorXd qdCmd = Eigen::VectorXd::Zero(mdof);
    Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(mdof);
    runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);
    return;
  }

  double phaseTime = fmod(time, duration_ * 2);
  int transIndex = static_cast<int>(time / (duration_ * 2));
  int index = transIndex % numKeyframes;
  int nextIndex = (index + 1) % numKeyframes;
  Eigen::VectorXd qCmd;

  if (phaseTime < duration_) {
    qCmd = keyframes[index];
  } else {
    double t = (phaseTime - duration_) / duration_;
    qCmd = utils::lerp(keyframes[index], keyframes[nextIndex], t);
  }

  Eigen::VectorXd qdCmd = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(mdof);
  runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);
}

void CJointMove::initSingleActuator() {
  int mdof = robotManager->getMotionDOF();
  pinDynamics = std::make_shared<utils::PinDynamics>(robotManager->getRobotMJCF(), false);

  source_.reserve(mdof);
  for (int i = 0; i < mdof; i++) {
    source_.emplace_back(std::make_unique<utils::CubicTrajectory>(duration_, amp_));
  }
}

} // namespace rynn
