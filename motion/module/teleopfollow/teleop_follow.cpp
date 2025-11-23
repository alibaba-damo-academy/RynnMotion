#include "teleop_follow.hpp"

#include <cmath>
#include <iostream>

namespace rynn {

TeleopFollow::TeleopFollow(const YAML::Node &yamlNode) :
    CModuleBase(yamlNode) {
  loadYaml();
}

void TeleopFollow::loadYaml() {
  YAML::Node spaceMoveYaml = _yamlNode["module"]["spaceMove"];
  YAML::Node plannerYaml = _yamlNode["module"]["planner"];

  _kpTaskSpace = spaceMoveYaml["kpTask"].as<double>();
  _kdTaskSpace = spaceMoveYaml["kdTask"].as<double>();
  _dt = getDt();
}

void TeleopFollow::initModule() {
  if (!runtimeData_) {
    throw std::runtime_error("TeleopFollow: RuntimeData not set");
  }
  if (runtimeData_->bodyPlans.size() == 0) {
    runtimeData_->addBodyPlanner();
  }

  _pinKine = std::make_unique<utils::PinKine>(robotManager->getPinoMJCF(), "EE");
}

void TeleopFollow::update() {
  _t = runtimeData_->simTime;

  _taskTeleopFollow();
}

Eigen::VectorXd TeleopFollow::_setupTaskPD(const Eigen::VectorXd &taskSpaceError) {
  Eigen::VectorXd kp = Eigen::VectorXd::Constant(6, _kpTaskSpace);
  Eigen::MatrixXd kpMat = kp.asDiagonal();
  return kpMat * taskSpaceError;
}

void TeleopFollow::_taskTeleopFollow() {
  int mdof = robotManager->getMotionDOF();
  Eigen::VectorXd qCmd = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(mdof);
  Eigen::Quaterniond identity_quat = Eigen::Quaterniond::Identity();
  qCmd = _pinKine->ikPos(runtimeData_->bodyPlans[0].pos, identity_quat, q_ref);

  // Set joint commands using batch API
  Eigen::VectorXd qdCmd = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(mdof);
  runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);
}
} // namespace rynn
