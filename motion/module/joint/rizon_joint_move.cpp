#include "rizon_joint_move.hpp"

#include <cmath>
#include <iostream>

namespace rynn {

RizonJointMove::RizonJointMove(const YAML::Node &yamlNode) :
    CJointMove(yamlNode) {
}

void RizonJointMove::loadYaml() {
  CJointMove::loadYaml();

  if (_yamlNode["module"] && _yamlNode["module"]["RizonJointMove"]) {
    YAML::Node config = _yamlNode["module"]["RizonJointMove"];

    if (config["hold_mode"]) {
      holdMode_ = config["hold_mode"].as<bool>();
    }
    if (config["sine_amplitude"]) {
      sineAmp_ = config["sine_amplitude"].as<double>();
    }
    if (config["sine_frequency"]) {
      sineFreq_ = config["sine_frequency"].as<double>();
    }
  }

  std::cout << "[RizonJointMove] mode=" << (holdMode_ ? "hold" : "sine-sweep")
            << ", amp=" << sineAmp_ << "rad, freq=" << sineFreq_ << "Hz" << std::endl;
}

void RizonJointMove::initModule() {
  CJointMove::initModule();

  int mdof = robotManager->getMotionDOF();
  initPos_.resize(mdof);
  initPos_.setZero();

  std::cout << "[RizonJointMove] initialized for " << mdof << " DOF robot" << std::endl;
}

void RizonJointMove::update() {
  double time = runtimeData_->simTime;
  int mdof = robotManager->getMotionDOF();

  if (!initPosCaptured_ || fsmManager_->isOnEntry()) {
    Eigen::VectorXd qFb, qdFb, qtauFb;
    runtimeData_->getJointsFeedback(qFb, qdFb, qtauFb);
    initPos_ = qFb;
    initPosCaptured_ = true;
  }

  if (holdMode_) {
    holdPosition();
  } else {
    sineSweep(time);
  }
}

void RizonJointMove::holdPosition() {
  int mdof = robotManager->getMotionDOF();

  Eigen::VectorXd qCmd = initPos_;
  Eigen::VectorXd qdCmd = Eigen::VectorXd::Zero(mdof);
  Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(mdof);

  runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);
}

void RizonJointMove::sineSweep(double time) {
  int mdof = robotManager->getMotionDOF();

  double sineValue = sineAmp_ * std::sin(2.0 * M_PI * sineFreq_ * time);
  double cosValue = sineAmp_ * 2.0 * M_PI * sineFreq_ * std::cos(2.0 * M_PI * sineFreq_ * time);

  Eigen::VectorXd qCmd = initPos_.array() + sineValue;
  Eigen::VectorXd qdCmd = Eigen::VectorXd::Constant(mdof, cosValue);
  Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(mdof);

  runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);
}

} // namespace rynn
