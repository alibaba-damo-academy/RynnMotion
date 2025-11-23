#include "estimator.hpp"

#include <iostream>

#include "motion_fsm.hpp"

namespace rynn {

CEstimator::CEstimator(const YAML::Node &yamlNode) :
    CModuleBase(yamlNode) {
  MotionFsm::start();
}

void CEstimator::loadYaml() {
}

void CEstimator::_stateEstimation() {
  _pinEstimator();
}

void CEstimator::_pinEstimator() {
  runtimeData_->getJointsFeedback(_qFb, _qdFb, _qtauFb);
  pinKine_->update(_qFb, _qdFb);

  int numEE = robotManager->getNumEndEffectors();
  if (numEE == 0) {
    numEE = 1;
  }

  for (int i = 0; i < numEE; i++) {
    runtimeData_->bodyStates[i].pos = pinKine_->getEEPos(i);
    runtimeData_->bodyStates[i].quat = pinKine_->getEEQuat(i);
    runtimeData_->bodyStates[i].velocity = pinKine_->getEEVel(i);
    runtimeData_->jacobians[i].jaco = pinKine_->getEEJaco(i);
  }
}

void CEstimator::initModule() {
  if (!runtimeData_) {
    throw std::runtime_error("CEstimator: RuntimeData not set");
  }

  int numEE = robotManager->getNumEndEffectors();
  if (numEE == 0) {
    std::cerr << "Warning: No end-effectors found, defaulting to 1" << std::endl;
    numEE = 1;
  }

  for (int i = 0; i < numEE; i++) {
    std::string eeName = "EE_" + std::to_string(i);
    runtimeData_->addBodyFeedback(eeName);
    runtimeData_->addJacobian(eeName);
  }

  // std::cout << "[CEstimator] Configured for " << numEE << " end-effector(s)" << std::endl;

  int mdof = robotManager->getMotionDOF();
  _qFb = Eigen::VectorXd::Zero(mdof);
  _qdFb = Eigen::VectorXd::Zero(mdof);
  _qtauFb = Eigen::VectorXd::Zero(mdof);
  _eeT = Eigen::Matrix4d::Identity();
  _eeJaco = Eigen::MatrixXd::Zero(6, mdof);
}

void CEstimator::update() {
  _stateEstimation();
  _updateFsm();
}

void CEstimator::_updateFsm() {
  double simTime = runtimeData_->simTime;
  static double lastFsmTickTime = 0.0;

  float dt = static_cast<float>(simTime - lastFsmTickTime);
  lastFsmTickTime = simTime;

  fsmManager_->tick(dt);
}
} // namespace rynn
