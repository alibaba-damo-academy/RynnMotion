#include "interface_base.hpp"

#include "robot_manager.hpp"

InterfaceBase::InterfaceBase(const YAML::Node &motionYaml,
                             int robotNumber,
                             int sceneNumber) :
    _motionYaml(motionYaml),
    _robotNumber(robotNumber),
    _sceneNumber(sceneNumber) {
  initInterface();
}

void InterfaceBase::initInterface() {
  robotManager = std::make_unique<rynn::RobotManager>(_robotNumber);
  sceneManager = std::make_unique<rynn::SceneManager>(_robotNumber, _sceneNumber);
  fsmManager = std::make_unique<rynn::FsmManager>();

  int mdof = robotManager->getMotionDOF();
  int numEE = robotManager->getNumEndEffectors();
  runtimeData_ = data::RuntimeData(mdof, numEE);

  moduleManager = std::make_unique<rynn::ModuleManager>(_motionYaml,
                                                        *robotManager,
                                                        *sceneManager,
                                                        *fsmManager,
                                                        runtimeData_);
}

void InterfaceBase::callController() {
  getFeedbacks();
  moduleManager->updateAllModules();
  setActuatorCommands();
}

void InterfaceBase::resetController() {
  if (moduleManager) {
    moduleManager->resetAllModules();
  }
}

void InterfaceBase::resetFsm() {
  if (fsmManager) {
    fsmManager->reset();
  }
}

void InterfaceBase::getFeedbacks() {
  getJointFeedbacks();
  getEEFeedbacks();
}

void InterfaceBase::getEEFeedbacks() {
  // Default empty implementation for robots without end-effector state
}

void InterfaceBase::setActuatorCommands() {
  setJointCommands();
  setEECommands();
}

void InterfaceBase::setEECommands() {
  // Default empty implementation for robots without end-effector commands
}
