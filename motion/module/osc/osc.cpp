#include "osc.hpp"

#include <cmath>
#include <iostream>

namespace rynn {

OSC::OSC(const YAML::Node &yamlNode) :
    CModuleBase(yamlNode) {
  loadYaml();
}

void OSC::loadYaml() {
  YAML::Node spaceMoveYaml = _yamlNode["module"]["spaceMove"];

  _kpTaskSpace = spaceMoveYaml["kpTask"].as<double>();
  _kdTaskSpace = spaceMoveYaml["kdTask"].as<double>();

  std::string solverStr = spaceMoveYaml["solver"].as<std::string>("qp");

  if (solverStr == "pinv") {
    _solver = IKSolver::kPseudoInverse;
    std::cout << "[OSC] Using PseudoInverse solver" << std::endl;

  } else if (solverStr == "qp") {
    _solver = IKSolver::kDiffQP;

    if (spaceMoveYaml["constraints"]) {
      auto conYaml = spaceMoveYaml["constraints"];
      _diffIKCfg.enableVelLimits = conYaml["vel_limits"].as<bool>(true);
      _diffIKCfg.enablePosLimits = conYaml["pos_limits"].as<bool>(false);
      _diffIKCfg.enableAccLimits = conYaml["acc_limits"].as<bool>(false);
      _diffIKCfg.enableNullSpace = conYaml["null_space"].as<bool>(false);
    }

    _diffIKCfg.damping = spaceMoveYaml["damping"].as<double>(0.0009);
    _diffIKCfg.nullLambda = spaceMoveYaml["null_lambda"].as<double>(0.01);
    _diffIKCfg.nullKp = spaceMoveYaml["null_kp"].as<double>(0.1);
    _diffIKCfg.maxIter = spaceMoveYaml["max_iter"].as<int>(30);

    std::cout << "[OSC] Using DiffQP solver with constraints: "
              << (_diffIKCfg.enableVelLimits ? "Vel " : "")
              << (_diffIKCfg.enablePosLimits ? "Pos " : "")
              << (_diffIKCfg.enableAccLimits ? "Acc " : "")
              << (_diffIKCfg.enableNullSpace ? "Null" : "")
              << std::endl;

  } else {
    throw std::runtime_error("Unsupported IK solver: " + solverStr);
  }
}

void OSC::initModule() {
  if (!runtimeData_) {
    throw std::runtime_error("OSC: RuntimeData not set");
  }

  _numEE = robotManager->getNumEndEffectors();
  if (_numEE == 0) {
    std::cerr << "Warning: No end-effectors found, defaulting to 1" << std::endl;
    _numEE = 1;
  }

  _eeStates.resize(_numEE);

  std::cout << "[OSC] Configured for " << _numEE
            << " end-effector(s)" << std::endl;

  int mdof = robotManager->getMotionDOF();
  if (mdof == 0) return;

  _qFb = Eigen::VectorXd::Zero(mdof);
  _qdFb = Eigen::VectorXd::Zero(mdof);
  _qCmd = Eigen::VectorXd::Zero(mdof);
  _qdCmd = Eigen::VectorXd::Zero(mdof);
  _eeT_plan = Eigen::Matrix4d::Identity();
  _eeT_fb = Eigen::Matrix4d::Identity();
  _eeVel_des = Eigen::VectorXd::Zero(6);
  _eeVel_fb = Eigen::VectorXd::Zero(6);
  _eeJacoFb = Eigen::MatrixXd::Zero(6, mdof);
  _qTask = Eigen::VectorXd::Zero(mdof);

  for (int i = 0; i < _numEE; i++) {
    _eeStates[i].eeVel_des = Eigen::VectorXd::Zero(6);
    _eeStates[i].eeJacoFb = Eigen::MatrixXd::Zero(6, mdof);

    if (_solver == IKSolver::kDiffQP) {
      utils::DiffIKConfig cfg = _diffIKCfg;
      cfg.qdMin = robotManager->getJointVelMax() * -1.0;
      cfg.qdMax = robotManager->getJointVelMax();
      cfg.qMin = robotManager->getJointPosMin();
      cfg.qMax = robotManager->getJointPosMax();
      cfg.qddMin = Eigen::VectorXd::Constant(mdof, -40.0 * M_PI);
      cfg.qddMax = Eigen::VectorXd::Constant(mdof, +40.0 * M_PI);

      _eeStates[i].dIKqp = std::make_unique<utils::DiffIKQP>(mdof, cfg);
    }
  }

  if (_solver == IKSolver::kDiffQP && !_dIKqp) {
    _initQP();
  }
}

void OSC::update() {
  _t = runtimeData_->simTime;

  if (fsmManager_->getCurrentState() == StateID::Action1) {
    _taskSpaceMove();
  }
}

Eigen::VectorXd OSC::_setupTaskPD(const Eigen::VectorXd &taskSpaceError) {
  Eigen::VectorXd kp = Eigen::VectorXd::Constant(6, _kpTaskSpace);
  Eigen::MatrixXd kpMat = kp.asDiagonal();
  return kpMat * taskSpaceError;
}

void OSC::_initQP() {
  int mdof = robotManager->getMotionDOF();

  _diffIKCfg.qdMin = -robotManager->getJointVelMax();
  _diffIKCfg.qdMax = robotManager->getJointVelMax();
  _diffIKCfg.qMin = robotManager->getJointPosMin();
  _diffIKCfg.qMax = robotManager->getJointPosMax();

  _diffIKCfg.qddMin = Eigen::VectorXd::Constant(mdof, -40.0 * M_PI);
  _diffIKCfg.qddMax = Eigen::VectorXd::Constant(mdof, +40.0 * M_PI);

  _dIKqp = std::make_unique<utils::DiffIKQP>(mdof, _diffIKCfg);
}

void OSC::_taskSpaceMove() {
  int mdof = robotManager->getMotionDOF();

  Eigen::VectorXd qtauFb_unused;
  runtimeData_->getJointsFeedback(_qFb, _qdFb, qtauFb_unused);

  if (_numEE > 1) {
    _solveDecoupledIK();
    _mergeJointCommands();
  } else {
    if (!_dIKqp && _solver == IKSolver::kDiffQP) {
      _initQP();
    }

    _eeJacoFb = runtimeData_->jacobians[0].jaco.leftCols(mdof);

    Eigen::Vector3d posErr = runtimeData_->bodyPlans[0].pos - runtimeData_->bodyStates[0].pos;
    Eigen::Quaterniond qErr = runtimeData_->bodyPlans[0].quat * runtimeData_->bodyStates[0].quat.inverse();
    Eigen::AngleAxisd aaErr(qErr);
    Eigen::Vector3d ornErr = aaErr.angle() * aaErr.axis();

    Eigen::VectorXd err(6);
    err.head<3>() = posErr;
    err.tail<3>() = ornErr;

    if (getDt() <= 1e-10) {
      _eeVel_des.setZero();
    } else {
      _eeVel_des = err / getDt();
    }

    switch (_solver) {
    case IKSolver::kPseudoInverse: {
      Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(_eeJacoFb);
      Eigen::MatrixXd Jpinv = cod.pseudoInverse();
      _qdCmd = Jpinv * _eeVel_des;
      _qCmd = _qFb + _qdCmd * getDt();
      break;
    }

    case IKSolver::kDiffQP:
      if (_dIKqp->solve(_eeJacoFb, _eeVel_des, _qFb, _qdFb, getDt())) {
        _qdCmd = _dIKqp->getSolution();
        _qCmd = _qFb + _qdCmd * getDt();
      } else {
        std::cerr << "[OSC] DiffIK solve failed" << std::endl;
      }
      break;

    default:
      throw std::runtime_error("Unsupported IK solver in OSC::_taskSpaceMove()");
    }
  }

  Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(mdof);
  runtimeData_->setJointsCommand(_qCmd, _qdCmd, qtauCmd);
}

void OSC::_solveDecoupledIK() {
  _qdCmdPerEE.resize(_numEE);

  for (int i = 0; i < _numEE; i++) {
    int mdof = robotManager->getMotionDOF();

    _eeStates[i].eeJacoFb = runtimeData_->jacobians[i].jaco.leftCols(mdof);

    Eigen::Vector3d posErr = runtimeData_->bodyPlans[i].pos - runtimeData_->bodyStates[i].pos;
    Eigen::Quaterniond qErr = runtimeData_->bodyPlans[i].quat * runtimeData_->bodyStates[i].quat.inverse();
    Eigen::AngleAxisd aaErr(qErr);
    Eigen::Vector3d ornErr = aaErr.angle() * aaErr.axis();

    Eigen::VectorXd err(6);
    err.head<3>() = posErr;
    err.tail<3>() = ornErr;

    if (getDt() <= 1e-10) {
      _eeStates[i].eeVel_des.setZero();
    } else {
      _eeStates[i].eeVel_des = err / getDt();
    }

    switch (_solver) {
    case IKSolver::kPseudoInverse: {
      Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(_eeStates[i].eeJacoFb);
      Eigen::MatrixXd Jpinv = cod.pseudoInverse();
      _qdCmdPerEE[i] = Jpinv * _eeStates[i].eeVel_des;
      break;
    }

    case IKSolver::kDiffQP:
      if (_eeStates[i].dIKqp && _eeStates[i].dIKqp->solve(_eeStates[i].eeJacoFb, _eeStates[i].eeVel_des, _qFb, _qdFb, getDt())) {
        _qdCmdPerEE[i] = _eeStates[i].dIKqp->getSolution();
      } else {
        std::cerr << "[OSC] DiffIK solve failed for EE[" << i << "]" << std::endl;
        _qdCmdPerEE[i] = Eigen::VectorXd::Zero(_qFb.size());
      }
      break;

    default:
      throw std::runtime_error("Unsupported IK solver in OSC");
    }
  }
}

void OSC::_mergeJointCommands() {
  int mdof = robotManager->getMotionDOF();

  if (_numEE == 1) {
    _qdCmd = _qdCmdPerEE[0];
    _qCmd = _qFb + _qdCmd * getDt();
  } else {
    Eigen::VectorXd weights = Eigen::VectorXd::Constant(_numEE, 1.0 / _numEE);

    _qdCmd = Eigen::VectorXd::Zero(mdof);
    for (int i = 0; i < _numEE; i++) {
      _qdCmd += weights(i) * _qdCmdPerEE[i];
    }

    _qCmd = _qFb + _qdCmd * getDt();
  }
}

} // namespace rynn
