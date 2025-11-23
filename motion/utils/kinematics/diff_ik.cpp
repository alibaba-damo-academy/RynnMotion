#include <iostream>

#include "diff_ik.hpp"

namespace utils {

DiffIKQP::DiffIKQP(int numDOF, const DiffIKConfig &cfg) :
    _cfg(cfg),
    _numDOF(numDOF),
    _qpNWSR(cfg.maxIter),
    _qdCmd(Eigen::VectorXd::Zero(numDOF)),
    _qdCmdPrev(Eigen::VectorXd::Zero(numDOF)) {
  int numC = _computeNumConstraints();
  _sqp = std::make_unique<qpOASES::SQProblem>(numDOF, numC);

  std::cout << "[DiffIKQP] Initialized: DOF=" << numDOF
            << ", Constraints=" << numC << " ("
            << (_cfg.enableVelLimits ? "Vel " : "")
            << (_cfg.enablePosLimits ? "Pos " : "")
            << (_cfg.enableAccLimits ? "Acc " : "")
            << (_cfg.enableNullSpace ? "Null" : "")
            << ")" << std::endl;
}

DiffIKQP::~DiffIKQP() {
}

int DiffIKQP::_computeNumConstraints() const {
  int numC = 0;

  if (_cfg.enablePosLimits) {
    numC += _numDOF;
  }

  if (_cfg.enableAccLimits) {
    numC += _numDOF;
  }

  return numC;
}

void DiffIKQP::_buildCost(const Eigen::MatrixXd &jaco,
                          const Eigen::VectorXd &taskVel,
                          const Eigen::VectorXd &qCur,
                          Eigen::MatrixXd &H,
                          Eigen::VectorXd &g) {
  H = jaco.transpose() * jaco;
  g = -2.0 * jaco.transpose() * taskVel;

  if (_cfg.enableNullSpace && _qTask.size() == _numDOF) {
    Eigen::MatrixXd N = computeNullspace(jaco);
    Eigen::MatrixXd Kp = _cfg.nullKp * Eigen::MatrixXd::Identity(_numDOF, _numDOF);
    Eigen::VectorXd qErr = _qTask - qCur;

    H += _cfg.nullLambda * N.transpose() * N;
    g -= 2.0 * _cfg.nullLambda * N.transpose() * N * Kp * qErr;
  }

  H += _cfg.damping * Eigen::MatrixXd::Identity(_numDOF, _numDOF);
}

void DiffIKQP::_buildConstraints(const Eigen::VectorXd &qCur,
                                 const Eigen::VectorXd &qdCur,
                                 double dt,
                                 Eigen::MatrixXd &A,
                                 Eigen::VectorXd &lbA,
                                 Eigen::VectorXd &ubA) {
  int rowIdx = 0;
  int numC = _computeNumConstraints();

  if (numC == 0) {
    return;
  }

  A.resize(numC, _numDOF);
  lbA.resize(numC);
  ubA.resize(numC);

  if (_cfg.enablePosLimits) {
    A.block(rowIdx, 0, _numDOF, _numDOF) = dt * Eigen::MatrixXd::Identity(_numDOF, _numDOF);
    lbA.segment(rowIdx, _numDOF) = _cfg.qMin - qCur;
    ubA.segment(rowIdx, _numDOF) = _cfg.qMax - qCur;
    rowIdx += _numDOF;
  }

  if (_cfg.enableAccLimits) {
    A.block(rowIdx, 0, _numDOF, _numDOF) = Eigen::MatrixXd::Identity(_numDOF, _numDOF);
    lbA.segment(rowIdx, _numDOF) = _cfg.qddMin * dt + qdCur;
    ubA.segment(rowIdx, _numDOF) = _cfg.qddMax * dt + qdCur;
    rowIdx += _numDOF;
  }
}

bool DiffIKQP::solve(const Eigen::MatrixXd &jaco,
                     const Eigen::VectorXd &taskVel,
                     const Eigen::VectorXd &qCur,
                     const Eigen::VectorXd &qdCur,
                     double dt) {
  Eigen::MatrixXd H(_numDOF, _numDOF);
  Eigen::VectorXd g(_numDOF);
  _buildCost(jaco, taskVel, qCur, H, g);

  int numC = _computeNumConstraints();
  Eigen::MatrixXd A;
  Eigen::VectorXd lbA, ubA;

  if (numC > 0) {
    _buildConstraints(qCur, qdCur, dt, A, lbA, ubA);
  }

  qpOASES::Options opts;
  opts.printLevel = qpOASES::PL_NONE;
  _sqp->setOptions(opts);

  Eigen::VectorXd lbx = _cfg.enableVelLimits ? _cfg.qdMin :
                                               Eigen::VectorXd::Constant(_numDOF, -INFINITY);
  Eigen::VectorXd ubx = _cfg.enableVelLimits ? _cfg.qdMax :
                                               Eigen::VectorXd::Constant(_numDOF, +INFINITY);

  _qpNWSR = _cfg.maxIter;
  _qpCPUtime[0] = dt;

  qpOASES::returnValue ret;
  if (!_qpSolved) {
    ret = _sqp->init(H.data(), g.data(),
                     numC > 0 ? A.data() : nullptr,
                     lbx.data(), ubx.data(),
                     numC > 0 ? lbA.data() : nullptr,
                     numC > 0 ? ubA.data() : nullptr,
                     _qpNWSR, _qpCPUtime);
  } else {
    ret = _sqp->hotstart(H.data(), g.data(),
                         numC > 0 ? A.data() : nullptr,
                         lbx.data(), ubx.data(),
                         numC > 0 ? lbA.data() : nullptr,
                         numC > 0 ? ubA.data() : nullptr,
                         _qpNWSR, _qpCPUtime);
  }

  if (ret == qpOASES::SUCCESSFUL_RETURN) {
    _sqp->getPrimalSolution(_qdCmd.data());
    _qdCmdPrev = _qdCmd;
    _qpSolved = true;
    return true;
  } else {
    std::cerr << "[DiffIKQP] QP solve failed, using previous solution" << std::endl;
    _qdCmd = _qdCmdPrev;
    _qpSolved = false;
    return false;
  }
}

} // namespace utils
