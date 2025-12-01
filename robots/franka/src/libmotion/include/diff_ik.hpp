#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <memory>
#include <qpOASES.hpp>
#include <qpOASES/Options.hpp>
#include <qpOASES/QProblem.hpp>
#include <stdexcept>
#include <vector>

#include "math_tools.hpp"

namespace utils {

struct DiffIKConfig {
  bool enableVelLimits = true;
  bool enablePosLimits = false;
  bool enableAccLimits = false;
  bool enableNullSpace = false;

  double damping = 0.0009;
  double nullLambda = 0.01;
  double nullKp = 0.1;
  int maxIter = 30;

  Eigen::VectorXd qdMin, qdMax;
  Eigen::VectorXd qMin, qMax;
  Eigen::VectorXd qddMin, qddMax;
};

/**
 * @class DiffIKQP
 * @brief Solves differential IK using QP with configurable constraints
 *
 * Responsibilities:
 * - Build and solve QP dynamically based on enabled constraints
 * - Support velocity, position, acceleration, and null-space constraints
 * - Provide hot-start capability for real-time performance
 */
class DiffIKQP {
public:
  DiffIKQP(int numDOF, const DiffIKConfig &cfg);
  ~DiffIKQP();

  bool solve(const Eigen::MatrixXd &jaco,
             const Eigen::VectorXd &taskVel,
             const Eigen::VectorXd &qCur,
             const Eigen::VectorXd &qdCur,
             double dt);

  void setNullSpaceTask(const Eigen::VectorXd &qTask) { _qTask = qTask; }

  Eigen::VectorXd getSolution() const { return _qdCmd; }
  bool wasSolved() const { return _qpSolved; }

private:
  DiffIKConfig _cfg;
  int _numDOF;
  Eigen::VectorXd _qTask;

  std::unique_ptr<qpOASES::SQProblem> _sqp;
  bool _qpSolved{false};
  int _qpNWSR;
  qpOASES::real_t _qpCPUtime[1];

  Eigen::VectorXd _qdCmd, _qdCmdPrev;

  int _computeNumConstraints() const;
  void _buildCost(const Eigen::MatrixXd &jaco,
                  const Eigen::VectorXd &taskVel,
                  const Eigen::VectorXd &qCur,
                  Eigen::MatrixXd &H,
                  Eigen::VectorXd &g);
  void _buildConstraints(const Eigen::VectorXd &qCur,
                         const Eigen::VectorXd &qdCur,
                         double dt,
                         Eigen::MatrixXd &A,
                         Eigen::VectorXd &lbA,
                         Eigen::VectorXd &ubA);
};

} // namespace utils