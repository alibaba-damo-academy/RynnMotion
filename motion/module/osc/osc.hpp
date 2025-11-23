#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <memory>

#include "diff_ik.hpp"
#include "kine_pin.hpp"
#include "module_base.hpp"
#include "motion_fsm.hpp"

namespace rynn {

/**
 * @class OSC
 * @brief Operational Space Control module
 *
 * Reads:  bodyPlans[] (desired EE poses), bodyStates[] (current EE poses),
 *         jacobians[], qFb, qdFb, simTime
 * Writes: qCmd, qdCmd, qtauCmd (joint commands)
 *
 * Performs inverse kinematics to convert task-space goals (end-effector poses)
 * into joint-space commands. Supports multiple IK solvers:
 * - PseudoInverse: Simple damped least-squares
 * - DiffQP: Differential IK with QP constraints (vel/pos/acc limits, nullspace)
 *
 * Handles both single and multi-arm robots with decoupled IK per end-effector.
 */
class OSC : public CModuleBase {
public:
  explicit OSC(const YAML::Node &yamlNode);
  virtual ~OSC() = default;

  void loadYaml() override;
  void update() override;
  void initModule() override;

protected:
  enum class IKSolver {
    kPseudoInverse,
    kDiffQP
  };

  // Inherits: std::shared_ptr<data::RuntimeData> runtimeData_;
  double _t{0.0};
  double _kpTaskSpace{0.}, _kdTaskSpace{0.};
  IKSolver _solver{IKSolver::kDiffQP};
  utils::DiffIKConfig _diffIKCfg;

  Eigen::VectorXd _qFb, _qdFb, _qCmd, _qdCmd;
  Eigen::Matrix4d _eeT_plan, _eeT_fb;
  Eigen::VectorXd _eeVel_des, _eeVel_fb;
  Eigen::MatrixXd _eeJacoFb;
  Eigen::VectorXd _qTask;

  std::unique_ptr<utils::DiffIKQP> _dIKqp;

  int _numEE{1};
  struct EEState {
    Eigen::VectorXd eeVel_des;
    Eigen::MatrixXd eeJacoFb;
    std::unique_ptr<utils::DiffIKQP> dIKqp;
  };
  std::vector<EEState> _eeStates;
  std::vector<Eigen::VectorXd> _qdCmdPerEE;

  Eigen::VectorXd _setupTaskPD(const Eigen::VectorXd &taskSpaceError);
  virtual void _taskSpaceMove();
  void _initQP();
  void _solveDecoupledIK();
  void _mergeJointCommands();
};

} // namespace rynn
