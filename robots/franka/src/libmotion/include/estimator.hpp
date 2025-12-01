#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "runtime_data.hpp"
#include "module_base.hpp"

namespace rynn {
/**
 * @class CEstimator
 * @brief Robot state estimation module
 *
 * Reads:  qFb, qdFb, qtauFb, simTime
 * Writes: bodyStates[] (end-effector poses), jacobians[]
 *
 * Uses forward kinematics (Pinocchio) to compute end-effector states
 * from joint feedback and store them in RuntimeData for downstream modules.
 */
class CEstimator : public CModuleBase {
public:
  explicit CEstimator(const YAML::Node &yamlNode);
  virtual ~CEstimator() = default;

  void update() override;

  void loadYaml() override;
  void initModule() override;

protected:
  // Inherits: std::shared_ptr<data::RuntimeData> runtimeData_;

  Eigen::VectorXd _qFb, _qdFb, _qtauFb;
  Eigen::Matrix4d _eeT;
  Eigen::MatrixXd _eeJaco;

  virtual void _stateEstimation();
  virtual void _updateFsm();

private:
  void _pinEstimator();
};

} // namespace rynn
