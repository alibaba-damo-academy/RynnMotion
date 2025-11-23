#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <memory>

#include "kine_pin.hpp"
#include "module_base.hpp"

namespace rynn {

/**
 * @class TeleopFollow
 * @brief Teleoperation following base class
 *
 * Reads:  bodyPlans[] (teleop commands), bodyStates[] (current EE states),
 *         jacobians[], qFb, qdFb
 * Writes: qCmd, qdCmd, kp, kd
 *
 * Enables teleoperation by converting Cartesian teleop commands
 * into joint-space trajectories with collision checking and limits.
 */
class TeleopFollow : public CModuleBase {
public:
  explicit TeleopFollow(const YAML::Node &yamlNode);
  virtual ~TeleopFollow() = default;

  void loadYaml() override;
  void update() override;
  void initModule() override;

protected:
  // Inherits: std::shared_ptr<data::RuntimeData> runtimeData_;
  double _t{0.0}, _dt{0.002};
  double _kpTaskSpace{0.}, _kdTaskSpace{0.};
  std::unique_ptr<utils::PinKine> _pinKine;
  // State vectors - single kinematic tree
  Eigen::VectorXd _qFb, _qdFb, _qCmd, _qdCmd;
  Eigen::VectorXd _eeVel_des, _eeVel_fb;
  Eigen::MatrixXd _eeJacoFb;
  Eigen::VectorXd _qTask; // Keep reference task vector

  // Common methods
  Eigen::VectorXd _setupTaskPD(const Eigen::VectorXd &taskSpaceError);
  virtual void _taskTeleopFollow();
};

} // namespace rynn
