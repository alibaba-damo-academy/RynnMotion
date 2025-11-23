#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "dyn_pin.hpp"
#include "interpolate.hpp"
#include "module_base.hpp"
#include "orient_tools.hpp"
#include "runtime_data.hpp"
#include "scene_manager.hpp"
#include "signal_generator.hpp"

namespace rynn {

/**
 * @class CJointMove
 * @brief Direct joint control module
 *
 * Reads:  qFb (joint feedback), simTime
 * Writes: qCmd, qdCmd, qtauCmd (joint commands), gripperCommands[]
 *
 * Generates joint-space trajectories for testing and simple motions:
 * - General joint motions (sine waves, keyframe interpolation)
 * - Single-link testing
 * - Keyframe cycling with wobble
 *
 * Used primarily for model testing and simple joint-level control.
 */
class CJointMove : public CModuleBase {
public:
  explicit CJointMove(const YAML::Node &yamlNode);
  ~CJointMove() override = default;

  void loadYaml() override;
  void update() override;
  void initModule() override;

protected:
  // Inherits: std::shared_ptr<data::RuntimeData> runtimeData_;
  std::vector<std::unique_ptr<utils::SignalGenerator>> source_;
  std::shared_ptr<utils::PinDynamics> pinDynamics;

private:
  double duration_{1.0};
  double amp_{M_PI};
  Eigen::VectorXd _qFb0; // Initial joint positions (captured on FSM entry, not in initModule)
  SceneType _sceneType{SceneType::kDefault};

  void generalJointMove(double time);
  void oneLinkMotion(double time);
  void keyframeWobbling(double time);
  void initSingleActuator();
};

} // namespace rynn
