#pragma once

#include "joint_move.hpp"

namespace rynn {

/**
 * @class RizonJointMove
 * @brief Joint control module for Rizon robot (MuJoCo simulation)
 *
 * Implements hold/sine-sweep behavior matching Flexiv RDK intermediate1 example.
 * Used for MuJoCo simulation when running Rizon robot with example 1-4.
 *
 * Modes:
 *   - Hold mode: Keep all joints at initial position
 *   - Sine-sweep mode: All joints oscillate with same amplitude/frequency
 *
 * Configuration (motion.yaml):
 *   RizonJointMove:
 *     hold_mode: false          # true = hold, false = sine-sweep
 *     sine_amplitude: 0.035     # radians (~2 degrees)
 *     sine_frequency: 0.3       # Hz
 */
class RizonJointMove : public CJointMove {
public:
  explicit RizonJointMove(const YAML::Node& yamlNode);
  ~RizonJointMove() override = default;

  void loadYaml() override;
  void initModule() override;
  void update() override;

private:
  double sineAmp_{0.035};     // radians (from intermediate1)
  double sineFreq_{0.3};      // Hz (from intermediate1)
  bool holdMode_{false};      // true = hold, false = sine-sweep

  Eigen::VectorXd initPos_;   // Captured initial positions
  bool initPosCaptured_{false};

  void holdPosition();
  void sineSweep(double time);
};

} // namespace rynn
