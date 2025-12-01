#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "robot_types.hpp"

struct mjModel_;
typedef struct mjModel_ mjModel;

namespace rynn {

/**
 * @brief Enumerates the MuJoCo actuator control modes
 */
enum class ActuatorMode {
  kGeneral,     ///< Default/general purpose actuator mode
  kPosition,    ///< Position control mode
  kVelocity,    ///< Velocity control mode
  kTorque,      ///< Direct torque control mode
  kIntVelocity, ///< Integrated velocity control mode
  kDamper,      ///< Damper actuator mode
  kCylinder,    ///< Hydraulic cylinder actuator
  kMuscle,      ///< Muscle actuator mode
  kAdhesion,    ///< Adhesion actuator mode
  kPlugin       ///< Plugin-defined actuator mode
};

/**
 * @class RobotManager (RobotMjcfManager)
 * @brief Manages robot configuration from MJCF files
 *
 * This class merges MjRobotParser and RobotManager functionality into a single
 * source of truth for robot configuration. It parses MJCF files to extract:
 * - DOF classification (motion vs action)
 * - End-effector configuration
 * - Joint limits, keyframes, actuator modes
 * - File paths (robot MJCF, Pinocchio MJCF)
 *
 * Philosophy: All robot configuration comes from MJCF parsing (YAML-free),
 * with optional YAML gains for simulation/real hardware tuning.
 */

class RobotManager {
public:
  using EigenVec3d = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>; // avoid compilation error with rbdl std::vector<Eigen::Vector3d>
  using EigenVecXd = std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>; // avoid compilation error with std::vector<Eigen::VectorXd>

  explicit RobotManager(int robotNumber);
  explicit RobotManager(const std::string &mjcfPath);

  int getMotionDOF() const;
  int getActionDOF() const;
  void setMotionDOF(int mdof);
  void setActionDOF(int adof);
  int getRobotNumber() const;
  rynn::RobotType getRobotType() const;
  std::string getRobotMJCF() const;
  std::string getPinoMJCF() const;

  std::vector<int> getEEJointIndices() const;
  void setEEJointIndices(const std::vector<int> &indices);
  int getNumEndEffectors() const;

  std::vector<std::pair<double, double>> getEERanges() const;
  void setEERanges(const std::vector<std::pair<double, double>> &ranges);
  double normalizeEECommand(int eeIndex, double normalizedCommand) const;

  void setEndEffectorConfig(int adof,
                            const std::vector<int> &eeJointIndices,
                            const std::vector<std::pair<double, double>> &eeRanges);

  std::vector<int> getJointIndices() const;
  std::vector<int> getEEIndices() const;
  void setActuatorIndices(const std::vector<int> &jointIndices, const std::vector<int> &eeIndices);

  std::vector<ActuatorMode> getActuatorModes() const;
  void setActuatorModes(const std::vector<ActuatorMode> &modes);

  Eigen::VectorXd getSimKp() const;
  Eigen::VectorXd getSimKd() const;

  // Uses same gains as simulation for consistency
  Eigen::VectorXd getRealKp() const;
  Eigen::VectorXd getRealKd() const;

  // Methods for robot joint keyframe values (indexed system)
  // Index 0: home, Index 1: standby1, Index 2: standby2, Index 3+: additional keyframes
  Eigen::VectorXd getQStandby(int index = 0) const; // Returns keyframes[1 + index] (default: standby1)
  Eigen::VectorXd getKeyframe(int index) const;     // Returns keyframes[index]
  int getNumKeyframes() const;                      // Returns total number of keyframes
  void setKeyframes(const EigenVecXd &keyframes,
                    const std::vector<std::string> &names); // Sets all keyframes at once

  EigenVecXd getAllKeyframes() const; // Returns all keyframes from mjModel_ (mdof x 1 each)

  Eigen::VectorXd getJointPosMin() const;
  Eigen::VectorXd getJointPosMax() const;
  Eigen::VectorXd getJointVelMax() const;
  Eigen::VectorXd getJointTorqueMax() const;
  void setJointLimits(const Eigen::VectorXd &qPosMin, const Eigen::VectorXd &qPosMax,
                      const Eigen::VectorXd &qVelMax, const Eigen::VectorXd &qTorqueMax);

  std::vector<std::string> getSiteNames() const;
  void setSiteNames(const std::vector<std::string> &siteNames);
  int getNumSites() const;
  bool hasSites() const;

  // Phase 5: All static conversion methods REMOVED
  // Use RobotDiscovery::getInstance().getRobotInfo() instead

  Eigen::VectorXd _simKp; // Simulation motor gains (from MJCF)
  Eigen::VectorXd _simKd;

private:
  void loadMjcfPaths();
  int _robotNumber{-1};
  int _mdof{0};                                     // Motion DOF (total joint actuators)
  int _adof{0};                                     // Action DOF per end-effector (1 for gripper, 5+ for hand)
  std::vector<int> _eeJointIndices;                 // Parent joint index in [0, mdof-1] for each EE
  std::vector<std::pair<double, double>> _eeRanges; // Control ranges for each EE
  std::string _robotMjcfPath, _pinoMjcfPath;
  EigenVec3d _eeVecs;

  // Parsed from MJCF
  std::vector<int> _jointIndices;           // Actuator indices for joints in [0, nu-1]
  std::vector<int> _eeIndices;              // Actuator indices for end-effectors in [0, nu-1]
  std::vector<ActuatorMode> _actuatorModes; // Control mode for each actuator

  // Keyframe values (from MJCF) - Indexed system matching Python implementation
  // Index 0: home, Index 1: standby1, Index 2: standby2, Index 3+: additional keyframes
  EigenVecXd _keyframes;
  std::vector<std::string> _keyframeNames;

  // Joint limits (from MJCF)
  Eigen::VectorXd _qPosMin;
  Eigen::VectorXd _qPosMax;
  Eigen::VectorXd _qVelMax;
  Eigen::VectorXd _qTorqueMax;

  // Site names (from MJCF, extracted via MuJoCo API)
  std::vector<std::string> _siteNames;
};

} // namespace rynn
