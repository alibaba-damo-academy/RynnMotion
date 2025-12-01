#pragma once

#include "robot_manager.hpp"

// Forward declaration
struct mjModel_;
typedef struct mjModel_ mjModel;

namespace rynn {

/**
 * @class MjcfParser
 * @brief Parses MuJoCo MJCF files to extract robot configuration
 *
 * Responsibilities:
 * - Load both robotMJCF (full model) and pinoMJCF (kinematic tree)
 * - Extract robot parameters (actuator types, indices, end-effector actuator types indices, joint limits, gains, keyframes, etc.)
 * - Populate RobotManager with extracted data
 *
 */
class MjcfParser {
public:
  /**
   * @brief Construct MjcfParser and load MJCF models
   * @param rm Reference to RobotManager to populate
   *
   * Automatically:
   * - Loads robotMJCF (*_robot.xml) into robotModel_
   * - Loads pinoMJCF (*_pinocchio.xml) into pinoModel_
   * - Calls parseAndPopulate() to extract all configuration
   */
  explicit MjcfParser(RobotManager &rm);

  /**
   * @brief Destructor - cleans up loaded MuJoCo models
   */
  ~MjcfParser();

  MjcfParser(const MjcfParser &) = delete;
  MjcfParser &operator=(const MjcfParser &) = delete;
  MjcfParser(MjcfParser &&) = delete;
  MjcfParser &operator=(MjcfParser &&) = delete;

  /**
   * @brief Extract keyframes from an already-loaded scene model
   * @param model Loaded MuJoCo scene model (mjModel_)
   * @param mdof Motion DOF (number of joints to extract)
   * @return Vector of keyframes (each is mdof x 1)
   *
   * This helper extracts keyframes from mjModel_->key_qpos without requiring
   * a RobotManager instance. Used for scene 2 keyframe cycling.
   */
  static RobotManager::EigenVecXd extractKeyframesFromModel(mjModel *model, int mdof);

private:
  /**
   * @brief Load MJCF files and populate RobotManager
   */
  void parseAndPopulate();

  /**
   * @brief Orchestrate all extraction methods
   */
  void parseRobotModel();

  /**
   * @brief Detect end-effectors (grippers) vs motion joints
   */
  void detectEndEffectors();

  /**
   * @brief Compute parent joint for each end-effector
   */
  void computeEEParentJoints();

  /**
   * @brief Extract joint position/velocity/torque limits
   */
  void extractJointLimits();

  /**
   * @brief Detect actuator control modes (position, velocity, torque)
   */
  void detectActuatorModes();

  /**
   * @brief Extract actuator PD gains (kp, kd) from MJCF
   */
  void extractActuatorGains();

  /**
   * @brief Extract keyframes from Pinocchio MJCF
   */
  void extractKeyframes();

  /**
   * @brief Extract site names from Pinocchio MJCF
   */
  void extractSiteNames();

  RobotManager &rm_;
  mjModel *robotModel_;
  mjModel *pinoModel_;
};

} // namespace rynn
