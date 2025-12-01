#pragma once

#include <memory>
#include <vector>

#include "pickplace_fsm.hpp"
#include "pose.hpp"
#include "robot_manager.hpp"

namespace rynn {

/**
 * @brief Configuration for pick-and-place operations (per-robot tunable)
 */
struct PickPlaceConfig {
  // Height offsets (meters)
  double prePickHeight = 0.1; // Hover height above object before descent
  double graspHeight = 0.01;  // Final grasp Z-offset from table surface
  double liftHeight = 0.1;    // Lift height after grasp (relative to grasp pose)
  double dropHeight = 0.2;    // Drop height above target container

  // Gripper parameters
  double graspTime = 0.3;        // Time to close gripper (seconds)
  double releaseTime = 0.3;      // Time to open gripper (seconds)
  double minGripperClosed = 0.2; // Gripper position threshold for "holding object"

  // Trajectory parameters
  double velMax = 1.0;        // Base velocity (m/s)
  double accMax = 5.0;        // Base acceleration (m/s²)
  double velMultiplier = 4.0; // Velocity multiplier for trajectories
  double accMultiplier = 2.0; // Acceleration multiplier for trajectories

  // Safety and validation
  double maxReachDistance = 1.0; // Maximum reach from stand pose (meters)
  double poseTolerance = 1e-3;   // Position error tolerance (1mm)
  double angleTolerance = 1e-3;  // Angle error tolerance (~0.06°)

  // Error recovery
  int maxGraspRetries = 2;            // Max attempts per object
  bool skipFailedObjects = true;      // Continue to next object on failure
  bool enableGraspValidation = false; // Check gripper position after grasp (future)

  // Debugging
  bool printDebug = false; // Enable detailed console output
};

/**
 * @brief Get pick-place configuration for specific robot type
 * @param robotType The robot type
 * @return Configuration with robot-specific parameters
 */
PickPlaceConfig getPickPlaceConfigForRobot(RobotType robotType);

/**
 * @brief High-level controller for pick-and-place operations
 *
 * This class manages the complete pick-and-place pipeline:
 * - Object detection and pose computation
 * - Per-robot configuration
 * - FSM coordination
 * - Grasp validation and error recovery
 */
class PickPlaceController {
public:
  /**
   * @brief Constructor
   * @param robotType Robot type for configuration
   * @param armIndex Arm index for dual-arm support (0=left, 1=right)
   * @param config Optional custom configuration (uses robot defaults if null)
   */
  explicit PickPlaceController(RobotType robotType,
                               int armIndex = 0,
                               const PickPlaceConfig *config = nullptr);

  ~PickPlaceController() = default;

  /**
   * @brief Command output from controller
   */
  struct Command {
    data::Pose targetPose;     // Desired end-effector pose
    bool openGripper = true;   // Gripper command (true=open, false=close)
    bool isCompleted = false;  // Sequence completed flag
    std::string statusMessage; // Human-readable status (for debugging)
  };

  /**
   * @brief Initialize the pick-place sequence
   * @param standPose Robot's standing pose (start and return position)
   * @param dropPose Drop target pose (above container)
   * @param objectPoses Positions of all objects to grasp
   * @param dt Timestep for trajectory generation
   * @return true if initialization successful
   */
  bool initialize(const data::Pose &standPose,
                  const data::Pose &dropPose,
                  const std::vector<data::Pose> &objectPoses);

  /**
   * @brief Update the controller (called every control cycle)
   * @param dt Timestep for this update
   * @return Command with target pose and gripper state
   */
  Command update(double dt);

  /**
   * @brief Reset the controller to initial state
   */
  void reset();

  /**
   * @brief Check if sequence is complete
   */
  bool isComplete() const;

  /**
   * @brief Get current configuration
   */
  const PickPlaceConfig &getConfig() const {
    return _config;
  }

  /**
   * @brief Update configuration (requires reset to take effect)
   */
  void setConfig(const PickPlaceConfig &config) {
    _config = config;
  }

private:
  // Configuration
  RobotType _robotType;
  int _armIndex = 0;  // Arm index for dual-arm support
  PickPlaceConfig _config;
  bool _initialized = false;

  // Task data
  data::Pose _standPose;
  data::Pose _dropPose;
  std::vector<data::Pose> _objectPoses;

  // Computed poses
  std::vector<data::Pose> _graspPoses;   // Grasp poses (with height offset)
  std::vector<data::Pose> _prePickPoses; // Pre-pick poses (hover above grasp)
  std::vector<data::Pose> _liftPoses;    // Lift poses (after grasp)

  // State tracking
  int _currentObjectIndex = 0;
  int _currentRetryCount = 0;

  // Helper methods
  /**
   * @brief Compute all poses for the sequence
   * @return true if all poses are valid
   */
  bool computeAllPoses();

  /**
   * @brief Validate that object is within reach
   * @param objectPose Object position to check
   * @return true if reachable
   */
  bool isObjectReachable(const data::Pose &objectPose) const;

  /**
   * @brief Compute grasp pose from object position
   * @param objectPose Object position (from MuJoCo)
   * @return Grasp pose (with height offset and orientation)
   */
  data::Pose computeGraspPose(const data::Pose &objectPose) const;

  /**
   * @brief Compute pre-pick hover pose
   * @param graspPose Grasp pose
   * @return Pre-pick pose (above grasp)
   */
  data::Pose computePrePickPose(const data::Pose &graspPose) const;

  /**
   * @brief Compute lift pose (after grasp)
   * @param graspPose Grasp pose
   * @return Lift pose (above grasp)
   */
  data::Pose computeLiftPose(const data::Pose &graspPose) const;

  /**
   * @brief Validate grasp success (check gripper feedback)
   * @return true if object is held
   */
  bool verifyGraspSuccess();

  /**
   * @brief Handle grasp failure (retry or skip)
   * @return true if should retry, false if should skip
   */
  bool handleGraspFailure();
};

} // namespace rynn
