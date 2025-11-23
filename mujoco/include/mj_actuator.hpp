#pragma once
#include "mj_interface.hpp"
#include "robot_manager.hpp"
#ifdef __APPLE__
#include "mujoco.h"
#else
#include "mujoco/mujoco.h"
#endif

namespace mujoco {

class MujocoInterface;

/**
 * @brief Handles actuator runtime control and feedback for MuJoCo
 *
 * This class is responsible for runtime actuator operations:
 * - Reading feedback from mjData (update)
 * - Writing commands to mjData (setCommand)
 * - PID control logic (pvtpidTorque)
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ Index Mapping Strategy                                          │
 * ├─────────────────────────────────────────────────────────────────┤
 * │ RuntimeData uses LOGICAL indices:                               │
 * │   - qCmd[0..mdof-1]              ← Joint commands by DOF       │
 * │   - gripperCommands[0..numEE-1]  ← Gripper commands by EE#     │
 * │                                                                 │
 * │ mjData->ctrl uses PHYSICAL indices:                            │
 * │   - ctrl[0..nu-1]                ← Actuator commands by ID     │
 * │                                                                 │
 * │ Mapping Arrays (cached from RobotManager):                     │
 * │   - jointIndices_[i] = actuator ID for joint i                 │
 * │   - eeIndices_[i]    = actuator ID for end-effector i          │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * Example (dual FR3):
 *   RuntimeData:   qCmd[0..13], gripperCommands[0..1]
 *   mjData->ctrl:  ctrl[0..15] with layout [arm0_joints, arm0_grip,
 *                                            arm1_joints, arm1_grip]
 *   Mapping:       jointIndices_ = [0..6, 8..14]
 *                  eeIndices_ = [7, 15]
 *
 * Note: Actuator parsing and mode detection is now handled by MjRobotParser
 */
class MujocoActuator {
public:
  /**
   * @brief Constructor that takes a reference to MujocoInterface
   * @param mj Reference to the parent MujocoInterface object
   */
  explicit MujocoActuator(MujocoInterface &mj);

  /**
   * @brief Initialize actuator system (allocates buffers, loads actuator modes from RobotManager)
   */
  void initActuatorSystem();

  /**
   * @brief Initialize robot pose and actuator commands from keyframe
   * @param keyframeIndex Index of the keyframe to load (default: 0)
   */
  void initFromKeyframe(int keyframeIndex = 0);

  /**
   * @brief Update actuator feedback data from mjData to dataStream
   */
  void update();

  /**
   * @brief Set control signals in mjData according to actuator modes
   */
  void setCommand();

  /**
   * @brief Validate actuator index mappings (checks ranges, detects overlaps)
   * @throws std::runtime_error if validation fails
   */
  void validateMapping();

  /**
   * @brief Print actuator mapping table for debugging
   */
  void printMapping();

private:
  /** @brief Reference to the parent MujocoInterface */
  MujocoInterface &mj_;

  /**
   * @brief Calculate PID torque for a specific motor
   * @param motorIndex Index of the motor for which to calculate torque
   * @param qCmd Position command vector (from batch API)
   * @param qdCmd Velocity command vector (from batch API)
   * @param qtauCmd Torque command vector (from batch API)
   * @param qFb Position feedback vector (from batch API)
   * @param qdFb Velocity feedback vector (from batch API)
   * @return Calculated torque value
   */
  double pvtpidTorque(int motorIndex,
                      const Eigen::VectorXd &qCmd,
                      const Eigen::VectorXd &qdCmd,
                      const Eigen::VectorXd &qtauCmd,
                      const Eigen::VectorXd &qFb,
                      const Eigen::VectorXd &qdFb);

  /**
   * @brief Update joint actuators feedback [0, mdof-1]
   */
  void updateJointsFeedback();

  /**
   * @brief Update end-effector actuators feedback [0, numEE*adof-1]
   */
  void updateEEFeedback();

  /**
   * @brief Set joint actuators commands [0, mdof-1]
   */
  void setJointsCommand();

  /**
   * @brief Set end-effector actuators commands [0, numEE*adof-1]
   */
  void setEECommand();

  std::vector<rynn::ActuatorMode> actuatorModes_; // Loaded from RobotManager during init

  // Cached index mappings (populated in initActuatorSystem)
  std::vector<int> jointIndices_; // Maps logical joint → actuator ID
  std::vector<int> eeIndices_;    // Maps logical EE → actuator ID
  int mdof_;                      // Motion DOF (controllable joints)
  int numEE_;                     // Number of end-effectors
  int adof_;                      // Action DOF per end-effector
};

} // namespace mujoco
