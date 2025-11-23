#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <vector>

#include "camera.hpp"
#include "com_feedback.hpp"
#include "ft_sensor.hpp"
#include "gripper.hpp"
#include "imu.hpp"
#include "jacobian.hpp"
#include "motor.hpp"
#include "rigid_body_state.hpp"
#include "timer.hpp"

// Forward declaration for MuJoCo data structure
struct mjData_;
typedef struct mjData_ mjData;

namespace data {

/**
 * @struct RuntimeData
 * @brief Runtime data container using LOGICAL indices
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ IMPORTANT: Index Convention                                    │
 * ├─────────────────────────────────────────────────────────────────┤
 * │ All arrays in RuntimeData use LOGICAL indices, NOT MuJoCo      │
 * │ actuator IDs! This provides a hardware-agnostic interface.     │
 * │                                                                 │
 * │ Joint data arrays (qFb, qCmd, etc.):                           │
 * │   - Size: mdof (motion degrees of freedom)                     │
 * │   - Index: [0, mdof-1] represents logical joint number         │
 * │   - Maps to MuJoCo via: RobotManager::getJointIndices()        │
 * │                                                                 │
 * │ Gripper data arrays (gripperCommands, gripperFeedbacks):       │
 * │   - Size: numEE (number of end-effectors)                      │
 * │   - Index: [0, numEE-1] represents logical EE number           │
 * │   - Maps to MuJoCo via: RobotManager::getEEIndices()           │
 * │                                                                 │
 * │ Example (dual FR3 robot):                                      │
 * │   qCmd[0..13]            → 14 joint positions (7 per arm)      │
 * │   gripperCommands[0..1]  → 2 gripper commands (1 per arm)      │
 * │   MuJoCo ctrl[0..15]     → Physical layout: [arm0_j, arm0_ee,  │
 * │                                               arm1_j, arm1_ee]  │
 * │   Mapping handled by MujocoActuator using index arrays         │
 * └─────────────────────────────────────────────────────────────────┘
 */
struct RuntimeData {
  // ========== Timing ==========
  double simTime{0.0};
  double wallTime{0.0};
  double duration{0.0};

  // ========== Joint Control Data (Motion DOF) ==========
  // Indexed by LOGICAL joint number [0, mdof-1]
  Eigen::VectorXd qFb;     // Joint position feedback
  Eigen::VectorXd qdFb;    // Joint velocity feedback
  Eigen::VectorXd qtauFb;  // Joint torque feedback
  Eigen::VectorXd qCmd;    // Joint position commands
  Eigen::VectorXd qdCmd;   // Joint velocity commands
  Eigen::VectorXd qtauCmd; // Joint torque commands
  std::vector<double> kp;  // Position gains
  std::vector<double> kd;  // Velocity gains

  // ========== Gripper/End-Effector Commands ==========
  // Indexed by LOGICAL end-effector number [0, numEE-1]
  std::vector<GripperCommand> gripperCommands;
  std::vector<GripperFeedback> gripperFeedbacks;

  // ========== Rigid Body States (UNIFIED) ==========
  std::vector<RigidBodyState> bodyStates;   // Robot body links/frames
  std::vector<RigidBodyState> objectStates; // Tracked objects
  std::vector<BodyPlan> bodyPlans;          // Planned trajectories

  // ========== Sensor Data ==========
  std::vector<Imu> imuSensors;              // IMU data (quat, rpy, gyro, accel)
  std::vector<FTSensor> ftSensors;          // Force/Torque sensors
  std::vector<RigidBodyState> frameSensors; // Frame sensors (pos+quat+vel)

  // ========== Camera Data ==========
  std::vector<Camera> cameras; // Camera poses

  // ========== Jacobians ==========
  std::vector<Jacobian> jacobians;

  // ========== Center of Mass ==========
  ComFeedback comFeedback;

  // ========== MuJoCo Data Reference (optional) ==========
  mjData *mjData_{nullptr};

  RuntimeData(int mdof = 0, int numEE = 0) {
    if (mdof > 0) {
      qFb = Eigen::VectorXd::Zero(mdof);
      qdFb = Eigen::VectorXd::Zero(mdof);
      qtauFb = Eigen::VectorXd::Zero(mdof);
      qCmd = Eigen::VectorXd::Zero(mdof);
      qdCmd = Eigen::VectorXd::Zero(mdof);
      qtauCmd = Eigen::VectorXd::Zero(mdof);
      kp.resize(mdof, 0.0);
      kd.resize(mdof, 0.0);
    }

    if (numEE > 0) {
      gripperCommands.resize(numEE);
      gripperFeedbacks.resize(numEE);
    }
  }

  void setJointsCommand(const Eigen::VectorXd &q, const Eigen::VectorXd &qd, const Eigen::VectorXd &qtau) {
    qCmd = q;
    qdCmd = qd;
    qtauCmd = qtau;
  }

  void getJointsFeedback(Eigen::VectorXd &q, Eigen::VectorXd &qd, Eigen::VectorXd &qtau) const {
    q = qFb;
    qd = qdFb;
    qtau = qtauFb;
  }

  void getJointsCommand(Eigen::VectorXd &q, Eigen::VectorXd &qd, Eigen::VectorXd &qtau) const {
    q = qCmd;
    qd = qdCmd;
    qtau = qtauCmd;
  }

  void setGripperPosition(size_t index, double position) {
    if (index < gripperCommands.size()) {
      gripperCommands[index].posCmd = std::clamp(position, 0.0, 1.0);
    }
  }

  double getGripperPosition(size_t index = 0) const {
    return (index < gripperFeedbacks.size()) ? gripperFeedbacks[index].posFb : 0.0;
  }

  void openGripper(size_t index = 0) {
    setGripperPosition(index, 1.0);
  }

  void closeGripper(size_t index = 0) {
    setGripperPosition(index, 0.0);
  }

  // ========== Sensor Access API ==========

  /**
   * @brief Get or create IMU sensor at index
   *
   * Auto-resizes the vector if index is out of bounds.
   */
  Imu &imuSensor(size_t index) {
    if (index >= imuSensors.size()) {
      imuSensors.resize(index + 1);
    }
    return imuSensors[index];
  }

  /**
   * @brief Get or create F/T sensor at index
   *
   * Auto-resizes the vector if index is out of bounds.
   */
  FTSensor &ftSensor(size_t index) {
    if (index >= ftSensors.size()) {
      ftSensors.resize(index + 1);
    }
    return ftSensors[index];
  }

  /**
   * @brief Get or create frame sensor at index
   *
   * Auto-resizes the vector if index is out of bounds.
   */
  RigidBodyState &frameSensor(size_t index) {
    if (index >= frameSensors.size()) {
      frameSensors.resize(index + 1);
    }
    return frameSensors[index];
  }

  // ========== Object/Body Management API ==========

  ObjectState *getObjectByName(const std::string &name) {
    for (auto &obj : objectStates) {
      if (obj.name == name) return &obj;
    }
    return nullptr;
  }

  void addObject(const std::string &name) {
    objectStates.emplace_back(name);
  }

  void addBodyFeedback(const std::string &name) {
    bodyStates.emplace_back(name);
  }

  void addBodyState(const std::string &name) {
    bodyStates.emplace_back(name);
  }

  void addBodyPlanner() {
    bodyPlans.emplace_back();
  }

  void addCamera(const std::string &name) {
    cameras.emplace_back(name);
  }

  void addJacobian(const std::string &name, int rows = 0, int cols = 0) {
    jacobians.emplace_back(name, rows, cols);
  }

  // ========== Counts ==========
  size_t getObjectCount() const {
    return objectStates.size();
  }

  size_t getCameraCount() const {
    return cameras.size();
  }

  size_t getImuCount() const {
    return imuSensors.size();
  }

  size_t getFTSensorCount() const {
    return ftSensors.size();
  }

  size_t getFrameSensorCount() const {
    return frameSensors.size();
  }
};

} // namespace data
