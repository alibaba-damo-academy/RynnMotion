#pragma once

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>
#include <vector>

#include "kine_pin.hpp"
#include "robot_manager.hpp"

#ifdef __APPLE__
#include "glfw3.h"
#include "mujoco.h"
#else
#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"
#endif

namespace test {

/**
 * @brief FK comparison test between Pinocchio and MuJoCo
 *
 * Supports two modes:
 * - Single-site: Compare EE pose or Jacobian
 * - Multi-site: Compare multiple sites (shoulder, elbow, wrist, EE)
 *
 * Quaternions use Eigen standard format (x, y, z, w).
 */
class HybridKinematicsTest {
public:
  /**
   * @brief Constructor for single-site mode (EE only)
   * @param robot_number Robot type: 20=FR3, 21=UR5E, 22=Piper, 23=RM75, 24=SO101
   */
  HybridKinematicsTest(int robot_number = 20);

  /**
   * @brief Constructor for multi-site mode
   * @param robot_number Robot type number
   * @param site_names Sites to track (e.g., {"shoulderSite", "elbowSite", "wristSite", "EE"})
   */
  HybridKinematicsTest(int robot_number, const std::vector<std::string> &site_names);

  ~HybridKinematicsTest();

  bool initialize();

  /// Single-site test. scenario: 0=zero (5 steps), 1=lerp (10s)
  bool runKineComparison(int scenario = 0, bool compareJacobian = false);

  /// Multi-site test. scenario: 0=zero (5 steps), 1=lerp (10s)
  bool runMultiSiteComparison(int scenario = 0);

  void setPositionTolerance(double tolerance) {
    posTolerance_ = tolerance;
  }

  void setOrientationTolerance(double tolerance) {
    quatTolerance_ = tolerance;
  }

  void setRenderingMode(bool enableRendering) {
    enableRendering_ = enableRendering;
  }

private:
  bool loadMujocoModel();
  bool initializePinocchio();
  Eigen::VectorXd genJointRef(double time, int scenario, double durationSeconds);

  // Pose getters (quat in x,y,z,w format)
  bool getMujocoEEPose(const Eigen::VectorXd &q, Eigen::Vector3d &pos, Eigen::Vector4d &quat);
  bool getPinEEPose(const Eigen::VectorXd &q, Eigen::Vector3d &pos, Eigen::Vector4d &quat);
  bool getMujocoSitePose(const Eigen::VectorXd &q, int siteIndex, Eigen::Vector3d &pos, Eigen::Vector4d &quat);
  bool getPinSitePose(const Eigen::VectorXd &q, int siteIndex, Eigen::Vector3d &pos, Eigen::Vector4d &quat);

  // Jacobian getters
  bool getMujocoJaco(const Eigen::VectorXd &q, Eigen::MatrixXd &jaco);
  bool getPinJaco(const Eigen::VectorXd &q, Eigen::MatrixXd &jaco);

  // Comparison
  bool comparePoses(const Eigen::Vector3d &pos1, const Eigen::Vector4d &quat1,
                    const Eigen::Vector3d &pos2, const Eigen::Vector4d &quat2);
  bool compareJacobians(const Eigen::MatrixXd &jaco1, const Eigen::MatrixXd &jaco2);

  // Rendering
  bool initRendering();
  void callRendering();
  void closeRendering();
  void printResults();

private:
  // MuJoCo
  mjModel *mjModel_;
  mjData *mjData_;
  std::string mjcfPath_;
  int eeSiteId_;
  double simTimestep_;

  // Rendering
  GLFWwindow *glfwWindow_;
  mjvCamera mjCamera_;
  mjvOption mjvOption_;
  mjvScene mjScene_;
  mjrContext mjrContext_;
  bool enableRendering_;
  bool renderingInitialized_;

  // Pinocchio
  std::unique_ptr<utils::PinKine> pinKine_;
  std::unique_ptr<rynn::RobotManager> robotManager_;

  // Config
  const int robotNumber_;
  int numJoints_;
  double posTolerance_;
  double quatTolerance_;
  double jacoTolerance_;
  Eigen::VectorXd targetJointAngles_;

  // Results
  int testsPassed_;
  int testsFailed_;
  double maxPosError_;
  double maxQuatError_;

  // Multi-site mode
  bool multiSiteMode_;
  std::vector<std::string> siteNames_;
  std::vector<int> siteIds_;
};

} // namespace test
