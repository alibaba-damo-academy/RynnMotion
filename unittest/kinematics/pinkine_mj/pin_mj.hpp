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
 * @brief Test class for comparing forward kinematics between Pinocchio and MuJoCo
 */
class HybridKinematicsTest {
public:
  /**
   * @brief Constructor for single-site mode
   * @param robot_number Robot type number (default: 30 for lerobot)\n   *                     Supported robots: 20=fr3, 21=ur5e, 22=piper, 23=rm75, 30=lerobot
   */
  HybridKinematicsTest(int robot_number = 30);

  /**
   * @brief Constructor for multi-site mode
   * @param robot_number Robot type number
   * @param site_names Vector of site names to track
   */
  HybridKinematicsTest(int robot_number, const std::vector<std::string> &site_names);

  /**
   * @brief Destructor
   */
  ~HybridKinematicsTest();

  /**
   * @brief Initialize the test environment
   * @return true if initialization successful, false otherwise
   */
  bool initialize();

  /**
   * @brief Run kinematics comparison test
   * @param scenario Test scenario: 0=zero angles (5 steps), 1=lerp motion (10s)
   * @param compareJacobian If true, compare Jacobians; if false, compare EE poses (default)
   * @return true if all tests pass, false otherwise
   */
  bool runKineComparison(int scenario = 0, bool compareJacobian = false);

  /**
   * @brief Run multi-site kinematics comparison test
   * @param scenario Test scenario: 0=zero angles (5 steps), 1=lerp motion (10s)
   * @return true if all tests pass, false otherwise
   */
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
  /**
   * @brief Load MuJoCo model and data
   * @return true if successful, false otherwise
   */
  bool loadMujocoModel();

  bool initializePinocchio();

  Eigen::VectorXd genJointRef(double time, int scenario, double durationSeconds);
  bool getMujocoEEPose(const Eigen::VectorXd &q, Eigen::Vector3d &pos, Eigen::Vector4d &quat);
  bool getPinEEPose(const Eigen::VectorXd &q, Eigen::Vector3d &pos, Eigen::Vector4d &quat);
  bool getMujocoJaco(const Eigen::VectorXd &q, Eigen::MatrixXd &jaco);
  bool getPinJaco(const Eigen::VectorXd &q, Eigen::MatrixXd &jaco);
  bool comparePoses(const Eigen::Vector3d &pos1, const Eigen::Vector4d &quat1,
                    const Eigen::Vector3d &pos2, const Eigen::Vector4d &quat2);
  bool compareJacobians(const Eigen::MatrixXd &jaco1, const Eigen::MatrixXd &jaco2);

  // Multi-site methods
  bool getMujocoSitePose(const Eigen::VectorXd &q, int siteIndex, Eigen::Vector3d &pos, Eigen::Vector4d &quat);
  bool getPinSitePose(const Eigen::VectorXd &q, int siteIndex, Eigen::Vector3d &pos, Eigen::Vector4d &quat);

  bool initRendering();
  void callRendering();
  void closeRendering();
  void printResults();

private:
  mjModel *mjModel_;
  mjData *mjData_;
  std::string mjcfPath_;

  GLFWwindow *glfwWindow_;
  mjvCamera mjCamera_;
  mjvOption mjvOption_;
  mjvScene mjScene_;
  mjrContext mjrContext_;
  bool enableRendering_;
  bool renderingInitialized_;

  std::unique_ptr<utils::PinKine> pinKine_;
  std::unique_ptr<rynn::RobotManager> robotManager_;

  const int robotNumber_;
  double posTolerance_;
  double quatTolerance_;
  double jacoTolerance_;
  int numJoints_;

  int testsPassed_;
  int testsFailed_;
  double maxPosError_;
  double maxQuatError_;

  int eeSiteId_;
  double simTimestep_;
  Eigen::VectorXd targetJointAngles_;

  // Multi-site support
  bool multiSiteMode_;
  std::vector<std::string> siteNames_;
  std::vector<int> siteIds_;
};

} // namespace test
