#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

// Include Pinocchio headers
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace utils {

/**
 * @class PinDynamics
 * @brief Class for robot dynamics calculation using Pinocchio
 */
class PinDynamics {
public:
  PinDynamics(const std::string &mjcf_path, bool floating_base = false);
  ~PinDynamics() = default;

  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
  Eigen::VectorXd qddot;

  // function declaration
  void setupInitConfig();
  bool getGravity(Eigen::VectorXd &grav);
  bool getCoriolis(Eigen::VectorXd &coriolis);

  /**
   * @brief Simplified inverse dynamics calculation
   * Calculate required joint torques for given motion
   * @param q Joint positions vector
   * @param qdot Joint velocities vector
   * @param qddot Joint accelerations vector
   * @return Joint torques vector
   */
  Eigen::VectorXd computeID(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, const Eigen::VectorXd &qddot);

  /**
   * @brief Simplified forward dynamics calculation
   * Calculate joint accelerations for given joint torques
   * @param q Joint positions vector
   * @param qdot Joint velocities vector
   * @param tau Joint torques vector
   * @return Joint accelerations vector
   */
  Eigen::VectorXd computeFD(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, const Eigen::VectorXd &tau);

  /**
   * @brief computeJacobian
   * Calculate the Jacobian matrix for the end effector
   * @param q Joint positions
   * @param jacobian Output Jacobian matrix (6 x dof)
   * @return true if calculation successful
   */
  bool computeJacobian(const Eigen::VectorXd &q, Eigen::MatrixXd &jacobian);

  /**
   * @brief nonlinearEffects
   * Calculate the nonlinear effects (Coriolis, centrifugal, and gravity)
   * @param q Joint positions
   * @param qdot Joint velocities
   * @param nonlinear Output nonlinear effects
   * @return true if calculation successful
   */
  bool nonlinearEffects(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, Eigen::VectorXd &nonlinear);

  /**
   * @brief getMassMatrix
   * Calculate the mass matrix
   * @param q Joint positions
   * @param mass_matrix Output mass matrix
   * @return true if calculation successful
   */
  bool getMassMatrix(const Eigen::VectorXd &q, Eigen::MatrixXd &mass_matrix);

  /**
   * @brief getEEPos
   * Calculate the position of the end effector
   * @param q Joint positions
   * @param position Output position vector
   * @return true if calculation successful
   */
  bool getEEPos(const Eigen::VectorXd &q, Eigen::Vector3d &position);

  /**
   * @brief Reset
   * Reset the dynamics model to initial state
   */
  void Reset();

  /**
   * @brief GetLinkIdByName
   * Get link id in Pinocchio from link name in urdf
   * @param link_name Name of the link in URDF
   * @return Link ID (frame ID in Pinocchio)
   */
  unsigned int GetLinkIdByName(const std::string &link_name) const;

  // class pointers
  std::shared_ptr<pinocchio::Model> robotModel;
  std::shared_ptr<pinocchio::Data> robotData;

private:
  int dof_count_;
  bool kinematics_calculated_;
  bool dynamics_calculated_;
  int ee_frame_id_;

  void computeKinematics();
  void computeDynamics();
  void initFromMJCF(const std::string &mjcf_path, bool floating_base);
};

} // namespace utils
