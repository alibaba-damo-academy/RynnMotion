#pragma once

#include <Eigen/Dense>
#include <memory>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/spatial/log.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "orient_tools.hpp"

namespace utils {

/**
 * @class PinKine
 * @brief Class to solve kinematics problem using Pinocchio library
 * @details This class provides a drop-in replacement for RbdlKine using Pinocchio.
 *          It maintains the same API interface for easy migration from RBDL.
 */
class PinKine {
public:
  /**
   * @brief Default constructor with MJCF path only
   * @param mjcfPath Path to the MJCF file
   * @note No end-effector sites are initialized in this mode
   */
  explicit PinKine(const std::string &mjcfPath);

  /**
   * @brief Constructor for single end-effector site
   * @param mjcfPath Path to the MJCF file
   * @param eeSiteName Name of the end-effector site (default: "EE")
   */
  PinKine(const std::string &mjcfPath,
          const std::string &eeSiteName);

  /**
   * @brief Constructor for multiple end-effector sites (multi-site mode)
   * @param mjcfPath Path to the MJCF file
   * @param siteNames Vector of end-effector site names (empty vector → no-site mode)
   * @note If siteNames is empty, automatically falls back to no-site mode
   */
  PinKine(const std::string &mjcfPath,
          const std::vector<std::string> &siteNames);

  /**
   * @brief Destructor for PinKine
   */
  ~PinKine() = default;

  void update(const Eigen::VectorXd &q);
  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot);

  /**
   * @brief Solve the inverse kinematics problem for a desired end-effector pose.
   *
   * Overloaded functions to solve IK for different representations of the end-effector pose.
   *
   * @param desiredPose Desired end-effector pose as a 4x4 transformation matrix.
   * @param eePos Desired end-effector position as a 3D vector.
   * @param eeQuat Desired end-effector orientation as a quaternion.
   * @param eeRpy Desired end-effector orientation as roll-pitch-yaw angles.
   * @param q_ref Reference joint position.
   * @return Joint position vector solved by the inverse kinematics.
   */
  Eigen::VectorXd ikPos(const Eigen::Matrix4d &desiredPose, const Eigen::VectorXd &q_ref);
  Eigen::VectorXd ikPos(const Eigen::Vector3d &eePos, const Eigen::Quaterniond &eeQuat, const Eigen::VectorXd &q_ref);
  Eigen::VectorXd ikPos(const Eigen::Vector3d &eePos, const Eigen::Vector4d &eeQuat, const Eigen::VectorXd &q_ref);
  Eigen::VectorXd ikPos(const Eigen::Vector3d &eePos, const Eigen::Vector3d &eeRpy, const Eigen::VectorXd &q_ref);

  /**
   * @brief Get the 4x4 homogeneous transformation matrix of the end-effector
   * @return 4x4 transformation matrix
   */
  const Eigen::Matrix4d getEET() const;

  /**
   * @brief Get the 6xDOF Jacobian matrix of the end-effector
   * @param eeIndex Index of the end-effector (default: 0 for first/only EE)
   * @return Jacobian matrix with linear velocity (rows 0-2) on top and angular velocity (rows 3-5) on bottom
   * @note Uses LOCAL_WORLD_ALIGNED reference frame, which is the default format for Pinocchio
   */
  const Eigen::MatrixXd &getEEJaco(int eeIndex = 0) const;

  /**
   * @brief Get the position of the end-effector
   * @param eeIndex Index of the end-effector (default: 0 for first/only EE)
   * @return End-effector position vector
   */
  const Eigen::Vector3d &getEEPos(int eeIndex = 0) const;

  /**
   * @brief Get the orientation of the end-effector in roll-pitch-yaw
   * @param eeIndex Index of the end-effector (default: 0 for first/only EE)
   * @return End-effector orientation in roll-pitch-yaw
   */
  Eigen::Vector3d getEERpy(int eeIndex = 0) const;

  Eigen::Vector4d getEEQuat(int eeIndex = 0) const;

  Eigen::Matrix<double, 6, 1> getEEVel(int eeIndex = 0) const;

  /**
   * @brief Get the Pinocchio model for accessing frames and joints
   * @return Reference to the Pinocchio model
   */
  const pinocchio::Model &getPinModel() const;

  // ========== End-Effector Queries ==========
  /**
   * @brief Get the number of end-effectors
   * @return Number of end-effector sites (auto-detected by name)
   */
  int getNumEndEffectors() const;

  /**
   * @brief Check if this PinKine has any end-effectors
   * @return True if at least one end-effector exists
   */
  bool hasEndEffectors() const;

  /**
   * @brief Get the name of a specific end-effector
   * @param eeIndex Index of the end-effector
   * @return End-effector site name (e.g., "EE", "EE_left", "eeSite_01")
   * @throws std::out_of_range if index is invalid
   */
  std::string getEESiteName(int eeIndex = 0) const;

  /**
   * @brief Convert EE index to site index
   * @param eeIndex Index in the end-effector list
   * @return Index in the full site list
   * @throws std::out_of_range if index is invalid
   */
  int getSiteIndexFromEEIndex(int eeIndex) const;

  /**
   * @brief Convert site index to EE index
   * @param siteIndex Index in the full site list
   * @return Index in the end-effector list, or -1 if not an EE
   */
  int getEEIndexFromSiteIndex(int siteIndex) const;

  /**
   * @brief Get the number of end-effector sites
   * @return Number of sites (0 for no-site mode, 1 for single-site mode, N for multi-site mode)
   */
  size_t getNumSites() const;

  /**
   * @brief Get the name of a site by index
   * @param siteIndex Index of the site
   * @return Site name
   * @throws std::out_of_range if index is invalid
   */
  const std::string &getSiteName(size_t siteIndex) const;

  /**
   * @brief Get the index of a site by name
   * @param siteName Name of the site (e.g., "EE_left", "camera_optical", "gripper_tip")
   * @return Site index, or -1 if not found
   */
  int getSiteIndex(const std::string &siteName) const;

  // ========== Site Access by Index ==========
  /**
   * @brief Get the position of a specific site by index
   * @param siteIndex Index of the site (0 for single-site mode)
   * @return Site position vector
   * @throws std::out_of_range if index is invalid
   */
  const Eigen::Vector3d &getSitePos(size_t siteIndex) const;

  /**
   * @brief Get the orientation quaternion of a specific site by index
   * @param siteIndex Index of the site (0 for single-site mode)
   * @return Site orientation as Vector4d in (x,y,z,w) order
   * @throws std::out_of_range if index is invalid
   * @note Returns quaternion in (x,y,z,w) format to match Python community standards
   */
  Eigen::Vector4d getSiteQuat(size_t siteIndex) const;

  /**
   * @brief Get the orientation in roll-pitch-yaw of a specific site by index
   * @param siteIndex Index of the site (0 for single-site mode)
   * @return Site orientation in roll-pitch-yaw
   * @throws std::out_of_range if index is invalid
   */
  Eigen::Vector3d getSiteRpy(size_t siteIndex) const;

  /**
   * @brief Get the 4x4 transformation matrix of a specific site by index
   * @param siteIndex Index of the site (0 for single-site mode)
   * @return Site transformation matrix
   * @throws std::out_of_range if index is invalid
   */
  const Eigen::Matrix4d &getSiteTransform(size_t siteIndex) const;

  /**
   * @brief Get the Jacobian of a specific site by index
   * @param siteIndex Index of the site (0 for single-site mode)
   * @return Site Jacobian matrix
   * @throws std::out_of_range if index is invalid
   */
  const Eigen::MatrixXd &getSiteJacobian(size_t siteIndex) const;

  // ========== Site Access by Name ==========
  /**
   * @brief Get the position of a specific site by name
   * @param siteName Name of the site (e.g., "EE_left", "camera_optical", "gripper_tip")
   * @return Site position vector
   * @throws std::runtime_error if site name not found
   */
  Eigen::Vector3d getSitePosByName(const std::string &siteName) const;

  /**
   * @brief Get the orientation quaternion of a specific site by name
   * @param siteName Name of the site
   * @return Site orientation as Vector4d in (x,y,z,w) order
   * @throws std::runtime_error if site name not found
   * @note Returns quaternion in (x,y,z,w) format to match Python community standards
   */
  Eigen::Vector4d getSiteQuatByName(const std::string &siteName) const;

  /**
   * @brief Get the orientation in roll-pitch-yaw of a specific site by name
   * @param siteName Name of the site
   * @return Site orientation in roll-pitch-yaw
   * @throws std::runtime_error if site name not found
   */
  Eigen::Vector3d getSiteRpyByName(const std::string &siteName) const;

  /**
   * @brief Get the Jacobian of a specific site by name
   * @param siteName Name of the site
   * @return Site Jacobian matrix
   * @throws std::runtime_error if site name not found
   */
  Eigen::MatrixXd getSiteJacoByName(const std::string &siteName) const;

  /**
   * @brief Get the 4x4 homogeneous transformation matrix of a specific link/frame
   * @param q Joint position vector
   * @param frameName Name of the frame/link
   * @return 4x4 transformation matrix of the specified frame
   * @note This is a stateless method for compatibility with RBDL's getLinkPose interface
   */
  Eigen::Matrix4d getLinkPose(const Eigen::VectorXd &q, const std::string &frameName) const;

private:
  /**
   * @brief Initialize the kinematics model
   */
  void init();

  /**
   * @brief Get the frame ID by its name
   * @param frame_name Name of the frame
   * @return Frame ID
   */
  pinocchio::FrameIndex getFrameIdByName(const std::string &frame_name) const;

  /**
   * @brief Check if a site name indicates an end-effector
   * @param siteName Name of the site
   * @return True if site name starts with "ee" or contains "_ee" (case-insensitive)
   */
  bool _isEndEffectorSite(const std::string &siteName) const;

  /**
   * @brief Identify which sites are end-effectors and populate _eeIndices
   */
  void _identifyEndEffectorSites();

  /**
   * @brief ikPos internal calls (without reference configuration)
   */
  Eigen::VectorXd ikPos(const Eigen::Matrix4d &desiredPose);
  Eigen::VectorXd ikPos(const Eigen::Vector3d &eePos, const Eigen::Quaterniond &eeQuat);
  Eigen::VectorXd ikPos(const Eigen::Vector3d &eePos, const Eigen::Vector4d &eeQuat);
  Eigen::VectorXd ikPos(const Eigen::Vector3d &eePos, const Eigen::Vector3d &eeRpy);

  std::string _mjcfPath;

  // Single-site members (legacy)
  std::string _eeSiteName;
  pinocchio::FrameIndex _eeSiteFrameId;
  Eigen::Vector3d _eePos;
  Eigen::Quaterniond _eeQuat;
  Eigen::Matrix4d _eeT;
  Eigen::MatrixXd _eeJaco;

  std::vector<std::string> _siteNames;
  std::vector<pinocchio::FrameIndex> _siteFrameIds;
  std::vector<Eigen::Vector3d> _sitePos;
  std::vector<Eigen::Quaterniond> _siteQuat;
  std::vector<Eigen::Matrix4d> _siteT;
  std::vector<Eigen::MatrixXd> _siteJaco;
  std::vector<Eigen::Matrix<double, 6, 1>> _siteVel;

  // End-effector tracking (maps EE index → site index)
  std::vector<int> _eeIndices;

  // Pinocchio model and data
  pinocchio::Model _pinModel;
  pinocchio::Data _pinData;
  Eigen::VectorXd _qIkRef_default;

  // IK solver parameters
  static constexpr double _ikStepTol = 1e-6;
  static constexpr double _ikLambda = 0.0001;
  static constexpr int _ikMaxSteps = 100;
};

} // namespace utils
