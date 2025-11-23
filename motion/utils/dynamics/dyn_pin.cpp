#include "dyn_pin.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/mjcf.hpp>

namespace utils {

PinDynamics::PinDynamics(const std::string &mjcf_path, bool floating_base) :
    kinematics_calculated_(false),
    dynamics_calculated_(false),
    ee_frame_id_(0) {
  initFromMJCF(mjcf_path, floating_base);
  robotData = std::make_shared<pinocchio::Data>(*robotModel);
  setupInitConfig();
  Reset();
}

void PinDynamics::initFromMJCF(const std::string &mjcf_path, bool floating_base) {
  robotModel = std::make_shared<pinocchio::Model>();

  if (floating_base) {
    pinocchio::mjcf::buildModel(mjcf_path, pinocchio::JointModelFreeFlyer(), *robotModel);
  } else {
    pinocchio::mjcf::buildModel(mjcf_path, *robotModel);
  }
  robotModel->gravity.linear(Eigen::Vector3d(0., 0., -9.80665));

  dof_count_ = robotModel->nv;
  if (robotModel->nframes > 0) {
    ee_frame_id_ = robotModel->nframes - 1;
  }
}

void PinDynamics::setupInitConfig() {
  q = Eigen::VectorXd::Zero(robotModel->nq);
  qdot = Eigen::VectorXd::Zero(robotModel->nv);
  tau = Eigen::VectorXd::Zero(robotModel->nv);
  qddot = Eigen::VectorXd::Zero(robotModel->nv);
}

void PinDynamics::Reset() {
  kinematics_calculated_ = false;
  dynamics_calculated_ = false;
}

void PinDynamics::computeKinematics() {
  if (!kinematics_calculated_) {
    pinocchio::forwardKinematics(*robotModel, *robotData, q, qdot, qddot);
    pinocchio::updateFramePlacements(*robotModel, *robotData);
    kinematics_calculated_ = true;
  }
}

void PinDynamics::computeDynamics() {
  if (!dynamics_calculated_) {
    computeKinematics();
    dynamics_calculated_ = true;
  }
}

unsigned int PinDynamics::GetLinkIdByName(const std::string &link_name) const {
  if (robotModel->existFrame(link_name)) {
    return robotModel->getFrameId(link_name);
  }
  return 0;
}

bool PinDynamics::getGravity(Eigen::VectorXd &grav) {
  Eigen::VectorXd zero_qdot = Eigen::VectorXd::Zero(robotModel->nv);
  Eigen::VectorXd zero_qddot = Eigen::VectorXd::Zero(robotModel->nv);

  grav = pinocchio::rnea(*robotModel, *robotData, q, zero_qdot, zero_qddot);
  return true;
}

bool PinDynamics::getCoriolis(Eigen::VectorXd &coriolis) {
  Eigen::VectorXd zero_qddot = Eigen::VectorXd::Zero(robotModel->nv);
  Eigen::VectorXd grav;

  getGravity(grav);
  Eigen::VectorXd coriolis_plus_grav = pinocchio::rnea(*robotModel, *robotData, q, qdot, zero_qddot);
  coriolis = coriolis_plus_grav - grav;
  return true;
}

Eigen::VectorXd PinDynamics::computeID(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, const Eigen::VectorXd &qddot) {
  if (q.size() != robotModel->nq || qdot.size() != robotModel->nv || qddot.size() != robotModel->nv) {
    throw std::runtime_error("Dimension mismatch in computeID");
  }
  Eigen::VectorXd tau = pinocchio::rnea(*robotModel, *robotData, q, qdot, qddot);
  return tau;
}

Eigen::VectorXd PinDynamics::computeFD(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, const Eigen::VectorXd &tau) {
  Eigen::VectorXd qddot = pinocchio::aba(*robotModel, *robotData, q, qdot, tau);
  return qddot;
}

bool PinDynamics::computeJacobian(const Eigen::VectorXd &q, Eigen::MatrixXd &jacobian) {
  jacobian.resize(6, dof_count_);
  jacobian.setZero();

  pinocchio::forwardKinematics(*robotModel, *robotData, q);
  pinocchio::updateFramePlacements(*robotModel, *robotData);
  pinocchio::getFrameJacobian(*robotModel, *robotData, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

  return true;
}

bool PinDynamics::nonlinearEffects(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, Eigen::VectorXd &nonlinear) {
  Eigen::VectorXd zero_qddot = Eigen::VectorXd::Zero(robotModel->nv);
  nonlinear = pinocchio::rnea(*robotModel, *robotData, q, qdot, zero_qddot);
  pinocchio::forwardKinematics(*robotModel, *robotData, q, qdot, qddot);

  return true;
}

bool PinDynamics::getMassMatrix(const Eigen::VectorXd &q, Eigen::MatrixXd &mass_matrix) {
  mass_matrix.resize(dof_count_, dof_count_);
  pinocchio::crba(*robotModel, *robotData, q);
  mass_matrix = robotData->M;

  return true;
}

bool PinDynamics::getEEPos(const Eigen::VectorXd &q, Eigen::Vector3d &position) {
  pinocchio::forwardKinematics(*robotModel, *robotData, q);
  pinocchio::updateFramePlacements(*robotModel, *robotData);
  position = robotData->oMf[ee_frame_id_].translation();

  return true;
}

} // namespace utils
