#include "kine_pin.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace utils {

PinKine::PinKine(const std::string &mjcfPath) :
    _mjcfPath(mjcfPath) {
  init();
  _qIkRef_default = Eigen::VectorXd::Zero(_pinModel.nq);
}

PinKine::PinKine(const std::string &mjcfPath,
                 const std::string &eeSiteName) :
    _mjcfPath(mjcfPath),
    _eeSiteName(eeSiteName),
    _siteNames({eeSiteName}) {
  init();
  _qIkRef_default = Eigen::VectorXd::Zero(_pinModel.nq);
}

PinKine::PinKine(const std::string &mjcfPath,
                 const std::vector<std::string> &siteNames) :
    _mjcfPath(mjcfPath),
    _siteNames(siteNames.empty() ? std::vector<std::string>{"EE"} : siteNames) {
  init();
  _qIkRef_default = Eigen::VectorXd::Zero(_pinModel.nq);
}

void PinKine::init() {
  try {
    pinocchio::mjcf::buildModel(_mjcfPath, _pinModel);
    _pinData = pinocchio::Data(_pinModel);

    if (_siteNames.empty()) {
      _siteNames.push_back("EE");
    }

    _siteFrameIds.resize(_siteNames.size());
    _sitePos.resize(_siteNames.size());
    _siteQuat.resize(_siteNames.size());
    _siteT.resize(_siteNames.size());
    _siteJaco.resize(_siteNames.size());
    _siteVel.resize(_siteNames.size());

    // Track which sites are valid (exist in Pinocchio model)
    std::vector<bool> validSites(_siteNames.size(), false);
    std::vector<size_t> invalidIndices;

    for (size_t i = 0; i < _siteNames.size(); ++i) {
      try {
        _siteFrameIds[i] = getFrameIdByName(_siteNames[i]);
        _sitePos[i] = Eigen::Vector3d::Zero();
        _siteQuat[i] = Eigen::Quaterniond::Identity();
        _siteT[i] = Eigen::Matrix4d::Identity();
        _siteJaco[i] = Eigen::MatrixXd::Zero(6, _pinModel.nv);
        validSites[i] = true;
      } catch (const std::runtime_error &e) {
        std::cerr << "Warning: Site '" << _siteNames[i]
                  << "' not found in Pinocchio model (from " << _mjcfPath
                  << "), skipping" << std::endl;
        invalidIndices.push_back(i);
        _siteFrameIds[i] = -1; // Mark as invalid
      }
    }

    // Remove invalid sites from all vectors (iterate in reverse to preserve indices)
    for (auto it = invalidIndices.rbegin(); it != invalidIndices.rend(); ++it) {
      size_t idx = *it;
      _siteNames.erase(_siteNames.begin() + idx);
      _siteFrameIds.erase(_siteFrameIds.begin() + idx);
      _sitePos.erase(_sitePos.begin() + idx);
      _siteQuat.erase(_siteQuat.begin() + idx);
      _siteT.erase(_siteT.begin() + idx);
      _siteJaco.erase(_siteJaco.begin() + idx);
    }

    if (!invalidIndices.empty()) {
      std::cerr << "Info: " << invalidIndices.size() << " site(s) were skipped, "
                << _siteNames.size() << " site(s) successfully loaded" << std::endl;
    }

    // Identify end-effector sites
    _identifyEndEffectorSites();

    if (_eeIndices.empty() && !_siteNames.empty()) {
      std::cerr << "Warning: No end-effector sites detected. "
                << "Site names should start with 'EE'/'ee' or contain '_EE'/'_ee'." << std::endl;
    }

  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to initialize Pinocchio model from " + _mjcfPath + ": " + e.what());
  }
}

void PinKine::update(const Eigen::VectorXd &q) {
  if (q.size() != _pinModel.nq) {
    throw std::runtime_error("Joint configuration size mismatch. Expected: " + std::to_string(_pinModel.nq) + ", got: " + std::to_string(q.size()));
  }

  pinocchio::forwardKinematics(_pinModel, _pinData, q);
  pinocchio::updateFramePlacements(_pinModel, _pinData);

  // Multi-site update
  for (size_t i = 0; i < _siteNames.size(); ++i) {
    const pinocchio::SE3 &siteTransform = _pinData.oMf[_siteFrameIds[i]];

    _sitePos[i] = siteTransform.translation();
    _siteQuat[i] = Eigen::Quaterniond(siteTransform.rotation());

    _siteT[i].block<3, 1>(0, 3) = _sitePos[i];
    _siteT[i].block<3, 3>(0, 0) = _siteQuat[i].toRotationMatrix();
    _siteT[i].row(3) << 0, 0, 0, 1;

    pinocchio::computeFrameJacobian(_pinModel, _pinData, q, _siteFrameIds[i],
                                    pinocchio::LOCAL_WORLD_ALIGNED, _siteJaco[i]);
  }
}

void PinKine::update(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot) {
  update(q);

  for (size_t i = 0; i < _siteNames.size(); ++i) {
    _siteVel[i] = _siteJaco[i] * qdot;
  }
}

Eigen::VectorXd PinKine::ikPos(const Eigen::Matrix4d &desiredPose, const Eigen::VectorXd &q_ref) {
  if (q_ref.size() != _pinModel.nq) {
    throw std::runtime_error("Reference configuration size mismatch in ikPos");
  }

  pinocchio::SE3 desiredSE3(desiredPose.block<3, 3>(0, 0), desiredPose.block<3, 1>(0, 3));

  Eigen::VectorXd q = q_ref;
  Eigen::VectorXd qPrev = q_ref;

  for (int iter = 0; iter < _ikMaxSteps; ++iter) {
    pinocchio::forwardKinematics(_pinModel, _pinData, q);
    pinocchio::updateFramePlacements(_pinModel, _pinData);

    const pinocchio::SE3 &currentSE3 = _pinData.oMf[_eeSiteFrameId];

    pinocchio::SE3 errorSE3 = currentSE3.actInv(desiredSE3);
    Eigen::Matrix<double, 6, 1> error = pinocchio::log6(errorSE3).toVector();

    if (error.norm() < _ikStepTol) {
      return q;
    }

    Eigen::MatrixXd J(6, _pinModel.nv);
    pinocchio::computeFrameJacobian(_pinModel, _pinData, q, _eeSiteFrameId, pinocchio::WORLD, J);

    Eigen::MatrixXd JtJ = J.transpose() * J;
    Eigen::MatrixXd damping = _ikLambda * Eigen::MatrixXd::Identity(JtJ.rows(), JtJ.cols());
    Eigen::VectorXd dq = (JtJ + damping).ldlt().solve(J.transpose() * error);

    qPrev = q;
    q = pinocchio::integrate(_pinModel, q, dq);
    q = q.cwiseMax(_pinModel.lowerPositionLimit).cwiseMin(_pinModel.upperPositionLimit);
  }

  std::cerr << "Warning: Pinocchio IK did not converge. Returning previous solution." << std::endl;
  return qPrev;
}

Eigen::VectorXd PinKine::ikPos(const Eigen::Vector3d &eePos, const Eigen::Quaterniond &eeQuat, const Eigen::VectorXd &q_ref) {
  Eigen::Matrix4d desiredPose = Eigen::Matrix4d::Identity();
  desiredPose.block<3, 1>(0, 3) = eePos;
  desiredPose.block<3, 3>(0, 0) = eeQuat.toRotationMatrix();
  return ikPos(desiredPose, q_ref);
}

Eigen::VectorXd PinKine::ikPos(const Eigen::Vector3d &eePos, const Eigen::Vector4d &eeQuat, const Eigen::VectorXd &q_ref) {
  Eigen::Quaterniond quat(eeQuat(3), eeQuat(0), eeQuat(1), eeQuat(2));
  return ikPos(eePos, quat, q_ref);
}

Eigen::VectorXd PinKine::ikPos(const Eigen::Vector3d &eePos, const Eigen::Vector3d &eeRpy, const Eigen::VectorXd &q_ref) {
  Eigen::Matrix4d desiredPose = Eigen::Matrix4d::Identity();
  desiredPose.block<3, 1>(0, 3) = eePos;
  desiredPose.block<3, 3>(0, 0) = rotmx(eeRpy[0]) * rotmy(eeRpy[1]) * rotmz(eeRpy[2]);
  return ikPos(desiredPose, q_ref);
}

Eigen::VectorXd PinKine::ikPos(const Eigen::Matrix4d &desiredPose) {
  return ikPos(desiredPose, _qIkRef_default);
}

Eigen::VectorXd PinKine::ikPos(const Eigen::Vector3d &eePos, const Eigen::Quaterniond &eeQuat) {
  return ikPos(eePos, eeQuat, _qIkRef_default);
}

Eigen::VectorXd PinKine::ikPos(const Eigen::Vector3d &eePos, const Eigen::Vector4d &eeQuat) {
  return ikPos(eePos, eeQuat, _qIkRef_default);
}

Eigen::VectorXd PinKine::ikPos(const Eigen::Vector3d &eePos, const Eigen::Vector3d &eeRpy) {
  return ikPos(eePos, eeRpy, _qIkRef_default);
}

pinocchio::FrameIndex PinKine::getFrameIdByName(const std::string &frame_name) const {
  if (!_pinModel.existFrame(frame_name)) {
    throw std::runtime_error("Frame '" + frame_name + "' not found in Pinocchio model");
  }
  return _pinModel.getFrameId(frame_name);
}

const Eigen::Matrix4d PinKine::getEET() const {
  if (_siteNames.empty()) {
    throw std::runtime_error("Cannot get EE transform in no-site mode");
  }

  if (!_siteNames.empty()) {
    // Use first end-effector (for backward compatibility)
    if (_eeIndices.empty()) {
      throw std::runtime_error("No end-effector sites found");
    }
    int siteIdx = _eeIndices[0];
    return _siteT[siteIdx];
  }
  return _eeT;
}

const Eigen::MatrixXd &PinKine::getEEJaco(int eeIndex) const {
  if (_siteNames.empty()) {
    throw std::runtime_error("Cannot get EE Jacobian in no-site mode");
  }

  if (!_siteNames.empty()) {
    if (eeIndex < 0 || eeIndex >= static_cast<int>(_eeIndices.size())) {
      throw std::out_of_range("EE index " + std::to_string(eeIndex) + " out of range (have " + std::to_string(_eeIndices.size()) + " end-effectors)");
    }
    int siteIdx = _eeIndices[eeIndex];
    return _siteJaco[siteIdx];
  }

  // Single-site mode
  if (eeIndex != 0) {
    throw std::out_of_range("Single-site mode only supports eeIndex=0");
  }
  return _eeJaco;
}

const Eigen::Vector3d &PinKine::getEEPos(int eeIndex) const {
  if (_siteNames.empty()) {
    throw std::runtime_error("Cannot get EE position in no-site mode");
  }

  if (!_siteNames.empty()) {
    if (eeIndex < 0 || eeIndex >= static_cast<int>(_eeIndices.size())) {
      throw std::out_of_range("EE index out of range");
    }
    int siteIdx = _eeIndices[eeIndex];
    return _sitePos[siteIdx];
  }

  // Single-site mode
  if (eeIndex != 0) {
    throw std::out_of_range("Single-site mode only supports eeIndex=0");
  }
  return _eePos;
}

Eigen::Vector3d PinKine::getEERpy(int eeIndex) const {
  if (_siteNames.empty()) {
    throw std::runtime_error("Cannot get EE RPY in no-site mode");
  }

  if (!_siteNames.empty()) {
    if (eeIndex < 0 || eeIndex >= static_cast<int>(_eeIndices.size())) {
      throw std::out_of_range("EE index out of range");
    }
    int siteIdx = _eeIndices[eeIndex];
    return R2rpy(_siteT[siteIdx].block<3, 3>(0, 0));
  }

  // Single-site mode
  if (eeIndex != 0) {
    throw std::out_of_range("Single-site mode only supports eeIndex=0");
  }
  return R2rpy(_eeT.block<3, 3>(0, 0));
}

Eigen::Vector4d PinKine::getEEQuat(int eeIndex) const {
  if (_siteNames.empty()) {
    throw std::runtime_error("Cannot get EE quaternion in no-site mode");
  }

  if (!_siteNames.empty()) {
    if (eeIndex < 0 || eeIndex >= static_cast<int>(_eeIndices.size())) {
      throw std::out_of_range("EE index out of range");
    }
    int siteIdx = _eeIndices[eeIndex];
    // Return in (x,y,z,w) format using coeffs()
    return _siteQuat[siteIdx].coeffs();
  }

  // Single-site mode
  if (eeIndex != 0) {
    throw std::out_of_range("Single-site mode only supports eeIndex=0");
  }
  return _eeQuat.coeffs();
}

Eigen::Matrix<double, 6, 1> PinKine::getEEVel(int eeIndex) const {
  if (_siteNames.empty()) {
    throw std::runtime_error("Cannot get EE velocity in no-site mode");
  }

  if (eeIndex < 0 || eeIndex >= static_cast<int>(_eeIndices.size())) {
    throw std::out_of_range("EE index out of range");
  }

  int siteIdx = _eeIndices[eeIndex];
  return _siteVel[siteIdx];
}

size_t PinKine::getNumSites() const {
  if (_siteNames.empty()) {
    return 0;
  } else if (!_siteNames.empty()) {
    return _siteNames.size();
  } else {
    return 1;
  }
}

const std::string &PinKine::getSiteName(size_t siteIndex) const {
  if (!_siteNames.empty()) {
    if (siteIndex >= _siteNames.size()) {
      throw std::out_of_range("Site index " + std::to_string(siteIndex) + " out of range (have " + std::to_string(_siteNames.size()) + " sites)");
    }
    return _siteNames[siteIndex];
  } else if (_siteNames.empty()) {
    throw std::runtime_error("No sites configured in no-site mode");
  } else {
    if (siteIndex != 0) {
      throw std::out_of_range("Single-site mode only supports index 0");
    }
    return _eeSiteName;
  }
}

int PinKine::getSiteIndex(const std::string &siteName) const {
  if (!_siteNames.empty()) {
    for (size_t i = 0; i < _siteNames.size(); ++i) {
      if (_siteNames[i] == siteName) {
        return static_cast<int>(i);
      }
    }
    return -1; // Not found
  } else if (_siteNames.empty()) {
    return -1; // No sites in no-site mode
  } else {
    return (_eeSiteName == siteName) ? 0 : -1;
  }
}

const Eigen::Vector3d &PinKine::getSitePos(size_t siteIndex) const {
  if (!_siteNames.empty()) {
    if (siteIndex >= _siteNames.size()) {
      throw std::out_of_range("Site index out of range");
    }
    return _sitePos[siteIndex];
  } else {
    if (siteIndex != 0) {
      throw std::out_of_range("Single-site mode only supports index 0");
    }
    return _eePos;
  }
}

Eigen::Vector4d PinKine::getSiteQuat(size_t siteIndex) const {
  if (!_siteNames.empty()) {
    if (siteIndex >= _siteNames.size()) {
      throw std::out_of_range("Site index out of range");
    }
    // Return in (x,y,z,w) format using coeffs()
    return _siteQuat[siteIndex].coeffs();
  } else {
    if (siteIndex != 0) {
      throw std::out_of_range("Single-site mode only supports index 0");
    }
    // Return in (x,y,z,w) format using coeffs()
    return _eeQuat.coeffs();
  }
}

const Eigen::Matrix4d &PinKine::getSiteTransform(size_t siteIndex) const {
  if (!_siteNames.empty()) {
    if (siteIndex >= _siteNames.size()) {
      throw std::out_of_range("Site index out of range");
    }
    return _siteT[siteIndex];
  } else {
    if (siteIndex != 0) {
      throw std::out_of_range("Single-site mode only supports index 0");
    }
    return _eeT;
  }
}

const Eigen::MatrixXd &PinKine::getSiteJacobian(size_t siteIndex) const {
  if (!_siteNames.empty()) {
    if (siteIndex >= _siteNames.size()) {
      throw std::out_of_range("Site index out of range");
    }
    return _siteJaco[siteIndex];
  } else {
    if (siteIndex != 0) {
      throw std::out_of_range("Single-site mode only supports index 0");
    }
    return _eeJaco;
  }
}

Eigen::Vector3d PinKine::getSiteRpy(size_t siteIndex) const {
  if (!_siteNames.empty()) {
    if (siteIndex >= _siteNames.size()) {
      throw std::out_of_range("Site index out of range");
    }
    return R2rpy(_siteT[siteIndex].block<3, 3>(0, 0));
  } else {
    if (siteIndex != 0) {
      throw std::out_of_range("Single-site mode only supports index 0");
    }
    return R2rpy(_eeT.block<3, 3>(0, 0));
  }
}

// ========== Site Access by Name ==========

Eigen::Vector3d PinKine::getSitePosByName(const std::string &siteName) const {
  int index = getSiteIndex(siteName);
  if (index < 0) {
    throw std::runtime_error("Site '" + siteName + "' not found");
  }
  return getSitePos(static_cast<size_t>(index));
}

Eigen::Vector4d PinKine::getSiteQuatByName(const std::string &siteName) const {
  int index = getSiteIndex(siteName);
  if (index < 0) {
    throw std::runtime_error("Site '" + siteName + "' not found");
  }
  return getSiteQuat(static_cast<size_t>(index));
}

Eigen::Vector3d PinKine::getSiteRpyByName(const std::string &siteName) const {
  int index = getSiteIndex(siteName);
  if (index < 0) {
    throw std::runtime_error("Site '" + siteName + "' not found");
  }
  return getSiteRpy(static_cast<size_t>(index));
}

Eigen::MatrixXd PinKine::getSiteJacoByName(const std::string &siteName) const {
  int index = getSiteIndex(siteName);
  if (index < 0) {
    throw std::runtime_error("Site '" + siteName + "' not found");
  }
  return getSiteJacobian(static_cast<size_t>(index));
}

const pinocchio::Model &PinKine::getPinModel() const {
  return _pinModel;
}

Eigen::Matrix4d PinKine::getLinkPose(const Eigen::VectorXd &q, const std::string &frameName) const {
  if (q.size() != _pinModel.nq) {
    throw std::runtime_error("Joint configuration size mismatch in getLinkPose. Expected: " + std::to_string(_pinModel.nq) + ", got: " + std::to_string(q.size()));
  }

  // Create temporary data object for thread-safe computation
  pinocchio::Data tempData(_pinModel);

  // Compute forward kinematics
  pinocchio::forwardKinematics(_pinModel, tempData, q);
  pinocchio::updateFramePlacements(_pinModel, tempData);

  // Get frame ID
  pinocchio::FrameIndex frameId = getFrameIdByName(frameName);

  // Get the SE3 transform
  const pinocchio::SE3 &frameTransform = tempData.oMf[frameId];

  // Convert to 4x4 homogeneous transformation matrix
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = frameTransform.rotation();
  T.block<3, 1>(0, 3) = frameTransform.translation();

  return T;
}

// ========== End-Effector Detection Helper Methods ==========

bool PinKine::_isEndEffectorSite(const std::string &siteName) const {
  std::string lower = siteName;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  // Check if starts with "ee" (case-insensitive)
  // Matches: "EE", "EE_01", "eeSite_01", "EESite_00"
  if (lower.size() >= 2 && lower.substr(0, 2) == "ee") {
    return true;
  }

  // Check if contains "_ee" (after underscore)
  // Matches: "site_EE_01", "left_EE", "right_EE"
  if (lower.find("_ee") != std::string::npos) {
    return true;
  }

  // Avoid false positives like "knee", "steering", "beetleSite"
  return false;
}

void PinKine::_identifyEndEffectorSites() {
  _eeIndices.clear();

  if (_siteNames.empty()) {
    // No sites = no end-effectors
    return;
  } else if (_siteNames.size() == 1) {
    // Single-site mode: the one site is always the EE
    _eeIndices.push_back(0);
  } else {
    // Multi-site mode: auto-detect EE sites
    for (size_t i = 0; i < _siteNames.size(); ++i) {
      if (_isEndEffectorSite(_siteNames[i])) {
        _eeIndices.push_back(static_cast<int>(i));
      }
    }
  }
}

// ========== End-Effector Query Methods ==========

int PinKine::getNumEndEffectors() const {
  return static_cast<int>(_eeIndices.size());
}

bool PinKine::hasEndEffectors() const {
  return !_eeIndices.empty();
}

std::string PinKine::getEESiteName(int eeIndex) const {
  if (eeIndex < 0 || eeIndex >= static_cast<int>(_eeIndices.size())) {
    throw std::out_of_range("EE index " + std::to_string(eeIndex) + " out of range");
  }

  if (!_siteNames.empty()) {
    int siteIdx = _eeIndices[eeIndex];
    return _siteNames[siteIdx];
  } else {
    // Single-site mode
    if (eeIndex != 0) {
      throw std::out_of_range("Single-site mode only supports eeIndex=0");
    }
    return _eeSiteName;
  }
}

int PinKine::getSiteIndexFromEEIndex(int eeIndex) const {
  if (eeIndex < 0 || eeIndex >= static_cast<int>(_eeIndices.size())) {
    throw std::out_of_range("EE index " + std::to_string(eeIndex) + " out of range");
  }
  return _eeIndices[eeIndex];
}

int PinKine::getEEIndexFromSiteIndex(int siteIndex) const {
  for (size_t i = 0; i < _eeIndices.size(); ++i) {
    if (_eeIndices[i] == siteIndex) {
      return static_cast<int>(i);
    }
  }
  return -1; // Not an end-effector
}

} // namespace utils
