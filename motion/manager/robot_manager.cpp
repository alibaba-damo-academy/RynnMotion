#include "robot_manager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <unordered_map>

#ifdef __APPLE__
#include "mujoco.h"
#else
#include "mujoco/mujoco.h"
#endif

#include "debug_config.hpp"
#include "discovery.hpp"
#include "mjcf_parser.hpp"

namespace rynn {

RobotManager::RobotManager(int robotNumber) :
    _robotNumber(robotNumber) {
  loadMjcfPaths();
  MjcfParser parser(*this);
}

RobotManager::RobotManager(const std::string &mjcfPath) :
    _robotNumber(-1),
    _robotMjcfPath(mjcfPath),
    _pinoMjcfPath(mjcfPath) {
  MjcfParser parser(*this);
}

void RobotManager::loadMjcfPaths() {
  auto &discovery = RobotDiscovery::getInstance();

  static bool initialized = false;
  if (!initialized) {
    discovery.scanRobots(MODEL_DIR);
    initialized = true;
  }

  try {
    auto robotInfo = discovery.getRobotInfo(_robotNumber);
    _robotMjcfPath = robotInfo.robotMjcfPath;
    _pinoMjcfPath = robotInfo.pinocchioPath;
  } catch (const std::exception &e) {
    std::cerr << "Warning: Robot " << _robotNumber << " not found in discovery system: "
              << e.what() << std::endl;
  }
}

std::string RobotManager::getRobotMJCF() const {
  return _robotMjcfPath;
}

std::string RobotManager::getPinoMJCF() const {
  return _pinoMjcfPath;
}

int RobotManager::getRobotNumber() const {
  return _robotNumber;
}

rynn::RobotType RobotManager::getRobotType() const {
  return static_cast<RobotType>(_robotNumber);
}

int RobotManager::getMotionDOF() const {
  return _mdof;
}

int RobotManager::getActionDOF() const {
  return _adof;
}

void RobotManager::setMotionDOF(int mdof) {
  _mdof = mdof;
}

void RobotManager::setActionDOF(int adof) {
  _adof = adof;
}

std::vector<int> RobotManager::getEEJointIndices() const {
  return _eeJointIndices;
}

void RobotManager::setEEJointIndices(const std::vector<int> &indices) {
  _eeJointIndices = indices;
}

int RobotManager::getNumEndEffectors() const {
  return static_cast<int>(_eeRanges.size());
}

std::vector<std::pair<double, double>> RobotManager::getEERanges() const {
  return _eeRanges;
}

void RobotManager::setEERanges(const std::vector<std::pair<double, double>> &ranges) {
  _eeRanges = ranges;
}

double RobotManager::normalizeEECommand(int eeIndex, double normalizedCommand) const {
  if (eeIndex < 0 || eeIndex >= static_cast<int>(_eeRanges.size())) {
    std::cerr << "Warning: Invalid EE index " << eeIndex << ", using default range [0, 255]" << std::endl;
    return normalizedCommand * 255.0;
  }

  const auto &range = _eeRanges[eeIndex];
  return range.first + normalizedCommand * (range.second - range.first);
}

void RobotManager::setEndEffectorConfig(int adof,
                                        const std::vector<int> &eeJointIndices,
                                        const std::vector<std::pair<double, double>> &eeRanges) {
  _adof = adof;
  _eeJointIndices = eeJointIndices;
  _eeRanges = eeRanges;
}

Eigen::VectorXd RobotManager::getSimKp() const {
  return _simKp;
}

Eigen::VectorXd RobotManager::getSimKd() const {
  return _simKd;
}

Eigen::VectorXd RobotManager::getRealKp() const {
  return Eigen::VectorXd::Constant(_mdof, 1.0);
}

Eigen::VectorXd RobotManager::getRealKd() const {
  return Eigen::VectorXd::Constant(_mdof, 0.0);
}

Eigen::VectorXd RobotManager::getJointPosMin() const {
  return _qPosMin;
}

Eigen::VectorXd RobotManager::getJointPosMax() const {
  return _qPosMax;
}

Eigen::VectorXd RobotManager::getJointVelMax() const {
  return _qVelMax;
}

Eigen::VectorXd RobotManager::getJointTorqueMax() const {
  return _qTorqueMax;
}

void RobotManager::setJointLimits(const Eigen::VectorXd &qPosMin, const Eigen::VectorXd &qPosMax,
                                  const Eigen::VectorXd &qVelMax, const Eigen::VectorXd &qTorqueMax) {
  _qPosMin = qPosMin;
  _qPosMax = qPosMax;
  _qVelMax = qVelMax;
  _qTorqueMax = qTorqueMax;
}

Eigen::VectorXd RobotManager::getQStandby(int index) const {
  int keyframeIndex = 1 + index;
  if (keyframeIndex >= static_cast<int>(_keyframes.size())) {
    // std::cerr << "Warning: Standby keyframe " << index << " not available, returning zero vector" << std::endl;
    return Eigen::VectorXd::Zero(_mdof);
  }
  return _keyframes[keyframeIndex];
}

Eigen::VectorXd RobotManager::getKeyframe(int index) const {
  if (index < 0 || index >= static_cast<int>(_keyframes.size())) {
    std::cerr << "Warning: Keyframe index " << index << " out of range [0, "
              << _keyframes.size() << "), returning zero vector" << std::endl;
    return Eigen::VectorXd::Zero(_mdof);
  }
  return _keyframes[index];
}

int RobotManager::getNumKeyframes() const {
  return static_cast<int>(_keyframes.size());
}

void RobotManager::setKeyframes(const EigenVecXd &keyframes,
                                const std::vector<std::string> &names) {
  _keyframes = keyframes;
  _keyframeNames = names;

  if (!keyframes.empty()) {
    DEBUG_LOG("Loaded " << keyframes.size() << " keyframe(s) from MJCF:");
    for (size_t i = 0; i < std::min(keyframes.size(), names.size()); ++i) {
      DEBUG_LOG("  [" << i << "] " << names[i] << ", q=" << _keyframes[i].transpose());
    }
  }
}

RobotManager::EigenVecXd RobotManager::getAllKeyframes() const {
  return _keyframes;
}

void RobotManager::setActuatorIndices(const std::vector<int> &jointIndices, const std::vector<int> &eeIndices) {
  _jointIndices = jointIndices;
  _eeIndices = eeIndices;
}

void RobotManager::setActuatorModes(const std::vector<ActuatorMode> &modes) {
  _actuatorModes = modes;
}

std::vector<int> RobotManager::getJointIndices() const {
  return _jointIndices;
}

std::vector<int> RobotManager::getEEIndices() const {
  return _eeIndices;
}

std::vector<ActuatorMode> RobotManager::getActuatorModes() const {
  return _actuatorModes;
}

std::vector<std::string> RobotManager::getSiteNames() const {
  return _siteNames;
}

void RobotManager::setSiteNames(const std::vector<std::string> &siteNames) {
  _siteNames = siteNames;
}

int RobotManager::getNumSites() const {
  return static_cast<int>(_siteNames.size());
}

bool RobotManager::hasSites() const {
  return !_siteNames.empty();
}

std::vector<SensorInfo> RobotManager::getForceSensors() const { return _forceSensors; }
std::vector<SensorInfo> RobotManager::getTorqueSensors() const { return _torqueSensors; }
std::vector<SensorInfo> RobotManager::getGyroSensors() const { return _gyroSensors; }
std::vector<SensorInfo> RobotManager::getAccelerometerSensors() const { return _accelSensors; }
std::vector<SensorInfo> RobotManager::getRangefinderSensors() const { return _rangefinderSensors; }
std::vector<SensorInfo> RobotManager::getFramePosSensors() const { return _framePosSensors; }
std::vector<SensorInfo> RobotManager::getFrameQuatSensors() const { return _frameQuatSensors; }
std::vector<SensorInfo> RobotManager::getTouchSensors() const { return _touchSensors; }

bool RobotManager::isDexHand() const { return _isDexHand; }
void RobotManager::setIsDexHand(bool isDexHand) { _isDexHand = isDexHand; }

void RobotManager::setSensors(const std::vector<SensorInfo> &force,
                              const std::vector<SensorInfo> &torque,
                              const std::vector<SensorInfo> &gyro,
                              const std::vector<SensorInfo> &accel,
                              const std::vector<SensorInfo> &rangefinder,
                              const std::vector<SensorInfo> &framePos,
                              const std::vector<SensorInfo> &frameQuat,
                              const std::vector<SensorInfo> &touch) {
  _forceSensors = force;
  _torqueSensors = torque;
  _gyroSensors = gyro;
  _accelSensors = accel;
  _rangefinderSensors = rangefinder;
  _framePosSensors = framePos;
  _frameQuatSensors = frameQuat;
  _touchSensors = touch;
}

int RobotManager::getNumCameras() const { return static_cast<int>(_cameraNames.size()); }
std::vector<std::string> RobotManager::getCameraNames() const { return _cameraNames; }
void RobotManager::setCameraNames(const std::vector<std::string> &cameraNames) { _cameraNames = cameraNames; }

std::vector<std::string> RobotManager::getJointNames() const { return _jointNames; }
std::vector<std::string> RobotManager::getEENames() const { return _eeNames; }
void RobotManager::setActuatorNames(const std::vector<std::string> &jointNames,
                                    const std::vector<std::string> &eeNames) {
  _jointNames = jointNames;
  _eeNames = eeNames;
}

std::vector<std::string> RobotManager::getKeyframeNames() const { return _keyframeNames; }

} // namespace rynn
