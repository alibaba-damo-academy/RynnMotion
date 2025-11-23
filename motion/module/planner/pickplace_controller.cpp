#include "pickplace_controller.hpp"

#include <iostream>

#include "orient_tools.hpp"

namespace rynn {

PickPlaceConfig getPickPlaceConfigForRobot(RobotType robotType) {
  PickPlaceConfig config;

  switch (robotType) {
  case RobotType::fr3:
  case RobotType::dual_fr3:
    config.prePickHeight = 0.1;
    config.graspHeight = 0.02;
    config.liftHeight = 0.1;
    config.dropHeight = 0.2;
    config.graspTime = 0.3;
    break;

  case RobotType::ur5e:
    config.prePickHeight = 0.12;
    config.graspHeight = 0.01;
    config.liftHeight = 0.12;
    config.dropHeight = 0.25;
    config.graspTime = 0.3;
    config.maxReachDistance = 1.2;
    break;

  case RobotType::piper:
    config.prePickHeight = 0.1;
    config.graspHeight = 0.00;
    config.liftHeight = 0.1;
    config.dropHeight = 0.1;
    config.graspTime = 0.3;
    break;

  case RobotType::rm75:
    config.prePickHeight = 0.08;
    config.graspHeight = 0.005;
    config.liftHeight = 0.08;
    config.dropHeight = 0.15;
    config.graspTime = 0.3;
    config.maxReachDistance = 0.8;
    break;

  case RobotType::so101:
    config.prePickHeight = 0.08;
    config.graspHeight = 0.005;
    config.liftHeight = 0.08;
    config.dropHeight = 0.15;
    config.graspTime = 0.35;
    config.maxReachDistance = 0.8;
    break;

  default:
    std::cerr << "Warning: Unknown robot type " << static_cast<int>(robotType)
              << ", using FR3 defaults" << std::endl;
    break;
  }

  return config;
}

PickPlaceController::PickPlaceController(RobotType robotType,
                                         int armIndex,
                                         const PickPlaceConfig *config) :
    _robotType(robotType),
    _armIndex(armIndex) {
  if (config) {
    _config = *config;
  } else {
    _config = getPickPlaceConfigForRobot(robotType);
  }
}

bool PickPlaceController::initialize(const data::Pose &standPose,
                                     const data::Pose &dropPose,
                                     const std::vector<data::Pose> &objectPoses) {
  if (objectPoses.empty()) {
    std::cerr << "[PickPlaceController] Error: No objects to pick" << std::endl;
    return false;
  }

  _standPose = standPose;
  _dropPose = dropPose;
  _objectPoses = objectPoses;

  if (!computeAllPoses()) {
    std::cerr << "[PickPlaceController] Error: Failed to compute poses" << std::endl;
    return false;
  }

  constexpr double TRAJ_DT = 0.001;
  auto startEvent = rynn::PickPlaceEventStart{_standPose, _dropPose, _graspPoses, TRAJ_DT};

  if (_armIndex == 0) {
    rynn::PickPlaceFsm<0>::start();
    rynn::PickPlaceFsm<0>::dispatch(startEvent);

    rynn::PickPlaceFsm<0>::ctx_.limits.velMax = _config.velMax;
    rynn::PickPlaceFsm<0>::ctx_.limits.accMax = _config.accMax;
    rynn::PickPlaceFsm<0>::ctx_.limits.graspTime = _config.graspTime;
    rynn::PickPlaceFsm<0>::ctx_.printDebug = _config.printDebug;
  } else {
    rynn::PickPlaceFsm<1>::start();
    rynn::PickPlaceFsm<1>::dispatch(startEvent);

    rynn::PickPlaceFsm<1>::ctx_.limits.velMax = _config.velMax;
    rynn::PickPlaceFsm<1>::ctx_.limits.accMax = _config.accMax;
    rynn::PickPlaceFsm<1>::ctx_.limits.graspTime = _config.graspTime;
    rynn::PickPlaceFsm<1>::ctx_.printDebug = _config.printDebug;
  }

  _initialized = true;
  _currentObjectIndex = 0;
  _currentRetryCount = 0;

  std::cout << "[PickPlaceController] Initialized with " << objectPoses.size()
            << " objects (Robot: " << static_cast<int>(_robotType) << ")" << std::endl;

  return true;
}

PickPlaceController::Command PickPlaceController::update(double dt) {
  Command cmd;

  if (!_initialized) {
    std::cerr << "[PickPlaceController] Error: Not initialized" << std::endl;
    cmd.statusMessage = "Not initialized";
    return cmd;
  }

  if (_armIndex == 0) {
    rynn::PickPlaceFsm<0>::dispatch(rynn::PickPlaceEventTick{static_cast<float>(dt)});
    cmd.targetPose = rynn::PickPlaceFsm<0>::getCurrentTargetPose();
    cmd.openGripper = rynn::PickPlaceFsm<0>::shouldOpenGripper();
    cmd.isCompleted = rynn::PickPlaceFsm<0>::isCompleted();

    auto &ctx = rynn::PickPlaceFsm<0>::ctx_;
    if (cmd.isCompleted) {
      cmd.statusMessage = "Sequence completed";
    } else {
      cmd.statusMessage = "Object " + std::to_string(ctx.currentObject + 1) + "/" + std::to_string(ctx.numObjects);
    }
  } else {
    rynn::PickPlaceFsm<1>::dispatch(rynn::PickPlaceEventTick{static_cast<float>(dt)});
    cmd.targetPose = rynn::PickPlaceFsm<1>::getCurrentTargetPose();
    cmd.openGripper = rynn::PickPlaceFsm<1>::shouldOpenGripper();
    cmd.isCompleted = rynn::PickPlaceFsm<1>::isCompleted();

    auto &ctx = rynn::PickPlaceFsm<1>::ctx_;
    if (cmd.isCompleted) {
      cmd.statusMessage = "Sequence completed";
    } else {
      cmd.statusMessage = "Object " + std::to_string(ctx.currentObject + 1) + "/" + std::to_string(ctx.numObjects);
    }
  }

  return cmd;
}

void PickPlaceController::reset() {
  if (_armIndex == 0) {
    rynn::PickPlaceFsm<0>::reset();
  } else {
    rynn::PickPlaceFsm<1>::reset();
  }

  _initialized = false;
  _currentObjectIndex = 0;
  _currentRetryCount = 0;
  _graspPoses.clear();
  _prePickPoses.clear();
  _liftPoses.clear();
}

bool PickPlaceController::isComplete() const {
  if (!_initialized) {
    return false;
  }

  if (_armIndex == 0) {
    return rynn::PickPlaceFsm<0>::isCompleted();
  } else {
    return rynn::PickPlaceFsm<1>::isCompleted();
  }
}

bool PickPlaceController::computeAllPoses() {
  _graspPoses.clear();
  _prePickPoses.clear();
  _liftPoses.clear();

  for (const auto &objectPose : _objectPoses) {
    data::Pose graspPose = computeGraspPose(objectPose);
    data::Pose prePickPose = computePrePickPose(graspPose);
    data::Pose liftPose = computeLiftPose(graspPose);

    _graspPoses.push_back(graspPose);
    _prePickPoses.push_back(prePickPose);
    _liftPoses.push_back(liftPose);
  }

  return !_graspPoses.empty();
}

bool PickPlaceController::isObjectReachable(const data::Pose &objectPose) const {
  double distance = (objectPose.pos - _standPose.pos).norm();
  return distance <= _config.maxReachDistance;
}

data::Pose PickPlaceController::computeGraspPose(const data::Pose &objectPose) const {
  data::Pose graspPose;

  graspPose.pos = objectPose.pos;
  graspPose.pos.z() = _config.graspHeight;
  graspPose.quat = _standPose.quat;

  return graspPose;
}

data::Pose PickPlaceController::computePrePickPose(const data::Pose &graspPose) const {
  data::Pose prePickPose = graspPose;
  prePickPose.pos.z() += _config.prePickHeight;
  return prePickPose;
}

data::Pose PickPlaceController::computeLiftPose(const data::Pose &graspPose) const {
  data::Pose liftPose = graspPose;
  liftPose.pos.z() += _config.liftHeight;
  return liftPose;
}

bool PickPlaceController::verifyGraspSuccess() {
  return true;
}

bool PickPlaceController::handleGraspFailure() {
  _currentRetryCount++;

  if (_currentRetryCount < _config.maxGraspRetries) {
    std::cout << "[PickPlaceController] Grasp failed, retry "
              << _currentRetryCount << "/" << _config.maxGraspRetries << std::endl;
    return true;
  }

  if (_config.skipFailedObjects) {
    std::cout << "[PickPlaceController] Grasp failed, skipping object" << std::endl;
    _currentRetryCount = 0;
    return false; // Skip to next object
  }

  std::cerr << "[PickPlaceController] Grasp failed, aborting sequence" << std::endl;
  return false;
}

} // namespace rynn
