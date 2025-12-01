#include "pickplace_controller.hpp"

#include "orient_tools.hpp"
#include "pickplace_fsm.hpp"

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
    break;
  }

  return config;
}

PickPlaceController::PickPlaceController(RobotType robotType, int armIndex, const PickPlaceConfig *config)
    : _armIndex(armIndex) {
  _config = config ? *config : getPickPlaceConfigForRobot(robotType);
}

bool PickPlaceController::initialize(const data::Pose &standPose,
                                     const data::Pose &dropPose,
                                     const std::vector<data::Pose> &objectPoses) {
  if (objectPoses.empty()) return false;

  _standPose = standPose;
  _dropPose = dropPose;
  _objectPoses = objectPoses;

  if (!computeAllPoses()) return false;

  constexpr double TRAJ_DT = 0.001;

  _fsm = std::make_unique<PickPlaceFsm>(_armIndex);
  _fsm->start();
  _fsm->dispatch(PickPlaceEventStart{_standPose, _dropPose, _graspPoses, TRAJ_DT});
  _fsm->setLimits(_config.velMax, _config.accMax, _config.graspTime);
  _fsm->setDebugPrint(_config.printDebug);

  _initialized = true;
  _currentObjectIndex = 0;
  _currentRetryCount = 0;

  return true;
}

PickPlaceController::Command PickPlaceController::update(double dt) {
  Command cmd;

  if (!_initialized || !_fsm) {
    cmd.statusMessage = "Not initialized";
    return cmd;
  }

  _fsm->dispatch(PickPlaceEventTick{static_cast<float>(dt)});
  cmd.targetPose = _fsm->getCurrentTargetPose();
  cmd.openGripper = _fsm->shouldOpenGripper();
  cmd.isCompleted = _fsm->isCompleted();

  const auto &ctx = _fsm->getContext();
  if (cmd.isCompleted) {
    cmd.statusMessage = "Sequence completed";
  } else {
    cmd.statusMessage =
        "Object " + std::to_string(ctx.currentObject + 1) + "/" + std::to_string(ctx.numObjects);
  }

  return cmd;
}

void PickPlaceController::reset() {
  if (_fsm) _fsm->reset();

  _initialized = false;
  _currentObjectIndex = 0;
  _currentRetryCount = 0;
  _graspPoses.clear();
  _prePickPoses.clear();
  _liftPoses.clear();
}

bool PickPlaceController::isComplete() const {
  return _initialized && _fsm && _fsm->isCompleted();
}

bool PickPlaceController::computeAllPoses() {
  _graspPoses.clear();
  _prePickPoses.clear();
  _liftPoses.clear();

  for (const auto &objectPose : _objectPoses) {
    data::Pose graspPose = computeGraspPose(objectPose);
    _graspPoses.push_back(graspPose);
    _prePickPoses.push_back(computePrePickPose(graspPose));
    _liftPoses.push_back(computeLiftPose(graspPose));
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

  if (_currentRetryCount < _config.maxGraspRetries) return true;

  if (_config.skipFailedObjects) {
    _currentRetryCount = 0;
    return false;
  }

  return false;
}

} // namespace rynn
