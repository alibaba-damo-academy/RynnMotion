#include "pickplace_fsm.hpp"

#include <iostream>

namespace rynn {

namespace {
constexpr double POSE_TOLERANCE = 1e-3;
constexpr double ANGLE_TOLERANCE = 1e-3;
constexpr double PRE_PICK_HEIGHT_OFFSET = 0.1;
} // namespace

PickPlaceFsm::PickPlaceFsm(int armIndex) : armIndex_(armIndex) {}

void PickPlaceFsm::start() {
  currentState_ = PickPlaceStateID::Idle;
  ctx_ = Context{};
}

void PickPlaceFsm::dispatch(const PickPlaceEventTick &e) {
  if (currentState_ != PickPlaceStateID::Idle) {
    onTick(e.duration);
  }
}

void PickPlaceFsm::dispatch(const PickPlaceEventStart &e) {
  if (currentState_ != PickPlaceStateID::Idle) return;

  ctx_.standPose = e.standPose;
  ctx_.placePose = e.placePose;
  ctx_.graspPoses = e.graspPoses;
  ctx_.numObjects = static_cast<int>(e.graspPoses.size());
  ctx_.currentObject = 0;
  ctx_.stateTimer = 0.0f;
  ctx_.totalTime = 0.0f;
  ctx_.dt = e.dt;
  ctx_.initialized = true;

  generatePrePoses();
  transitTo(PickPlaceStateID::MoveToPrePick);
}

void PickPlaceFsm::dispatch(const PickPlaceEventReset &) {
  reset();
}

void PickPlaceFsm::reset() {
  currentState_ = PickPlaceStateID::Idle;
  ctx_.initialized = false;
  ctx_.completed = false;
  ctx_.currentObject = 0;
  ctx_.stateTimer = 0.0f;
  ctx_.totalTime = 0.0f;
  ctx_.currentTrajectory.reset();
}

void PickPlaceFsm::setLimits(double velMax, double accMax, double graspTime) {
  ctx_.limits.velMax = velMax;
  ctx_.limits.accMax = accMax;
  ctx_.limits.graspTime = graspTime;
}

void PickPlaceFsm::transitTo(PickPlaceStateID newState) {
  currentState_ = newState;
  onEntry(newState);
}

void PickPlaceFsm::onEntry(PickPlaceStateID state) {
  ctx_.stateTimer = 0.0f;
  ctx_.trajAccumTime = 0.0;

  switch (state) {
  case PickPlaceStateID::Idle:
    break;

  case PickPlaceStateID::MoveToPrePick: {
    data::Pose startPose = (ctx_.currentObject == 0) ? ctx_.standPose : ctx_.placePose;
    createTrajectory(startPose, ctx_.prePoseHeight[ctx_.currentObject]);
    break;
  }

  case PickPlaceStateID::MoveToPick:
    createTrajectory(ctx_.prePoseHeight[ctx_.currentObject], ctx_.graspPoses[ctx_.currentObject]);
    break;

  case PickPlaceStateID::Grasp:
    break;

  case PickPlaceStateID::Lift:
    createTrajectory(ctx_.graspPoses[ctx_.currentObject], ctx_.prePoseHeight[ctx_.currentObject]);
    break;

  case PickPlaceStateID::MoveToPlace:
    createTrajectory(ctx_.prePoseHeight[ctx_.currentObject], ctx_.placePose);
    break;

  case PickPlaceStateID::Drop:
    break;

  case PickPlaceStateID::ReturnToStand:
    createTrajectory(ctx_.placePose, ctx_.standPose);
    break;
  }
}

void PickPlaceFsm::onTick(float dt) {
  ctx_.stateTimer += dt;
  ctx_.totalTime += dt;

  switch (currentState_) {
  case PickPlaceStateID::Idle:
    break;

  case PickPlaceStateID::MoveToPrePick:
  case PickPlaceStateID::MoveToPick:
  case PickPlaceStateID::Lift:
  case PickPlaceStateID::MoveToPlace:
  case PickPlaceStateID::ReturnToStand: {
    updateTrajectory(dt);
    data::Pose targetPose = getTargetPoseForState(currentState_);
    if (isTrajectoryComplete(targetPose)) {
      PickPlaceStateID nextState = getNextState(currentState_);
      if (nextState == PickPlaceStateID::Idle && currentState_ == PickPlaceStateID::ReturnToStand) {
        ctx_.completed = true;
      }
      transitTo(nextState);
    }
    break;
  }

  case PickPlaceStateID::Grasp:
  case PickPlaceStateID::Drop:
    if (ctx_.stateTimer >= ctx_.limits.graspTime) {
      if (currentState_ == PickPlaceStateID::Drop) {
        ctx_.currentObject++;
      }
      transitTo(getNextState(currentState_));
    }
    break;
  }
}

PickPlaceStateID PickPlaceFsm::getNextState(PickPlaceStateID current) const {
  switch (current) {
  case PickPlaceStateID::Idle:
    return PickPlaceStateID::MoveToPrePick;
  case PickPlaceStateID::MoveToPrePick:
    return PickPlaceStateID::MoveToPick;
  case PickPlaceStateID::MoveToPick:
    return PickPlaceStateID::Grasp;
  case PickPlaceStateID::Grasp:
    return PickPlaceStateID::Lift;
  case PickPlaceStateID::Lift:
    return PickPlaceStateID::MoveToPlace;
  case PickPlaceStateID::MoveToPlace:
    return PickPlaceStateID::Drop;
  case PickPlaceStateID::Drop:
    if (ctx_.currentObject < ctx_.numObjects) {
      return PickPlaceStateID::MoveToPrePick;
    }
    return PickPlaceStateID::ReturnToStand;
  case PickPlaceStateID::ReturnToStand:
    return PickPlaceStateID::Idle;
  }
  return PickPlaceStateID::Idle;
}

void PickPlaceFsm::generatePrePoses() {
  ctx_.prePoseHeight.clear();
  for (const auto &objPose : ctx_.graspPoses) {
    data::Pose prePose = objPose;
    prePose.pos.z() += PRE_PICK_HEIGHT_OFFSET;
    ctx_.prePoseHeight.push_back(prePose);
  }
}

bool PickPlaceFsm::createTrajectory(const data::Pose &start, const data::Pose &end) {
  ctx_.currentTrajectory = std::make_unique<EEPoseTrajGen>(ctx_.dt);
  double velLimit = 4.0 * ctx_.limits.velMax;
  double accLimit = 2.0 * ctx_.limits.accMax;
  ctx_.currentTrajectory->setTcpLimits(velLimit, accLimit);
  ctx_.currentTrajectory->setStartState(start.pos, start.quat);
  ctx_.currentTrajectory->setTargetState(end.pos, end.quat);
  ctx_.currentTrajectory->update();
  return true;
}

void PickPlaceFsm::updateTrajectory(float dt) {
  if (!ctx_.currentTrajectory) return;

  ctx_.trajAccumTime += dt;
  while (ctx_.trajAccumTime >= ctx_.dt) {
    ctx_.currentTrajectory->update();
    ctx_.currentTrajectory->passToInput();
    ctx_.trajAccumTime -= ctx_.dt;
  }
}

bool PickPlaceFsm::isTrajectoryComplete(const data::Pose &targetPose) const {
  if (!ctx_.currentTrajectory) return false;

  auto [currPos, currQuat] = ctx_.currentTrajectory->getCurrPose();
  double posDistance = (currPos - targetPose.pos).norm();

  Eigen::Quaterniond quatDiff = currQuat.inverse() * targetPose.quat;
  double angleDistance = 2.0 * std::acos(std::abs(quatDiff.w()));

  return (posDistance < POSE_TOLERANCE) && (angleDistance < ANGLE_TOLERANCE);
}

data::Pose PickPlaceFsm::getTargetPoseForState(PickPlaceStateID state) const {
  data::Pose targetPose;
  targetPose.quat = ctx_.standPose.quat;

  switch (state) {
  case PickPlaceStateID::Idle:
    targetPose = ctx_.standPose;
    break;
  case PickPlaceStateID::MoveToPrePick:
    targetPose = ctx_.prePoseHeight[ctx_.currentObject];
    break;
  case PickPlaceStateID::MoveToPick:
  case PickPlaceStateID::Grasp:
    targetPose = ctx_.graspPoses[ctx_.currentObject];
    break;
  case PickPlaceStateID::Lift:
    targetPose = ctx_.prePoseHeight[ctx_.currentObject];
    break;
  case PickPlaceStateID::MoveToPlace:
  case PickPlaceStateID::Drop:
    targetPose = ctx_.placePose;
    break;
  case PickPlaceStateID::ReturnToStand:
    targetPose = ctx_.standPose;
    break;
  }

  return targetPose;
}

data::Pose PickPlaceFsm::getCurrentTargetPose() const {
  if (ctx_.currentTrajectory) {
    auto [pos, quat] = ctx_.currentTrajectory->getCurrPose();
    data::Pose pose;
    pose.pos = pos;
    pose.quat = quat;
    return pose;
  }
  return getTargetPoseForState(currentState_);
}

bool PickPlaceFsm::shouldOpenGripper() const {
  switch (currentState_) {
  case PickPlaceStateID::Idle:
  case PickPlaceStateID::MoveToPrePick:
  case PickPlaceStateID::MoveToPick:
  case PickPlaceStateID::Drop:
  case PickPlaceStateID::ReturnToStand:
    return true;
  case PickPlaceStateID::Grasp:
  case PickPlaceStateID::Lift:
  case PickPlaceStateID::MoveToPlace:
    return false;
  }
  return true;
}

} // namespace rynn
