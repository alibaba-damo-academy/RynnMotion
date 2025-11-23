#include "pickplace_fsm.hpp"

FSM_INITIAL_STATE(rynn::PickPlaceFsm<0>, rynn::PickPlaceIdle<0>)
FSM_INITIAL_STATE(rynn::PickPlaceFsm<1>, rynn::PickPlaceIdle<1>)

namespace rynn {

constexpr double POSE_TOLERANCE = 1e-3;
constexpr double ANGLE_TOLERANCE = 1e-3;

template<> typename PickPlaceFsm<0>::Context PickPlaceFsm<0>::ctx_{};
template<> typename PickPlaceFsm<1>::Context PickPlaceFsm<1>::ctx_{};

template<int ArmIndex>
void PickPlaceFsm<ArmIndex>::reset() {
  tinyfsm::StateList<PickPlaceIdle<ArmIndex>, MoveToPrePick<ArmIndex>, MoveToPick<ArmIndex>, Grasp<ArmIndex>,
                     Lift<ArmIndex>, MoveToPlace<ArmIndex>, Drop<ArmIndex>, ReturnToStand<ArmIndex>>::reset();
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.initialized = false;
  ctx.completed = false;
  ctx.currentObject = 0;
  ctx.stateTimer = 0.0f;
  ctx.totalTime = 0.0f;
  ctx.currentTrajectory.reset();
}

template<int ArmIndex>
void PickPlaceFsm<ArmIndex>::generatePrePoses() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.prePoseHeight.clear();

  for (const auto &objPose : ctx.graspPoses) {
    data::Pose prePose = objPose;
    prePose.pos.z() += 0.1;
    prePose.quat = objPose.quat;
    ctx.prePoseHeight.push_back(prePose);
  }
}

template<int ArmIndex>
bool PickPlaceFsm<ArmIndex>::createTrajectory(const data::Pose &start, const data::Pose &end, double dt) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;

  ctx.currentTrajectory = std::make_unique<EEPoseTrajGen>(dt);
  double velLimit = 4.0 * ctx.limits.velMax;
  double accLimit = 2.0 * ctx.limits.accMax;
  ctx.currentTrajectory->setTcpLimits(velLimit, accLimit);
  ctx.currentTrajectory->setStartState(start.pos, start.quat);
  ctx.currentTrajectory->setTargetState(end.pos, end.quat);
  ctx.currentTrajectory->update();

  double distance = (end.pos - start.pos).norm();
  double duration = ctx.currentTrajectory->getDuration();

  Eigen::Quaterniond deltaQuat = start.quat.inverse() * end.quat;
  Eigen::AngleAxisd deltaAxisAngle(deltaQuat);
  double rotationAngle = std::abs(deltaAxisAngle.angle());

  std::string stateName = "Unknown";
  if (ctx.currentObject < ctx.numObjects) {
    if ((start.pos - ctx.standPose.pos).norm() < 0.1 || (start.pos - ctx.placePose.pos).norm() < 0.1) {
      if ((end.pos - ctx.prePoseHeight[ctx.currentObject].pos).norm() < 0.1) {
        stateName = "MoveToPrePick";
      }
    } else if ((start.pos - ctx.prePoseHeight[ctx.currentObject].pos).norm() < 0.1) {
      if ((end.pos - ctx.graspPoses[ctx.currentObject].pos).norm() < 0.1) {
        stateName = "MoveToPick";
      } else if ((end.pos - ctx.placePose.pos).norm() < 0.1) {
        stateName = "MoveToPlace";
      }
    } else if ((start.pos - ctx.graspPoses[ctx.currentObject].pos).norm() < 0.1) {
      stateName = "Lift";
    }
  } else {
    stateName = "ReturnToStand";
  }

  if (ctx.printDebug) {
    std::cout << "*** " << stateName << " Trajectory (Object " << ctx.currentObject + 1 << "/5) ***" << std::endl;
    std::cout << "  Distance: " << distance << " m" << std::endl;
    std::cout << "  Rotation: " << rotationAngle << " rad (" << rotationAngle * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "  Vel limit: " << velLimit << " m/s, Rot vel limit: " << 10.0 * velLimit << " rad/s" << std::endl;
    std::cout << "  Acc limit: " << accLimit << " m/s²" << std::endl;
    std::cout << "  Duration: " << duration << " s" << std::endl;
    std::cout << "  Expected duration (linear): " << distance / velLimit << " s" << std::endl;
    std::cout << "  Expected duration (rotation): " << rotationAngle / (10.0 * velLimit) << " s" << std::endl;
  }

  return true;
}

template<int ArmIndex>
bool PickPlaceFsm<ArmIndex>::isTrajectoryComplete(const data::Pose &targetPose) const {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;

  if (!ctx.currentTrajectory) {
    return false;
  }

  auto [currPos, currQuat] = ctx.currentTrajectory->getCurrPose();
  double posDistance = (currPos - targetPose.pos).norm();

  Eigen::Quaterniond quatDiff = currQuat.inverse() * targetPose.quat;
  double angleDistance = 2.0 * std::acos(std::abs(quatDiff.w()));

  return (posDistance < POSE_TOLERANCE) && (angleDistance < ANGLE_TOLERANCE);
}

template<int ArmIndex>
data::Pose PickPlaceFsm<ArmIndex>::getTargetPoseForState(PickPlaceStateID stateId) const {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  data::Pose targetPose;
  targetPose.quat = ctx.standPose.quat;

  switch (stateId) {
  case PickPlaceStateID::Idle:
    targetPose = ctx.standPose;
    break;
  case PickPlaceStateID::MoveToPrePick:
    targetPose = ctx.prePoseHeight[ctx.currentObject];
    break;
  case PickPlaceStateID::MoveToPick:
  case PickPlaceStateID::Grasp:
    targetPose = ctx.graspPoses[ctx.currentObject];
    break;
  case PickPlaceStateID::Lift:
    targetPose = ctx.prePoseHeight[ctx.currentObject];
    break;
  case PickPlaceStateID::MoveToPlace:
  case PickPlaceStateID::Drop:
    targetPose = ctx.placePose;
    break;
  case PickPlaceStateID::ReturnToStand:
    targetPose = ctx.standPose;
    break;
  }

  return targetPose;
}

template<int ArmIndex>
data::Pose PickPlaceFsm<ArmIndex>::getCurrentTargetPose() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  data::Pose targetPose;
  targetPose.quat = ctx.standPose.quat;
  PickPlaceStateID currentState;
  if (PickPlaceFsm<ArmIndex>::template is_in_state<PickPlaceIdle<ArmIndex>>()) {
    currentState = PickPlaceStateID::Idle;
  } else if (PickPlaceFsm<ArmIndex>::template is_in_state<MoveToPrePick<ArmIndex>>()) {
    currentState = PickPlaceStateID::MoveToPrePick;
  } else if (PickPlaceFsm<ArmIndex>::template is_in_state<MoveToPick<ArmIndex>>()) {
    currentState = PickPlaceStateID::MoveToPick;
  } else if (PickPlaceFsm<ArmIndex>::template is_in_state<Grasp<ArmIndex>>()) {
    currentState = PickPlaceStateID::Grasp;
  } else if (PickPlaceFsm<ArmIndex>::template is_in_state<Lift<ArmIndex>>()) {
    currentState = PickPlaceStateID::Lift;
  } else if (PickPlaceFsm<ArmIndex>::template is_in_state<MoveToPlace<ArmIndex>>()) {
    currentState = PickPlaceStateID::MoveToPlace;
  } else if (PickPlaceFsm<ArmIndex>::template is_in_state<Drop<ArmIndex>>()) {
    currentState = PickPlaceStateID::Drop;
  } else if (PickPlaceFsm<ArmIndex>::template is_in_state<ReturnToStand<ArmIndex>>()) {
    currentState = PickPlaceStateID::ReturnToStand;
  } else {
    currentState = PickPlaceStateID::Idle;
  }

  switch (currentState) {
  case PickPlaceStateID::Idle:
    targetPose = ctx.standPose;
    break;
  case PickPlaceStateID::MoveToPrePick:
    if (ctx.currentTrajectory) {
      auto [pos, quat] = ctx.currentTrajectory->getCurrPose();
      targetPose.pos = pos;
      targetPose.quat = quat;
    } else {
      targetPose = ctx.prePoseHeight[ctx.currentObject];
    }
    break;
  case PickPlaceStateID::MoveToPick:
  case PickPlaceStateID::Grasp:
    if (ctx.currentTrajectory) {
      auto [pos, quat] = ctx.currentTrajectory->getCurrPose();
      targetPose.pos = pos;
      targetPose.quat = quat;
    } else {
      targetPose = ctx.graspPoses[ctx.currentObject];
    }
    break;
  case PickPlaceStateID::Lift:
    if (ctx.currentTrajectory) {
      auto [pos, quat] = ctx.currentTrajectory->getCurrPose();
      targetPose.pos = pos;
      targetPose.quat = quat;
    } else {
      targetPose = ctx.prePoseHeight[ctx.currentObject];
    }
    break;
  case PickPlaceStateID::MoveToPlace:
  case PickPlaceStateID::Drop:
    if (ctx.currentTrajectory) {
      auto [pos, quat] = ctx.currentTrajectory->getCurrPose();
      targetPose.pos = pos;
      targetPose.quat = quat;
    } else {
      targetPose = ctx.placePose;
    }
    break;
  case PickPlaceStateID::ReturnToStand:
    if (ctx.currentTrajectory) {
      auto [pos, quat] = ctx.currentTrajectory->getCurrPose();
      targetPose.pos = pos;
      targetPose.quat = quat;
    } else {
      targetPose = ctx.standPose;
    }
    break;
  }

  return targetPose;
}

template<int ArmIndex>
bool PickPlaceFsm<ArmIndex>::shouldOpenGripper() {
  return PickPlaceFsm<ArmIndex>::template is_in_state<PickPlaceIdle<ArmIndex>>() || PickPlaceFsm<ArmIndex>::template is_in_state<MoveToPrePick<ArmIndex>>() || PickPlaceFsm<ArmIndex>::template is_in_state<MoveToPick<ArmIndex>>() || PickPlaceFsm<ArmIndex>::template is_in_state<Drop<ArmIndex>>() || PickPlaceFsm<ArmIndex>::template is_in_state<ReturnToStand<ArmIndex>>();
}

template<int ArmIndex>
bool PickPlaceFsm<ArmIndex>::isCompleted() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  return ctx.completed;
}

template<int ArmIndex>
PickPlaceStateID PickPlaceIdle<ArmIndex>::getID() const {
  return PickPlaceStateID::Idle;
}

template<int ArmIndex>
void PickPlaceIdle<ArmIndex>::entry() {
}

template<int ArmIndex>
void PickPlaceIdle<ArmIndex>::react(PickPlaceEventStart const &e) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.standPose = e.standPose;
  ctx.placePose = e.placePose;
  ctx.graspPoses = e.graspPoses;
  ctx.numObjects = static_cast<int>(e.graspPoses.size());
  ctx.currentObject = 0;
  ctx.stateTimer = 0.0f;
  ctx.totalTime = 0.0f;
  ctx.dt = e.dt;
  ctx.initialized = true;

  this->generatePrePoses();

  this->template transit<MoveToPrePick<ArmIndex>>();
}

template<int ArmIndex>
void PickPlaceIdle<ArmIndex>::react(PickPlaceEventReset const &) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.initialized = false;
  ctx.completed = false;
  ctx.currentObject = 0;
  ctx.stateTimer = 0.0f;
  ctx.totalTime = 0.0f;
  ctx.currentTrajectory.reset();
}

template<int ArmIndex>
PickPlaceStateID MoveToPrePick<ArmIndex>::getID() const {
  return PickPlaceStateID::MoveToPrePick;
}

template<int ArmIndex>
void MoveToPrePick<ArmIndex>::entry() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer = 0.0f;
  ctx.trajAccumTime = 0.0;

  data::Pose startPose;
  if (ctx.currentObject == 0) {
    startPose = ctx.standPose;
  } else {
    startPose = ctx.placePose;
  }

  this->createTrajectory(startPose, ctx.prePoseHeight[ctx.currentObject], ctx.dt);
}

template<int ArmIndex>
void MoveToPrePick<ArmIndex>::react(PickPlaceEventTick const &e) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer += e.duration;
  ctx.totalTime += e.duration;

  if (ctx.currentTrajectory) {
    ctx.trajAccumTime += e.duration;

    while (ctx.trajAccumTime >= ctx.dt) {
      ctx.currentTrajectory->update();
      ctx.currentTrajectory->passToInput();
      ctx.trajAccumTime -= ctx.dt;
    }

    data::Pose targetPose = this->getTargetPoseForState(PickPlaceStateID::MoveToPrePick);
    if (this->isTrajectoryComplete(targetPose)) {
      if (ctx.printDebug) {
        std::cout << "  ✓ Reached pre-pick position, transitioning to MoveToPick" << std::endl;
      }
      this->template transit<MoveToPick<ArmIndex>>();
    }
  }
}

template<int ArmIndex>
PickPlaceStateID MoveToPick<ArmIndex>::getID() const {
  return PickPlaceStateID::MoveToPick;
}

template<int ArmIndex>
void MoveToPick<ArmIndex>::entry() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer = 0.0f;
  ctx.trajAccumTime = 0.0;

  this->createTrajectory(ctx.prePoseHeight[ctx.currentObject], ctx.graspPoses[ctx.currentObject], ctx.dt);
}

template<int ArmIndex>
void MoveToPick<ArmIndex>::react(PickPlaceEventTick const &e) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer += e.duration;
  ctx.totalTime += e.duration;

  if (ctx.currentTrajectory) {
    ctx.trajAccumTime += e.duration;

    while (ctx.trajAccumTime >= ctx.dt) {
      ctx.currentTrajectory->update();
      ctx.currentTrajectory->passToInput();
      ctx.trajAccumTime -= ctx.dt;
    }

    data::Pose targetPose = this->getTargetPoseForState(PickPlaceStateID::MoveToPick);
    if (this->isTrajectoryComplete(targetPose)) {
      if (ctx.printDebug) {
        std::cout << "  ✓ Reached pick position, transitioning to Grasp" << std::endl;
      }
      this->template transit<Grasp<ArmIndex>>();
    }
  }
}

template<int ArmIndex>
PickPlaceStateID Grasp<ArmIndex>::getID() const {
  return PickPlaceStateID::Grasp;
}

template<int ArmIndex>
void Grasp<ArmIndex>::entry() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer = 0.0f;
}

template<int ArmIndex>
void Grasp<ArmIndex>::react(PickPlaceEventTick const &e) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer += e.duration;
  ctx.totalTime += e.duration;

  if (ctx.stateTimer >= ctx.limits.graspTime) {
    this->template transit<Lift<ArmIndex>>();
  }
}

template<int ArmIndex>
PickPlaceStateID Lift<ArmIndex>::getID() const {
  return PickPlaceStateID::Lift;
}

template<int ArmIndex>
void Lift<ArmIndex>::entry() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer = 0.0f;
  ctx.trajAccumTime = 0.0;

  this->createTrajectory(ctx.graspPoses[ctx.currentObject], ctx.prePoseHeight[ctx.currentObject], ctx.dt);
}

template<int ArmIndex>
void Lift<ArmIndex>::react(PickPlaceEventTick const &e) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer += e.duration;
  ctx.totalTime += e.duration;

  if (ctx.currentTrajectory) {
    ctx.trajAccumTime += e.duration;

    while (ctx.trajAccumTime >= ctx.dt) {
      ctx.currentTrajectory->update();
      ctx.currentTrajectory->passToInput();
      ctx.trajAccumTime -= ctx.dt;
    }

    data::Pose targetPose = this->getTargetPoseForState(PickPlaceStateID::Lift);
    if (this->isTrajectoryComplete(targetPose)) {
      if (ctx.printDebug) {
        std::cout << "  ✓ Lifted object to safe height, transitioning to MoveToPlace" << std::endl;
      }
      this->template transit<MoveToPlace<ArmIndex>>();
    }
  }
}

template<int ArmIndex>
PickPlaceStateID MoveToPlace<ArmIndex>::getID() const {
  return PickPlaceStateID::MoveToPlace;
}

template<int ArmIndex>
void MoveToPlace<ArmIndex>::entry() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer = 0.0f;
  ctx.trajAccumTime = 0.0;

  this->createTrajectory(ctx.prePoseHeight[ctx.currentObject], ctx.placePose, ctx.dt);
}

template<int ArmIndex>
void MoveToPlace<ArmIndex>::react(PickPlaceEventTick const &e) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer += e.duration;
  ctx.totalTime += e.duration;

  if (ctx.currentTrajectory) {
    ctx.trajAccumTime += e.duration;

    while (ctx.trajAccumTime >= ctx.dt) {
      ctx.currentTrajectory->update();
      ctx.currentTrajectory->passToInput();
      ctx.trajAccumTime -= ctx.dt;
    }

    data::Pose targetPose = this->getTargetPoseForState(PickPlaceStateID::MoveToPlace);
    if (this->isTrajectoryComplete(targetPose)) {
      if (ctx.printDebug) {
        std::cout << "  ✓ Reached place position, transitioning to Drop" << std::endl;
      }
      this->template transit<Drop<ArmIndex>>();
    }
  }
}

template<int ArmIndex>
PickPlaceStateID Drop<ArmIndex>::getID() const {
  return PickPlaceStateID::Drop;
}

template<int ArmIndex>
void Drop<ArmIndex>::entry() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer = 0.0f;
}

template<int ArmIndex>
void Drop<ArmIndex>::react(PickPlaceEventTick const &e) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer += e.duration;
  ctx.totalTime += e.duration;

  if (ctx.stateTimer >= ctx.limits.graspTime) {
    ctx.currentObject++;

    if (ctx.currentObject < ctx.numObjects) {
      this->template transit<MoveToPrePick<ArmIndex>>();
    } else {
      this->template transit<ReturnToStand<ArmIndex>>();
    }
  }
}

template<int ArmIndex>
PickPlaceStateID ReturnToStand<ArmIndex>::getID() const {
  return PickPlaceStateID::ReturnToStand;
}

template<int ArmIndex>
void ReturnToStand<ArmIndex>::entry() {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer = 0.0f;
  ctx.trajAccumTime = 0.0;

  this->createTrajectory(ctx.placePose, ctx.standPose, ctx.dt);
}

template<int ArmIndex>
void ReturnToStand<ArmIndex>::react(PickPlaceEventTick const &e) {
  auto &ctx = PickPlaceFsm<ArmIndex>::ctx_;
  ctx.stateTimer += e.duration;
  ctx.totalTime += e.duration;

  if (ctx.currentTrajectory) {
    ctx.trajAccumTime += e.duration;

    while (ctx.trajAccumTime >= ctx.dt) {
      ctx.currentTrajectory->update();
      ctx.currentTrajectory->passToInput();
      ctx.trajAccumTime -= ctx.dt;
    }

    data::Pose targetPose = this->getTargetPoseForState(PickPlaceStateID::ReturnToStand);
    if (this->isTrajectoryComplete(targetPose)) {
      if (ctx.printDebug) {
        std::cout << "  ✓ Returned to stand position, pick and place sequence completed!" << std::endl;
      }
      ctx.completed = true;
      this->template transit<PickPlaceIdle<ArmIndex>>();
    }
  }
}

template struct PickPlaceFsm<0>;
template struct PickPlaceFsm<1>;

template struct PickPlaceIdle<0>;
template struct PickPlaceIdle<1>;

template struct MoveToPrePick<0>;
template struct MoveToPrePick<1>;

template struct MoveToPick<0>;
template struct MoveToPick<1>;

template struct Grasp<0>;
template struct Grasp<1>;

template struct Lift<0>;
template struct Lift<1>;

template struct MoveToPlace<0>;
template struct MoveToPlace<1>;

template struct Drop<0>;
template struct Drop<1>;

template struct ReturnToStand<0>;
template struct ReturnToStand<1>;

} // namespace rynn
