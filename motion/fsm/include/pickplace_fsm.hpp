#pragma once

#include <memory>

#include "pose.hpp"
#include "trajgen.hpp"

namespace rynn {

struct PickPlaceEventTick {
  explicit PickPlaceEventTick(float dt_sec) : duration(dt_sec) {}
  float duration;
};

struct PickPlaceEventStart {
  explicit PickPlaceEventStart(const data::Pose &standPose, const data::Pose &placePose,
                               const std::vector<data::Pose> &graspPoses, double dt = 0.001)
      : standPose(standPose), placePose(placePose), graspPoses(graspPoses), dt(dt) {}

  data::Pose standPose;
  data::Pose placePose;
  std::vector<data::Pose> graspPoses;
  double dt;
};

struct PickPlaceEventReset {};

enum class PickPlaceStateID { Idle, MoveToPrePick, MoveToPick, Grasp, Lift, MoveToPlace, Drop, ReturnToStand };

/**
 * @class PickPlaceFsm
 * @brief Instance-based pick-and-place finite state machine
 *
 * Responsibilities:
 * - Manage state transitions for pick-place sequences
 * - Generate smooth trajectories between waypoints
 * - Track gripper open/close commands per state
 * - Support multiple independent instances for dual-arm robots
 */
class PickPlaceFsm {
public:
  struct TrajectoryLimits {
    double velMax = 1.0;
    double accMax = 5.0;
    double graspTime = 0.3;
  };

  struct Context {
    data::Pose standPose;
    data::Pose placePose;
    std::vector<data::Pose> graspPoses;
    std::vector<data::Pose> prePoseHeight;

    int currentObject = 0;
    int numObjects = 0;
    float stateTimer = 0.0f;
    float totalTime = 0.0f;
    double dt = 0.001;
    double trajAccumTime = 0.0;

    std::unique_ptr<EEPoseTrajGen> currentTrajectory;
    TrajectoryLimits limits;

    bool initialized = false;
    bool completed = false;
    bool printDebug = false;
  };

  explicit PickPlaceFsm(int armIndex = 0);
  ~PickPlaceFsm() = default;

  PickPlaceFsm(const PickPlaceFsm &) = delete;
  PickPlaceFsm &operator=(const PickPlaceFsm &) = delete;
  PickPlaceFsm(PickPlaceFsm &&) = default;
  PickPlaceFsm &operator=(PickPlaceFsm &&) = default;

  void start();
  void dispatch(const PickPlaceEventTick &e);
  void dispatch(const PickPlaceEventStart &e);
  void dispatch(const PickPlaceEventReset &e);
  void reset();

  PickPlaceStateID getCurrentState() const { return currentState_; }
  data::Pose getCurrentTargetPose() const;
  bool shouldOpenGripper() const;
  bool isCompleted() const { return ctx_.completed; }

  void setLimits(double velMax, double accMax, double graspTime);
  void setDebugPrint(bool enable) { ctx_.printDebug = enable; }
  bool isDebugPrintEnabled() const { return ctx_.printDebug; }

  int getArmIndex() const { return armIndex_; }
  const Context &getContext() const { return ctx_; }

private:
  int armIndex_;
  PickPlaceStateID currentState_ = PickPlaceStateID::Idle;
  Context ctx_;

  void transitTo(PickPlaceStateID newState);
  void onEntry(PickPlaceStateID state);
  void onTick(float dt);

  void generatePrePoses();
  bool createTrajectory(const data::Pose &start, const data::Pose &end);
  void updateTrajectory(float dt);
  bool isTrajectoryComplete(const data::Pose &target) const;
  data::Pose getTargetPoseForState(PickPlaceStateID state) const;
  PickPlaceStateID getNextState(PickPlaceStateID current) const;
};

} // namespace rynn
