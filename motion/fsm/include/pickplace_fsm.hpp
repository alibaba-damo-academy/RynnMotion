#pragma once

#include <memory>

#include "pose.hpp"
#include "tinyfsm.hpp"
#include "trajgen.hpp"

namespace rynn {
class PickPlaceController;
}

namespace rynn {

template<int ArmIndex> struct PickPlaceIdle;
template<int ArmIndex> struct MoveToPrePick;
template<int ArmIndex> struct MoveToPick;
template<int ArmIndex> struct Grasp;
template<int ArmIndex> struct Lift;
template<int ArmIndex> struct MoveToPlace;
template<int ArmIndex> struct Drop;
template<int ArmIndex> struct ReturnToStand;

/**
 * @brief Tick event for pick-place FSM
 */
struct PickPlaceEventTick : tinyfsm::Event {
  explicit PickPlaceEventTick(float dt_sec) : duration(dt_sec) {
  }
  float duration;
};

/**
 * @brief Start pick-place sequence
 */
struct PickPlaceEventStart : tinyfsm::Event {
  explicit PickPlaceEventStart(const data::Pose &standPose, const data::Pose &placePose,
                               const std::vector<data::Pose> &graspPoses, double dt = 0.001) : standPose(standPose), placePose(placePose), graspPoses(graspPoses), dt(dt) {
  }

  data::Pose standPose;
  data::Pose placePose;
  std::vector<data::Pose> graspPoses;
  double dt;
};

/**
 * @brief Reset pick-place FSM
 */
struct PickPlaceEventReset : tinyfsm::Event {};

enum class PickPlaceStateID {
  Idle,
  MoveToPrePick,
  MoveToPick,
  Grasp,
  Lift,
  MoveToPlace,
  Drop,
  ReturnToStand
};

/**
 * @brief Pick-and-place finite state machine (templated for dual-arm support)
 * @tparam ArmIndex Arm identifier (0=left/single-arm, 1=right)
 *
 * Template creates independent FSM instances:
 * - PickPlaceFsm<0> for left arm (and single-arm robots)
 * - PickPlaceFsm<1> for right arm in dual-arm setups
 */
template<int ArmIndex = 0>
struct PickPlaceFsm : tinyfsm::Fsm<PickPlaceFsm<ArmIndex>> {
  friend class rynn::PickPlaceController;

  virtual PickPlaceStateID getID() const = 0;

  virtual void entry() {
  }
  virtual void exit() {
  }

  void react(tinyfsm::Event const &) {
  }
  virtual void react(PickPlaceEventTick const &) {
  }
  virtual void react(PickPlaceEventStart const &) {
  }
  virtual void react(PickPlaceEventReset const &) {
  }

  static void reset();

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

protected:
  // Each template instantiation gets its own static context
  static Context ctx_;

  void generatePrePoses();
  bool createTrajectory(const data::Pose &start, const data::Pose &end, double dt);
  bool isTrajectoryComplete(const data::Pose &targetPose) const;
  data::Pose getTargetPoseForState(PickPlaceStateID stateId) const;

public:
  static data::Pose getCurrentTargetPose();
  static bool shouldOpenGripper();
  static bool isCompleted();
  static void setDebugPrint(bool enable) {
    ctx_.printDebug = enable;
  }
  static bool isDebugPrintEnabled() {
    return ctx_.printDebug;
  }
};

// Type aliases for convenience
using PickPlaceFsmLeft = PickPlaceFsm<0>;
using PickPlaceFsmRight = PickPlaceFsm<1>;

template<int ArmIndex = 0>
struct PickPlaceIdle : PickPlaceFsm<ArmIndex> {
  PickPlaceStateID getID() const override;
  void entry() override;
  void react(PickPlaceEventStart const &e) override;
  void react(PickPlaceEventReset const &) override;
};

template<int ArmIndex = 0>
struct MoveToPrePick : PickPlaceFsm<ArmIndex> {
  PickPlaceStateID getID() const override;
  void entry() override;
  void react(PickPlaceEventTick const &e) override;
};

template<int ArmIndex = 0>
struct MoveToPick : PickPlaceFsm<ArmIndex> {
  PickPlaceStateID getID() const override;
  void entry() override;
  void react(PickPlaceEventTick const &e) override;
};

template<int ArmIndex = 0>
struct Grasp : PickPlaceFsm<ArmIndex> {
  PickPlaceStateID getID() const override;
  void entry() override;
  void react(PickPlaceEventTick const &e) override;
};

template<int ArmIndex = 0>
struct Lift : PickPlaceFsm<ArmIndex> {
  PickPlaceStateID getID() const override;
  void entry() override;
  void react(PickPlaceEventTick const &e) override;
};

template<int ArmIndex = 0>
struct MoveToPlace : PickPlaceFsm<ArmIndex> {
  PickPlaceStateID getID() const override;
  void entry() override;
  void react(PickPlaceEventTick const &e) override;
};

template<int ArmIndex = 0>
struct Drop : PickPlaceFsm<ArmIndex> {
  PickPlaceStateID getID() const override;
  void entry() override;
  void react(PickPlaceEventTick const &e) override;
};

template<int ArmIndex = 0>
struct ReturnToStand : PickPlaceFsm<ArmIndex> {
  PickPlaceStateID getID() const override;
  void entry() override;
  void react(PickPlaceEventTick const &e) override;
};

} // namespace rynn
