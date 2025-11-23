#include "fsm_manager.hpp"

#include <iomanip>

namespace rynn {

FsmManager::FsmManager() {
  MotionFsm::start();

  data_.nowState = StateID::Init;
  data_.prevState = StateID::Init;
  data_.state_t = 0.0f;
  data_.total_t = 0.0f;
  data_.onEntry = true;

  lastKnownState_ = StateID::Init;
}

void FsmManager::reset() {
  MotionFsm::reset();
  MotionFsm::start();

  data_.nowState = StateID::Init;
  data_.prevState = StateID::Init;
  data_.state_t = 0.0f;
  data_.total_t = 0.0f;
  data_.onEntry = true;

  lastKnownState_ = StateID::Init;
}

void FsmManager::tick(float dt) {
  EventTick e(dt, nullptr);
  MotionFsm::dispatch(e);
  syncDataFromStateMachine(dt);
}

void FsmManager::tickWithSimTime(double simTime) {
  float dt = static_cast<float>(simTime - lastSimTime_);
  lastSimTime_ = simTime;

  EventTick e(dt, nullptr);
  MotionFsm::dispatch(e);

  StateID detectedState = MotionFsm::currentStateID;

  if (detectedState != data_.nowState) {
    data_.prevState = data_.nowState;
    data_.nowState = detectedState;
    data_.onEntry = true;
    stateEntryTime_ = simTime;
    data_.state_t = 0.0f;
  } else {
    data_.onEntry = false;
    data_.state_t = static_cast<float>(simTime - stateEntryTime_);
  }

  data_.total_t = static_cast<float>(simTime);
  lastKnownState_ = detectedState;
}

void FsmManager::sendEvent(const EventTick &e) {
  MotionFsm::dispatch(e);
  syncDataFromStateMachine(e.duration);
}

StateID FsmManager::getCurrentStateFromFSM() const {
  return lastKnownState_;
}

void FsmManager::syncDataFromStateMachine(float dt) {
  StateID detectedState = MotionFsm::currentStateID;

  if (detectedState != data_.nowState) {
    data_.prevState = data_.nowState;
    data_.nowState = detectedState;
    data_.onEntry = true;
    data_.state_t = 0.0f;
  } else {
    data_.onEntry = false;
    data_.state_t += dt;
  }

  data_.total_t += dt;
  lastKnownState_ = detectedState;
}

void FsmManager::printState() const {
  std::cout << "=== FSM State ===" << std::endl;
  std::cout << "Current State: " << static_cast<int>(data_.nowState);

  switch (data_.nowState) {
  case StateID::Init:
    std::cout << " (Init)";
    break;
  case StateID::GoStand1:
    std::cout << " (GoStand1)";
    break;
  case StateID::Action1:
    std::cout << " (Action1)";
    break;
  case StateID::GoHome:
    std::cout << " (GoHome)";
    break;
  }
  std::cout << std::endl;

  std::cout << "Previous State: " << static_cast<int>(data_.prevState) << std::endl;
  std::cout << "State Time: " << std::fixed << std::setprecision(3)
            << data_.state_t << " s" << std::endl;
  std::cout << "Total Time: " << std::fixed << std::setprecision(3)
            << data_.total_t << " s" << std::endl;
  std::cout << "On Entry: " << (data_.onEntry ? "true" : "false") << std::endl;
  std::cout << "=================" << std::endl;
}

} // namespace rynn
