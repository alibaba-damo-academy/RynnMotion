#include "motion_fsm.hpp"

FSM_INITIAL_STATE(MotionFsm, Init)

StateID MotionFsm::currentStateID = StateID::Init;

void MotionFsm::reset() {
  tinyfsm::StateList<Init, GoStand1, Action1, GoHome>::reset();
  state<Init>().state_timer = 0.0f;
  state<GoStand1>().state_timer = 0.0f;
  state<Action1>().state_timer = 0.0f;
  state<GoHome>().state_timer = 0.0f;
  currentStateID = StateID::Init;
}
