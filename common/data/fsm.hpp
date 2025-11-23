#pragma once

#include "data_base.hpp"
#include "motion_fsm.hpp"

namespace data {

/**
 * @brief Data structure to hold FSM state information
 *
 * This structure provides access to the current FSM state and timing
 * information for all modules in the system. Data-only, no logic.
 */
struct FsmData : public DataMsg {
  StateID nowState{StateID::Init};
  StateID prevState{StateID::Init};
  float state_t{0.0f}; // Time spent in current state
  float total_t{0.0f}; // Total elapsed time

  // State transition flags
  bool onEntry{false}; // True on first tick of new state

  FsmData() = default;

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<FsmData>(*this);
  }
};

} // namespace data