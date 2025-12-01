#pragma once

#include <iostream>
#include <memory>

#include "fsm.hpp"
#include "motion_fsm.hpp"

namespace rynn {

/**
 * @brief Centralized FSM manager - single source of truth for state machine
 *
 * Responsibilities:
 * - Owns FSM data (FsmData) that modules read
 * - Controls FSM state machine (MotionFsm via TinyFSM)
 * - Guarantees synchronization between FSM logic and data
 * - Provides unified reset/event API
 * - Prevents state divergence bugs
 *
 * Design:
 * - FsmManager is the ONLY way to interact with the FSM
 * - Modules get read-only access to FSM state via getData()
 * - All state transitions go through this manager
 * - Reset logic centralized in one place
 */
class FsmManager {
public:
  FsmManager();
  ~FsmManager() = default;

  /**
   * @brief Reset FSM to initial state
   *
   * Resets both the TinyFSM state machine and FsmData.
   * Guaranteed to be synchronized - no manual sync needed.
   */
  void reset();

  /**
   * @brief Update FSM with time tick
   *
   * @param dt Time delta in seconds
   *
   * Automatically synchronizes FsmData with current FSM state.
   * Handles state transitions and timing updates.
   */
  void tick(float dt);

  /**
   * @brief Update FSM using absolute simulation time
   *
   * @param simTime Current simulation time in seconds
   *
   * Uses absolute time instead of dt accumulation. More robust when
   * controller runs at different frequencies than physics.
   */
  void tickWithSimTime(double simTime);

  /**
   * @brief Send event to FSM and update data
   *
   * All events go through this method to ensure synchronization.
   * Future: Add more event types as needed (EventRequestStand, etc.)
   */
  void sendEvent(const EventTick &e);

  /**
   * @brief Get current FSM state
   */
  StateID getCurrentState() const {
    return data_.nowState;
  }

  /**
   * @brief Get previous FSM state
   */
  StateID getPreviousState() const {
    return data_.prevState;
  }

  /**
   * @brief Get time spent in current state (seconds)
   */
  float getStateTime() const {
    return data_.state_t;
  }

  /**
   * @brief Get total elapsed time (seconds)
   */
  float getTotalTime() const {
    return data_.total_t;
  }

  /**
   * @brief Check if this is first tick of new state
   */
  bool isOnEntry() const {
    return data_.onEntry;
  }

  /**
   * @brief Get read-only access to FSM data
   *
   * Modules use this to read FSM state without direct coupling to TinyFSM.
   *
   * @example
   * const auto& fsm = fsmManager->getData();
   * if (fsm.nowState == StateID::GoStand1 && fsm.onEntry) {
   *   // Initialize stand controller
   * }
   */
  const data::FsmData &getData() const {
    return data_;
  }

  /**
   * @brief Print current FSM state for debugging
   */
  void printState() const;

private:
  // Single source of truth for FSM state data
  data::FsmData data_;

  // Track previous state for transition detection
  StateID lastKnownState_;

  // For absolute time tracking
  double lastSimTime_{0.0};
  double stateEntryTime_{0.0};

  /**
   * @brief Query current state from TinyFSM state machine
   *
   * This extracts the current state from MotionFsm.
   * Since TinyFSM uses static dispatch, we check state timers
   * to infer which state is active.
   */
  StateID getCurrentStateFromFSM() const;

  /**
   * @brief Synchronize FsmData with current FSM state
   *
   * Called after every FSM update to ensure data matches reality.
   * Detects state transitions and updates timing accordingly.
   */
  void syncDataFromStateMachine(float dt);
};

} // namespace rynn
