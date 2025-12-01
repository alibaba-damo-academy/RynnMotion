#pragma once

#include <memory>

#include "tinyfsm.hpp"

struct Init;
struct GoStand1;
struct Action1;
struct GoHome;

/**
 * @brief Tick event with duration
 */
struct EventTick : tinyfsm::Event {
  explicit EventTick(float dt_sec, void *unused = nullptr) : duration(dt_sec) {
    (void)unused;
  }
  float duration;
};

/**
 * @brief Callback event for external triggers
 */
struct EventCallback : tinyfsm::Event {};

enum class StateID { Init,
                     GoStand1,
                     Action1,
                     GoHome };

/**
 * @brief Base FSM class for motion state machine
 */
struct MotionFsm : tinyfsm::Fsm<MotionFsm> {
  virtual StateID getID() const = 0;

  virtual void entry() {
  }
  virtual void exit() {
  }

  void react(tinyfsm::Event const &) {
  }
  virtual void react(EventTick const &) {
  }
  void react(EventCallback const &) {
  }

  static void reset();
  static StateID currentStateID;

protected:
};

/**
 * @brief Initial state
 */
struct Init : MotionFsm {
  float state_timer{0.0f};

  StateID getID() const override {
    return StateID::Init;
  }

  void entry() override {
    MotionFsm::entry();
    state_timer = 0.0f;
    MotionFsm::currentStateID = StateID::Init;
  }

  void react(EventTick const &e) override {
    state_timer += e.duration;
    transit<GoStand1>();
  }
};

/**
 * @brief Go to standby position 1
 */
struct GoStand1 : MotionFsm {
  float state_timer{0.0f};

  StateID getID() const override {
    return StateID::GoStand1;
  }

  void entry() override {
    MotionFsm::entry();
    state_timer = 0.0f;
    MotionFsm::currentStateID = StateID::GoStand1;
  }

  void react(EventTick const &e) override {
    state_timer += e.duration;
    if (state_timer >= 1.0f) transit<Action1>();
  }
};

/**
 * @brief Main action state
 */
struct Action1 : MotionFsm {
  float state_timer{0.0f};

  StateID getID() const override {
    return StateID::Action1;
  }

  void entry() override {
    MotionFsm::entry();
    state_timer = 0.0f;
    MotionFsm::currentStateID = StateID::Action1;
  }

  void react(EventTick const &e) override {
    state_timer += e.duration;
    if (state_timer >= 60.0f) transit<GoHome>();
  }
};

/**
 * @brief Return to home position
 */
struct GoHome : MotionFsm {
  float state_timer{0.0f};

  StateID getID() const override {
    return StateID::GoHome;
  }

  void entry() override {
    MotionFsm::entry();
    state_timer = 0.0f;
    MotionFsm::currentStateID = StateID::GoHome;
  }

  void react(EventTick const &e) override {
    state_timer += e.duration;
    if (state_timer >= 2.0f) transit<Init>();
  }
};
