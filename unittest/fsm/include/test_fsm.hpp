#pragma once
#include <iostream>

#include "tinyfsm.hpp"

/* ---------- 1. Events ------------------------------------------------ */
struct EventTick : tinyfsm::Event {
  explicit EventTick(float dt_sec) :
      duration(dt_sec) {
  }
  float duration; // Δt in seconds
};

struct EventCallback : tinyfsm::Event // reserved for future use
{
  // Add ROS / LCM message members when needed
};

/* ---------- 2. Shared context ---------------------------------------- */
// Note: Using RuntimeData timing instead of global context
// struct Context {
//   float timer{0.f}; // time accumulated in current state
// };
// extern Context g_ctx;

/* ---------- 3. Base FSM class ---------------------------------------- */
enum class StateID { Init,
                     GoStand1,
                     Action1,
                     GoHome };

struct MotionFsm : tinyfsm::Fsm<MotionFsm> {
  virtual StateID getID() const = 0;

  virtual void entry() {
  }
  virtual void exit() {
  }

  void react(tinyfsm::Event const &) {
  } // default react

  virtual void react(EventTick const &) {
  } // react for EventTick

  void react(EventCallback const &) {
  } // react for Event callback
};

/* ---------- 4. Forward declarations of states ----------------------- */
struct Init;
struct GoStand1;
struct Action1;
struct GoHome;

/* ---------- 5. State definitions ------------------------------------ */
struct Init : MotionFsm {
  float state_timer{0.0f}; // Local state timer

  StateID getID() const override {
    return StateID::Init;
  }

  void entry() override {
    MotionFsm::entry();
    state_timer = 0.0f;
    std::cout << "▶ Init \n";
  }

  void react(EventTick const &e) override {
    state_timer += e.duration;
    // Transition immediately to GoStand1
    transit<GoStand1>();
  }
};

struct GoStand1 : MotionFsm {
  float state_timer{0.0f};

  StateID getID() const override {
    return StateID::GoStand1;
  }

  void entry() override {
    MotionFsm::entry();
    state_timer = 0.0f;
    std::cout << "▶ GoStand1 (2 s)\n";
  }

  void react(EventTick const &e) override {
    state_timer += e.duration;
    if (state_timer >= 2.0f)
      transit<Action1>();
  }
};

struct Action1 : MotionFsm {
  float state_timer{0.0f};

  StateID getID() const override {
    return StateID::Action1;
  }

  void entry() override {
    MotionFsm::entry();
    state_timer = 0.0f;
    std::cout << "▶ Action1 (10 s)\n";
  }

  void react(EventTick const &e) override {
    state_timer += e.duration;
    if (state_timer >= 10.0f)
      transit<GoHome>();
  }
};

struct GoHome : MotionFsm {
  float state_timer{0.0f};

  StateID getID() const override {
    return StateID::GoHome;
  }

  void entry() override {
    MotionFsm::entry();
    state_timer = 0.0f;
    std::cout << "▶ GoHome (2 s)\n";
  }

  void react(EventTick const &e) override {
    state_timer += e.duration;
    if (state_timer >= 2.0f)
      transit<Init>();
  }
};
