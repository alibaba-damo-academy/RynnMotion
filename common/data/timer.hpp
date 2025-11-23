#pragma once

#include "data_base.hpp"

namespace data {
struct TimerState : public DataMsg {
  double simTime;  // Simulation time
  double wallTime; // Wall clock time
  double duration; // Duration since last call

  TimerState(double simTime = 0.0, double wallTime = 0.0, double duration = 0.0) :
      simTime(simTime), wallTime(wallTime), duration(duration) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<TimerState>(*this);
  }

  TimerState &operator=(const TimerState &other) {
    if (this != &other) {
      simTime = other.simTime;
      wallTime = other.wallTime;
      duration = other.duration;
    }
    return *this;
  }

  TimerState(const TimerState &other) :
      DataMsg(other), simTime(other.simTime),
      wallTime(other.wallTime), duration(other.duration) {
  }
};
} // namespace data
