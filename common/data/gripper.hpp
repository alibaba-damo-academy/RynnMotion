#pragma once

#include <vector>

#include "data_base.hpp"

namespace data {

struct GripperCommand : public DataMsg {
  double posCmd;    // Normalized position command [0,1]: 0=closed, 1=open
  double qtauCmd;   // Normalized force/torque command [0,1]
  bool toggleGripper; // Toggle signal from UI (true = toggle requested)

  GripperCommand() :
      posCmd(0.0), qtauCmd(0.0), toggleGripper(false) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<GripperCommand>(*this);
  }

  GripperCommand &operator=(const GripperCommand &other) {
    if (this != &other) {
      posCmd = other.posCmd;
      qtauCmd = other.qtauCmd;
      toggleGripper = other.toggleGripper;
    }
    return *this;
  }

  GripperCommand(const GripperCommand &other) :
      DataMsg(other),
      posCmd(other.posCmd), qtauCmd(other.qtauCmd), toggleGripper(other.toggleGripper) {
  }
};

struct GripperFeedback : public DataMsg {
  double posFb;     // Normalized position feedback [0,1]: 0=closed, 1=open
  double tauFb;     // Normalized force/torque feedback [0,1]

  GripperFeedback() :
      posFb(0.0), tauFb(0.0) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<GripperFeedback>(*this);
  }

  GripperFeedback &operator=(const GripperFeedback &other) {
    if (this != &other) {
      posFb = other.posFb;
      tauFb = other.tauFb;
    }
    return *this;
  }

  GripperFeedback(const GripperFeedback &other) :
      DataMsg(other),
      posFb(other.posFb), tauFb(other.tauFb) {
  }
};

} // namespace data