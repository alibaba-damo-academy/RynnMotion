#pragma once

#include <Eigen/Dense>
#include <memory>

#include "data_base.hpp"

namespace data {
struct MotorCommand : public DataMsg {
  int branchIndex;
  double qCmd;
  double qdCmd;
  double qtauCmd;
  double kp;
  double kd;

  MotorCommand() :
      branchIndex(0), qCmd(0.0), qdCmd(0.0), qtauCmd(0.0), kp(0.0), kd(0.0) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<MotorCommand>(*this);
  }

  MotorCommand &operator=(const MotorCommand &other) {
    if (this != &other) {
      branchIndex = other.branchIndex;
      qCmd = other.qCmd;
      qdCmd = other.qdCmd;
      qtauCmd = other.qtauCmd;
      kp = other.kp;
      kd = other.kd;
    }
    return *this;
  }

  MotorCommand(const MotorCommand &other) :
      DataMsg(other), branchIndex(other.branchIndex), qCmd(other.qCmd), qdCmd(other.qdCmd),
      qtauCmd(other.qtauCmd), kp(other.kp), kd(other.kd) {
  }
};

struct MotorFeedback : public DataMsg {
  int branchIndex;
  double qFb;
  double qdFb;
  double qtauFb;

  MotorFeedback() :
      branchIndex(0), qFb(0.0), qdFb(0.0), qtauFb(0.0) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<MotorFeedback>(*this);
  }

  MotorFeedback &operator=(const MotorFeedback &other) {
    if (this != &other) {
      branchIndex = other.branchIndex;
      qFb = other.qFb;
      qdFb = other.qdFb;
      qtauFb = other.qtauFb;
    }
    return *this;
  }

  MotorFeedback(const MotorFeedback &other) :
      DataMsg(other), branchIndex(other.branchIndex), qFb(other.qFb), qdFb(other.qdFb), qtauFb(other.qtauFb) {
  }
};
} // namespace data