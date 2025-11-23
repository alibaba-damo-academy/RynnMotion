#pragma once

#include <Eigen/Dense>

#include "data_base.hpp"

namespace data {
struct ComFeedback : public DataMsg {
  Eigen::Vector3d pos;
  Eigen::Vector3d linVel;
  Eigen::Vector3d linAcc;

  ComFeedback() :
      pos(0.0, 0.0, 0.0), linVel(0.0, 0.0, 0.0), linAcc(0.0, 0.0, 0.0) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<ComFeedback>(*this);
  }

  ComFeedback &operator=(const ComFeedback &other) {
    if (this != &other) {
      pos = other.pos;
      linVel = other.linVel;
      linAcc = other.linAcc;
    }
    return *this;
  }

  ComFeedback(const ComFeedback &other) :
      DataMsg(other), pos(other.pos), linVel(other.linVel), linAcc(other.linAcc) {
  }
};
} // namespace data