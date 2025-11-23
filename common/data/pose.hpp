#pragma once
#include <Eigen/Dense>

#include "data_base.hpp"

namespace data {
struct Pose : public DataMsg {
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;

  Pose() :
      pos(0.0, 0.0, 0.0), quat(1.0, 0.0, 0.0, 0.0) {
  }

  Pose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) :
      pos(position), quat(orientation) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<Pose>(*this);
  }

  Pose &operator=(const Pose &other) {
    if (this != &other) {
      pos = other.pos;
      quat = other.quat;
    }
    return *this;
  }

  Pose(const Pose &other) :
      DataMsg(other), pos(other.pos), quat(other.quat) {
  }
};
} // namespace data
