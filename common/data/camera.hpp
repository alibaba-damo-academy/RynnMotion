#pragma once
#include <Eigen/Dense>

#include "data_base.hpp"
#include "pose.hpp"

namespace data {
struct Camera : public Pose {
  std::string name;

  Camera(const std::string &name = "") :
      Pose(), name(name) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<Camera>(*this);
  }

  Camera &operator=(const Camera &other) {
    if (this != &other) {
      Pose::operator=(other);
      name = other.name;
    }
    return *this;
  }

  Camera(const Camera &other) :
      Pose(other), name(other.name) {
  }
};
} // namespace data
