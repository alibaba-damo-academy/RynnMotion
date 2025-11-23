#pragma once

#include <Eigen/Dense>

#include "data_base.hpp"

namespace data {
struct FTSensor : public DataMsg {
  std::string name;
  Eigen::Vector3d force;
  Eigen::Vector3d torque;

  FTSensor(const std::string &name = "") :
      name(name), force(0.0, 0.0, 0.0), torque(0.0, 0.0, 0.0) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<FTSensor>(*this);
  }

  FTSensor &operator=(const FTSensor &other) {
    if (this != &other) {
      name = other.name;
      force = other.force;
      torque = other.torque;
    }
    return *this;
  }

  FTSensor(const FTSensor &other) :
      DataMsg(other), name(other.name), force(other.force), torque(other.torque) {
  }
};
} // namespace data
