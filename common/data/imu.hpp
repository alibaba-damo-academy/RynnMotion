#pragma once
#include <Eigen/Dense>

#include "data_base.hpp"

namespace data {
struct Imu : public DataMsg {
  std::string name;        // Name of the IMU
  Eigen::Quaterniond quat; // Quaternion (w, x, y, z)
  Eigen::Vector3d rpy;     // Roll, Pitch, Yaw
  Eigen::Vector3d angVel;  // Angular velocity (x, y, z)
  Eigen::Vector3d linAcc;  // Linear acceleration (x, y, z)

  Imu(const std::string &name = "") :
      name(name), quat(1.0, 0.0, 0.0, 0.0), rpy(0.0, 0.0, 0.0), angVel(0.0, 0.0, 0.0), linAcc(0.0, 0.0, 0.0) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<Imu>(*this);
  }

  Imu &operator=(const Imu &other) {
    if (this != &other) {
      name = other.name;
      quat = other.quat;
      rpy = other.rpy;
      angVel = other.angVel;
      linAcc = other.linAcc;
    }
    return *this;
  }

  Imu(const Imu &other) :
      DataMsg(other), name(other.name), quat(other.quat), rpy(other.rpy), angVel(other.angVel), linAcc(other.linAcc) {
  }
};

} // namespace data