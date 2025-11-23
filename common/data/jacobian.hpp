#pragma once
#include <Eigen/Dense>

#include "data_base.hpp"

namespace data {
struct Jacobian : public DataMsg {
  std::string name;
  Eigen::MatrixXd jaco;

  Jacobian(const std::string &name = "", int rows = 0, int cols = 0) :
      name(name), jaco(Eigen::MatrixXd::Zero(rows, cols)) {
  }

  std::shared_ptr<DataMsg> clone() const override {
    return std::make_shared<Jacobian>(*this);
  }

  Jacobian &operator=(const Jacobian &other) {
    if (this != &other) {
      name = other.name;
      jaco = other.jaco;
    }
    return *this;
  }

  Jacobian(const Jacobian &other) :
      DataMsg(other), name(other.name), jaco(other.jaco) {
  }
};
} // namespace data