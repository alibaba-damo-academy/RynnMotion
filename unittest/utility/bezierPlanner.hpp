#pragma once

#include <vector>

#include "interpolate.hpp"

class BezierPlanner {
public:
  BezierPlanner(const Eigen::Vector3d &start, const Eigen::Vector3d &end, double totalTime, double timeStep);

  void update(double t);

  Eigen::Vector3d getDesiredEePos() const;

  void setTotalTime(double totalTime);
  void setTimeStep(double timeStep);

private:
  std::vector<Eigen::Vector3d> generateTrajectory();
  Eigen::Vector3d eePos_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  double totalTime_;
  double timeStep_;
};
