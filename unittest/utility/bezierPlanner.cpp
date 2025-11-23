#include "bezierPlanner.hpp"

#include <cassert>
#include <iostream>

BezierPlanner::BezierPlanner(const Eigen::Vector3d &start, const Eigen::Vector3d &end, double totalTime, double timeStep) :
    start_(start), end_(end), totalTime_(totalTime), timeStep_(timeStep) {
  assert(totalTime > 0);
  assert(timeStep > 0);
}

void BezierPlanner::update(double t) {
  double normalizedTime = t / totalTime_;
  double x = utils::cubicBezier(start_.x(), end_.x(), normalizedTime);
  double y = utils::cubicBezier(start_.y(), end_.y(), normalizedTime);
  double z = utils::cubicBezier(start_.z(), end_.z(), normalizedTime);

  eePos_ = Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d BezierPlanner::getDesiredEePos() const {
  return eePos_;
}

void BezierPlanner::setTotalTime(double totalTime) {
  assert(totalTime > 0);
  totalTime_ = totalTime;
}

void BezierPlanner::setTimeStep(double timeStep) {
  assert(timeStep > 0);
  timeStep_ = timeStep;
}

std::vector<Eigen::Vector3d> BezierPlanner::generateTrajectory() {
  std::vector<Eigen::Vector3d> trajectory;
  double t = 0.0;

  while (t <= totalTime_) {
    double normalizedTime = t / totalTime_;
    double x = utils::cubicBezier(start_.x(), end_.x(), normalizedTime);
    double y = utils::cubicBezier(start_.y(), end_.y(), normalizedTime);
    double z = utils::cubicBezier(start_.z(), end_.z(), normalizedTime);
    trajectory.emplace_back(Eigen::Vector3d(x, y, z));
    t += timeStep_;
  }

  trajectory.emplace_back(end_);
  return trajectory;
}
