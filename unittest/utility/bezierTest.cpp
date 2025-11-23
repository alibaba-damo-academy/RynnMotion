#include <filesystem>
#include <iostream>

#include "bezierPlanner.hpp"
#include "interpolate.hpp"

std::vector<Eigen::Vector3d> generateTrajectory(const Eigen::Vector3d &start, const Eigen::Vector3d &end, double totalTime, double timeStep) {
  assert(totalTime > 0);
  assert(timeStep > 0);

  std::vector<Eigen::Vector3d> trajectory;
  double t = 0.0;

  while (t <= totalTime) {
    double normalizedTime = t / totalTime; // Normalize time between 0 and 1

    double x = utils::cubicBezier(start.x(), end.x(), normalizedTime);
    double y = utils::cubicBezier(start.y(), end.y(), normalizedTime);
    double z = utils::cubicBezier(start.z(), end.z(), normalizedTime);

    trajectory.emplace_back(Eigen::Vector3d(x, y, z));
    t += timeStep;
  }

  trajectory.emplace_back(end);
  return trajectory;
}

void bezierTest1() {
  Eigen::Vector3d start(1.0, 1.2, 0.0);
  Eigen::Vector3d end(0.8, 1.6, 0.0);
  double totalTime = 1.2;
  double timeStep = 0.002;

  auto trajectory = generateTrajectory(start, end, totalTime, timeStep);

  std::cout << "**********************************" << std::endl;
  std::cout << "begin of bezierTest1" << std::endl;
  for (const auto &point : trajectory) {
    std::cout << "x: " << point.x() << ", y: " << point.y() << ", z: " << point.z() << std::endl;
  }
  std::cout << "end of bezierTest1" << std::endl;
  std::cout << "**********************************" << std::endl;
}

void bezierTest2() {
  double totalTime = 1.2;
  double dt = 0.002;
  double t = 0.0;
  Eigen::Vector3d start(1.0, 1.2, 0.0);
  Eigen::Vector3d end(0.8, 1.6, 0.0);
  BezierPlanner planner(start, end, totalTime, dt);
  Eigen::Vector3d eePos;
  std::cout << "**********************************" << std::endl;
  std::cout << "begin of bezierTest2" << std::endl;
  while (t <= totalTime) {
    planner.update(t);
    eePos = planner.getDesiredEePos();
    std::cout << "time: " << t << " sec, desired eePos x: " << eePos.x() << ", y: " << eePos.y() << ", z: " << eePos.z() << std::endl;
    t += dt;
  }
  std::cout << "end of bezierTest2" << std::endl;
  std::cout << "**********************************" << std::endl;
}

int main() {
  bezierTest1();
  bezierTest2();
  return 0;
}
