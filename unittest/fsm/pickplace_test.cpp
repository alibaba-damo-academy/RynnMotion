#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "pickplace_fsm.hpp"

int main() {
  std::cout << "Starting PickPlace FSM test..." << std::endl;

  data::Pose standPose;
  standPose.pos = Eigen::Vector3d(0.0, 0.0, 0.5);
  standPose.quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  data::Pose placePose;
  placePose.pos = Eigen::Vector3d(0.5, 0.0, 0.3);
  placePose.quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  std::vector<data::Pose> objectPoses;
  for (int i = 0; i < 2; ++i) {
    data::Pose objPose;
    objPose.pos = Eigen::Vector3d(0.2 + i * 0.1, 0.2, 0.1);
    objPose.quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    objectPoses.push_back(objPose);
  }

  rynn::PickPlaceFsm fsm(0);
  fsm.start();

  constexpr float dt = 0.002f;
  fsm.dispatch(rynn::PickPlaceEventStart{standPose, placePose, objectPoses, dt});

  float simTime = 0.0f;
  float lastOutputTime = 0.0f;

  std::cout << "Time: " << std::fixed << std::setprecision(1) << simTime << "s" << std::endl;

  while (!fsm.isCompleted()) {
    fsm.dispatch(rynn::PickPlaceEventTick{dt});

    auto targetPose = fsm.getCurrentTargetPose();
    bool openGripper = fsm.shouldOpenGripper();

    if (simTime - lastOutputTime >= 1.0f) {
      std::cout << "Time: " << std::fixed << std::setprecision(1) << simTime << "s"
                << " | State: " << static_cast<int>(fsm.getCurrentState())
                << " | Gripper: " << (openGripper ? "Open" : "Closed") << " | Pos: ["
                << std::setprecision(3) << targetPose.pos.x() << ", " << targetPose.pos.y() << ", "
                << targetPose.pos.z() << "]" << std::endl;
      lastOutputTime = simTime;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    simTime += dt;

    if (simTime > 60.0f) {
      std::cout << "Safety timeout reached!" << std::endl;
      break;
    }
  }

  std::cout << "\nPickPlace FSM test completed after " << std::fixed << std::setprecision(1)
            << simTime << "s" << std::endl;

  return 0;
}
