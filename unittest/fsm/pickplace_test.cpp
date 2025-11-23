#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "pickplace_fsm.hpp"

int main() {
  std::cout << "Starting PickPlace FSM test..." << std::endl;

  // Define poses
  data::Pose standPose;
  standPose.pos = Eigen::Vector3d(0.0, 0.0, 0.5);
  standPose.quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  data::Pose placePose;
  placePose.pos = Eigen::Vector3d(0.5, 0.0, 0.3);
  placePose.quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Create object poses
  std::vector<data::Pose> objectPoses;
  for (int i = 0; i < 2; ++i) {
    data::Pose objPose;
    objPose.pos = Eigen::Vector3d(0.2 + i * 0.1, 0.2, 0.1);
    objPose.quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    objectPoses.push_back(objPose);
  }

  // Start the FSM
  rynn::PickPlaceFsm::start();

  constexpr float dt = 0.002f;

  // Initialize the pick and place sequence
  rynn::PickPlaceFsm::dispatch(rynn::PickPlaceEventStart{standPose, placePose, objectPoses, dt});
  float sim_time = 0.0f;
  float last_output_time = 0.0f;

  std::cout << "Time: " << std::fixed << std::setprecision(1) << sim_time << "s" << std::endl;

  // Run until pick and place sequence is completed
  while (!rynn::PickPlaceFsm::isCompleted()) {
    // Dispatch tick event
    rynn::PickPlaceFsm::dispatch(rynn::PickPlaceEventTick{dt});

    // Get current target pose and gripper state
    auto targetPose = rynn::PickPlaceFsm::getCurrentTargetPose();
    bool openGripper = rynn::PickPlaceFsm::shouldOpenGripper();

    // Output progress information
    if (sim_time - last_output_time >= 1.0f) {
      std::cout << "Time: " << std::fixed << std::setprecision(1) << sim_time << "s"
                << " | Gripper: " << (openGripper ? "Open" : "Closed")
                << " | Pos: [" << std::setprecision(3)
                << targetPose.pos.x() << ", "
                << targetPose.pos.y() << ", "
                << targetPose.pos.z() << "]"
                << " | Quat: [" << std::setprecision(3)
                << targetPose.quat.w() << ", "
                << targetPose.quat.x() << ", "
                << targetPose.quat.y() << ", "
                << targetPose.quat.z() << "]"
                << std::endl;
      last_output_time = sim_time;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    sim_time += dt;

    // Safety timeout to prevent infinite loop
    if (sim_time > 60.0f) {
      std::cout << "Safety timeout reached!" << std::endl;
      break;
    }
  }

  std::cout << "\nPickPlace FSM test completed after " << std::fixed << std::setprecision(1)
            << sim_time << "s" << std::endl;

  return 0;
}
