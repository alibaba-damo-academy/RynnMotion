#include <filesystem>
#include <iostream>
#include <string>

#include "discovery.hpp"
#include "robot_manager.hpp"
#include "ros_mj_interface.hpp"
#include "scene_manager.hpp"

/**
 * ROS MuJoCo Robot Simulation Launcher
 *
 * Supports both robot names and legacy numbers for better usability:
 *
 * Usage Examples:
 *   ./rosMujocoExe fr3         # Robot name (recommended)
 *   ./rosMujocoExe 20                 # Robot number (same as above)
 *   ./rosMujocoExe piper              # Piper robot (handles gripper/pen via scenes)
 *   ./rosMujocoExe dual_rm75          # Dual arm setup
 *
 * Available Robot Names (with numbers):
 *   Low DOF: oneLink(1), twoLink(2)
 *   Single Arms: fr3(20), ur5e(21), piper(22), rm75(23), eco65(24)
 *   LeRobot: lerobot(30), lekiwi(31), xlerobot(32)
 *   Dual Arms: dual_rm75(43), dual_eco65(44)
 *   Mobile: diffMobile(50), diffCar(51), diffMobile_Franka(60)
 *
 * Error Handling:
 *   - Invalid robot names show error message with available options
 *   - Case-insensitive matching with warnings
 *   - Defaults to oneLink (robot #1) if no argument provided
 */

int parseRobotInput(const std::string &robotInput) {
  auto &discovery = rynn::RobotDiscovery::getInstance();

  try {
    // Discovery handles both names and numbers
    auto info = discovery.getRobotInfo(robotInput);
    return info.number;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    throw;
  }
}

int parseSceneInput(const std::string &sceneInput) {
  try {
    int sceneNum = std::stoi(sceneInput);
    if (sceneNum >= 1 && sceneNum <= 6) {
      return sceneNum;
    }
  } catch (...) {
  }

  rynn::SceneType sceneType = rynn::SceneManager::sceneNameToType(sceneInput);
  return static_cast<int>(sceneType);
}

int main(int argc, char *argv[]) {
  // Initialize robot discovery
  auto &discovery = rynn::RobotDiscovery::getInstance();
  discovery.scanRobots(MODEL_DIR);

  int robotNumber = 1;
  int sceneNumber = 1;

  if (argc > 1) {
    try {
      robotNumber = parseRobotInput(argv[1]);
    } catch (const std::exception &e) {
      std::cerr << "Error parsing robot input. Use robot name or number." << std::endl;
      return 1;
    }
  }

  // Parse scene argument if provided
  if (argc > 2) {
    try {
      sceneNumber = parseSceneInput(argv[2]);
    } catch (const std::exception &e) {
      std::cerr << "Error parsing scene input. Use scene name or number." << std::endl;
      return 1;
    }
  }

  std::string sceneName = rynn::SceneManager::sceneNumberToName(sceneNumber);
  std::cout << "Starting ROS MuJoCo simulation with robot #" << robotNumber
            << ", scene '" << sceneName << "' (" << sceneNumber << ")" << std::endl;

  std::filesystem::path motionConfigPath = "../config/motion.yaml";
  std::filesystem::path mujocoConfigPath = "../config/mujoco.yaml";

  YAML::Node config1, config2;
  try {
    config1 = YAML::LoadFile(mujocoConfigPath.string());
    config2 = YAML::LoadFile(motionConfigPath.string());
  } catch (const YAML::Exception &e) {
    std::cerr << "Failed to load config file: " << e.what() << std::endl;
  }

  auto rosMujocoSimulation = std::make_shared<mujoco::RosMujocoInterface>(config1, config2, robotNumber, sceneNumber);
  rosMujocoSimulation->runApplication();
  return 0;
}
