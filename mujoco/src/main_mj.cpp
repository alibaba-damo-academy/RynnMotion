#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include "debug_config.hpp"
#include "discovery.hpp"
#include "mj_interface.hpp"
#include "robot_manager.hpp"
#include "scene_manager.hpp"

/**
 * MuJoCo Robot Simulation Launcher
 *
 * Supports both robot names and legacy numbers for better usability:
 *
 * Usage Examples:
 *   ./mujocoExe fr3 1          # Robot name + scene (recommended)
 *   ./mujocoExe 20 1                  # Robot number + scene (same as above)
 *   ./mujocoExe piper 1               # Piper robot (handles gripper/pen via scenes)
 *   ./mujocoExe dual_rm75 2           # Dual arm setup, scene 2
 *   ./mujocoExe --help                # Show all available robots
 *
 * Available Robot Names (with numbers):
 *   Low DOF: oneLink(1), twoLink(2)
 *   Single Arms: fr3(20), ur5e(21), piper(22), rm75(23), eco65(24)
 *   LeRobot: lerobot(30), lekiwi(31), xlerobot(32)
 *   Dual Arms: dual_rm75(43), dual_eco65(44)
 *   Mobile: diffMobile(50), diffCar(51), diffMobile_Franka(60)
 *
 * Error Handling:
 *   - Invalid robot names show list of available robots
 *   - Case-insensitive matching with warnings (e.g., "fr3" → "fr3")
 *   - Missing arguments show usage help
 */

void printUsage(const char *programName) {
  std::cout << "Usage: " << programName << " [options] <robot> [scene]\n";
  std::cout << "\n";
  std::cout << "Options:\n";
  std::cout << "  -v, --verbose    Enable verbose debug output\n";
  std::cout << "  -h, --help       Show this help message\n";
  std::cout << "\n";
  std::cout << "Arguments:\n";
  std::cout << "  robot        Robot name (e.g., 'fr3') or number (e.g., '20')\n";
  std::cout << "  scene        Scene name (e.g., 'ui', 'pickplace') or number (1-5, default: 1)\n";
  std::cout << "\n";
  std::cout << "Available scenes:\n";
  std::cout << "  joint (1)        Joint range motion\n";
  std::cout << "  keyframe (2)     Keyframe cycling\n";
  std::cout << "  ui (3)           UI tracking / keyboard control\n";
  std::cout << "  predefined (4)   Workspace predefined motion\n";
  std::cout << "  pickplace (5)    Pick and place tasks\n";
  std::cout << "\n";
  std::cout << "Available robot names:\n";

  auto &discovery = rynn::RobotDiscovery::getInstance();
  auto robotNames = discovery.getAllRobotNames();
  for (size_t i = 0; i < robotNames.size(); ++i) {
    if (i % 3 == 0) std::cout << "  ";
    std::cout << std::setw(18) << std::left << robotNames[i];
    if ((i + 1) % 3 == 0) std::cout << "\n";
  }
  if (robotNames.size() % 3 != 0) std::cout << "\n";

  std::cout << "\nExamples:\n";
  std::cout << "  " << programName << " fr3 ui\n";
  std::cout << "  " << programName << " piper pickplace\n";
  std::cout << "  " << programName << " dual_rm75 tracking\n";
  std::cout << "  " << programName << " fr3 1              # Scene numbers still work\n";
}

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
    if (sceneNum >= 1 && sceneNum <= 5) {
      return sceneNum;
    }
  } catch (...) {
  }

  rynn::SceneType sceneType = rynn::SceneManager::sceneNameToType(sceneInput);
  return static_cast<int>(sceneType);
}

int main(int argc, char *argv[]) {
  auto &discovery = rynn::RobotDiscovery::getInstance();
  discovery.scanRobots(MODEL_DIR);

  if (argc < 2) {
    std::cerr << "Error: Robot argument required\n\n";
    printUsage(argv[0]);
    return 1;
  }

  bool verboseMode = false;
  int argIndex = 1;

  while (argIndex < argc && argv[argIndex][0] == '-') {
    std::string flag(argv[argIndex]);

    if (flag == "--help" || flag == "-h") {
      printUsage(argv[0]);
      return 0;
    } else if (flag == "--verbose" || flag == "-v") {
      verboseMode = true;
      argIndex++;
    } else {
      std::cerr << "Error: Unknown option '" << flag << "'\n\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  if (verboseMode) {
    utils::DebugConfig::getInstance().setVerbose(true);
  }

  if (argIndex >= argc) {
    std::cerr << "Error: Robot argument required\n\n";
    printUsage(argv[0]);
    return 1;
  }

  int robotNumber = 1;
  int sceneNumber = 1;

  try {
    robotNumber = parseRobotInput(argv[argIndex]);
  } catch (const std::exception &e) {
    std::cerr << "\n";
    printUsage(argv[0]);
    return 1;
  }
  argIndex++;

  if (argIndex < argc) {
    try {
      sceneNumber = parseSceneInput(argv[argIndex]);
    } catch (std::exception &) {
      std::cerr << "Error: Invalid scene '" << argv[argIndex] << "'\n";
      std::cerr << "Valid scenes: joint (1), keyframe (2), ui (3), predefined (4), pickplace (5)\n";
      return 1;
    }
  }

  std::string robotName;
  try {
    auto &discovery = rynn::RobotDiscovery::getInstance();
    auto info = discovery.getRobotInfo(robotNumber);
    robotName = info.canonicalName;
  } catch (...) {
    robotName = "robot #" + std::to_string(robotNumber);
  }

  std::string sceneName = rynn::SceneManager::sceneNumberToName(sceneNumber);
  std::string sceneDescription;
  switch (sceneNumber) {
  case 1: sceneDescription = "scene '" + sceneName + "' (1): joint range motion"; break;
  case 2: sceneDescription = "scene '" + sceneName + "' (2): keyframe cycling"; break;
  case 3: sceneDescription = "scene '" + sceneName + "' (3): keyboard UI / end-effector tracking"; break;
  case 4: sceneDescription = "scene '" + sceneName + "' (4): workspace predefined motion"; break;
  case 5: sceneDescription = "scene '" + sceneName + "' (5): pick and place"; break;
  default: sceneDescription = "scene " + std::to_string(sceneNumber); break;
  }

  std::cout << "Starting MuJoCo with " << robotName << ", " << sceneDescription << std::endl;

  std::filesystem::path motionConfigPath = "../config/motion.yaml";
  std::filesystem::path mujocoConfigPath = "../config/mujoco.yaml";

  YAML::Node config1, config2;
  try {
    config1 = YAML::LoadFile(mujocoConfigPath.string());
    config2 = YAML::LoadFile(motionConfigPath.string());
  } catch (const YAML::Exception &e) {
    std::cerr << "Failed to load config file: " << e.what() << std::endl;
  }

  auto mujocoSimulation = std::make_shared<mujoco::MujocoInterface>(config1, config2, robotNumber, sceneNumber);
  mujocoSimulation->runApplication();
  return 0;
}
