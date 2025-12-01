#include <chrono>
#include <iostream>
#include <map>
#include <vector>

#include "pin_mj.hpp"

const std::map<int, std::vector<std::string>> ROBOT_SITE_MAP = {
    {20, {"shoulderSite", "elbowSite", "wristSite", "EE"}},  // FR3
    {21, {"shoulderSite", "elbowSite", "wristSite", "EE"}},  // UR5E
    {22, {"shoulderSite", "elbowSite", "wristSite", "EE"}},  // Piper
    {23, {"shoulderSite", "elbowSite", "wristSite", "EE"}},  // RM75
    {24, {"shoulderSite", "elbowSite", "wristSite", "EE"}},  // SO101
};

const std::map<int, std::string> ROBOT_NAME_MAP = {
    {20, "FR3"}, {21, "UR5E"}, {22, "Piper"}, {23, "RM75"}, {24, "SO101"}};

void printUsage(const char* programName) {
  std::cout << "Usage: " << programName << " [options]\n";
  std::cout << "Pinocchio + MuJoCo kinematics comparison test\n\n";
  std::cout << "Options:\n";
  std::cout << "  --robot, -R <num>  Robot number (default: 20)\n";
  std::cout << "                     20=FR3, 21=UR5E, 22=Piper, 23=RM75, 24=SO101\n";
  std::cout << "  --scenario, -s <n> Test scenario (default: 0)\n";
  std::cout << "                     0=zero angles (5 steps), 1=lerp motion (10s)\n";
  std::cout << "  --multi, -m        Multi-site mode (test shoulder, elbow, wrist, EE)\n";
  std::cout << "  --jaco, -j         Compare Jacobians (single-site mode only)\n";
  std::cout << "  --render, -r       Enable MuJoCo rendering\n";
  std::cout << "  --help, -h         Show this help\n";
  std::cout << "\nExamples:\n";
  std::cout << "  " << programName << "                  # Single EE test on FR3\n";
  std::cout << "  " << programName << " --multi          # Multi-site test on FR3\n";
  std::cout << "  " << programName << " -R 22 --multi    # Multi-site test on Piper\n";
  std::cout << "  " << programName << " --jaco           # Jacobian comparison on FR3\n";
}

int main(int argc, char* argv[]) {
  bool enableRendering = false;
  bool compareJacobian = false;
  bool multiSiteMode = false;
  int robotNumber = 20;
  int scenario = 0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--render" || arg == "-r") {
      enableRendering = true;
    } else if (arg == "--jaco" || arg == "-j") {
      compareJacobian = true;
    } else if (arg == "--multi" || arg == "-m") {
      multiSiteMode = true;
    } else if (arg == "--robot" || arg == "-R") {
      if (i + 1 < argc) {
        robotNumber = std::atoi(argv[++i]);
        if (robotNumber <= 0) {
          std::cerr << "Invalid robot number: " << argv[i] << std::endl;
          return 1;
        }
      } else {
        std::cerr << "--robot requires a number" << std::endl;
        return 1;
      }
    } else if (arg == "--scenario" || arg == "-s") {
      if (i + 1 < argc) {
        scenario = std::atoi(argv[++i]);
        if (scenario < 0 || scenario > 1) {
          std::cerr << "Invalid scenario: " << argv[i] << " (must be 0 or 1)" << std::endl;
          return 1;
        }
      } else {
        std::cerr << "--scenario requires a number (0 or 1)" << std::endl;
        return 1;
      }
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown option: " << arg << "\nUse --help for usage\n";
      return 1;
    }
  }

  if (compareJacobian && multiSiteMode) {
    std::cerr << "Warning: --jaco ignored in multi-site mode\n";
    compareJacobian = false;
  }

  std::string robotName = "Robot";
  auto nameIt = ROBOT_NAME_MAP.find(robotNumber);
  if (nameIt != ROBOT_NAME_MAP.end()) {
    robotName = nameIt->second;
  }

  bool success = false;
  auto startTime = std::chrono::high_resolution_clock::now();

  if (multiSiteMode) {
    auto siteIt = ROBOT_SITE_MAP.find(robotNumber);
    if (siteIt == ROBOT_SITE_MAP.end()) {
      std::cerr << "Robot " << robotNumber << " has no multi-site config\n";
      std::cerr << "Supported: 20 (FR3), 21 (UR5E), 22 (Piper), 23 (RM75), 24 (SO101)\n";
      return 1;
    }

    const std::vector<std::string>& siteNames = siteIt->second;

    std::cout << "\n=== Multi-Site Kinematics Test ===" << std::endl;
    std::cout << "Robot: " << robotName << " (" << robotNumber << ")" << std::endl;
    std::cout << "Scenario: " << scenario << (scenario == 0 ? " (zero)" : " (lerp)") << std::endl;
    std::cout << "Sites: ";
    for (size_t i = 0; i < siteNames.size(); ++i) {
      std::cout << siteNames[i];
      if (i < siteNames.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    test::HybridKinematicsTest kineTest(robotNumber, siteNames);
    kineTest.setPositionTolerance(1e-3);
    kineTest.setOrientationTolerance(1e-2);
    kineTest.setRenderingMode(enableRendering);

    if (!kineTest.initialize()) {
      std::cerr << "Failed to initialize multi-site test" << std::endl;
      return 1;
    }

    success = kineTest.runMultiSiteComparison(scenario);

  } else {
    std::cout << "\n=== Single-Site Kinematics Test ===" << std::endl;
    std::cout << "Robot: " << robotName << " (" << robotNumber << ")" << std::endl;
    std::cout << "Scenario: " << scenario << (scenario == 0 ? " (zero)" : " (lerp)") << std::endl;
    if (compareJacobian) std::cout << "Mode: Jacobian comparison" << std::endl;

    test::HybridKinematicsTest kineTest(robotNumber);
    kineTest.setPositionTolerance(1e-5);
    kineTest.setOrientationTolerance(1e-5);
    kineTest.setRenderingMode(enableRendering);

    if (!kineTest.initialize()) {
      std::cerr << "Failed to initialize single-site test" << std::endl;
      return 1;
    }

    success = kineTest.runKineComparison(scenario, compareJacobian);
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "\nCompleted in " << duration.count() << " ms" << std::endl;

  return success ? 0 : 1;
}
