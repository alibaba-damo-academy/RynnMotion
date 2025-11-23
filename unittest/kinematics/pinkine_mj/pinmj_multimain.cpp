#include <chrono>
#include <iostream>
#include <map>
#include <vector>

#include "pin_mj.hpp"

// Define site names for each robot
std::map<int, std::vector<std::string>> robotSiteMap = {
    {20, {"shoulderSite", "elbowSite", "wristSite", "EE"}}, // FR3
    {21, {"shoulderSite", "elbowSite", "wristSite", "EE"}}, // UR5E
    {22, {"shoulderSite", "elbowSite", "wristSite", "EE"}}, // Piper
    {23, {"shoulderSite", "elbowSite", "wristSite", "EE"}}, // RM75
    {24, {"shoulderSite", "elbowSite", "wristSite", "EE"}}, // SO101
};

int main(int argc, char *argv[]) {
  bool enableRendering = false;
  int robotNumber = 20; // Default to FR3
  int scenario = 0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--render" || arg == "-r") {
      enableRendering = true;
      std::cout << "Rendering enabled" << std::endl;
    } else if (arg == "--robot" || arg == "-R") {
      if (i + 1 < argc) {
        robotNumber = std::atoi(argv[++i]);
        if (robotNumber <= 0) {
          std::cerr << "Invalid robot number: " << argv[i] << std::endl;
          return 1;
        }
      } else {
        std::cerr << "--robot option requires a robot number" << std::endl;
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
        std::cerr << "--scenario option requires a scenario number (0 or 1)" << std::endl;
        return 1;
      }
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
      std::cout << "Multi-site kinematics comparison test" << std::endl;
      std::cout << "\nOptions:" << std::endl;
      std::cout << "  --robot, -R <num>  Robot type number (default: 20)" << std::endl;
      std::cout << "                     20=FR3, 21=UR5E, 22=Piper, 23=RM75, 24=SO101" << std::endl;
      std::cout << "  --scenario, -s <n> Test scenario: 0=zero (5 steps), 1=lerp (10s) (default: 0)" << std::endl;
      std::cout << "  --render, -r       Enable rendering (default: off)" << std::endl;
      std::cout << "  --help, -h         Show this help" << std::endl;
      std::cout << "\nSupported robots and sites:" << std::endl;
      std::cout << "  FR3 (20):   shoulderSite, elbowSite, wristSite, EE" << std::endl;
      std::cout << "  UR5E (21):  shoulderSite, elbowSite, wristSite, EE" << std::endl;
      std::cout << "  Piper (22): shoulderSite, elbowSite, wristSite, EE" << std::endl;
      std::cout << "  RM75 (23):  shoulderSite, elbowSite, wristSite, EE" << std::endl;
      std::cout << "  SO101 (24): shoulderSite, elbowSite, wristSite, EE" << std::endl;
      return 0;
    } else {
      std::cerr << "Invalid argument: " << arg << std::endl;
      std::cerr << "Use --help for usage information" << std::endl;
      return 1;
    }
  }

  // Check if robot is supported
  if (robotSiteMap.find(robotNumber) == robotSiteMap.end()) {
    std::cerr << "Robot " << robotNumber << " does not have multi-site configuration" << std::endl;
    std::cerr << "Supported robots: 20 (FR3), 21 (UR5E), 22 (Piper), 23 (RM75), 24 (SO101)" << std::endl;
    return 1;
  }

  // Get site names for this robot
  std::vector<std::string> siteNames = robotSiteMap[robotNumber];

  // Robot name map
  std::map<int, std::string> robotNameMap = {
      {20, "FR3"}, {21, "UR5E"}, {22, "Piper"}, {23, "RM75"}, {24, "SO101"}};

  std::cout << "\n=== Multi-Site Kinematics Test ===" << std::endl;
  std::cout << "Robot: " << robotNameMap[robotNumber] << " (" << robotNumber << ")" << std::endl;
  std::cout << "Scenario: " << scenario << (scenario == 0 ? " (zero angles)" : " (lerp motion)") << std::endl;
  std::cout << "Sites: ";
  for (size_t i = 0; i < siteNames.size(); ++i) {
    std::cout << siteNames[i];
    if (i < siteNames.size() - 1)
      std::cout << ", ";
  }
  std::cout << std::endl;

  // Create test instance in multi-site mode
  test::HybridKinematicsTest kineTest(robotNumber, siteNames);
  kineTest.setPositionTolerance(1e-3);
  kineTest.setOrientationTolerance(1e-2);
  kineTest.setRenderingMode(enableRendering);

  if (!kineTest.initialize()) {
    std::cerr << "Failed to initialize test" << std::endl;
    return 1;
  }

  auto startTime = std::chrono::high_resolution_clock::now();

  bool success = kineTest.runMultiSiteComparison(scenario);

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

  std::cout << "\nTest completed in " << duration.count() << " ms" << std::endl;

  return success ? 0 : 1;
}
