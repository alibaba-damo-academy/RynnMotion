#include <chrono>
#include <iostream>

#include "pin_mj.hpp"

int main(int argc, char *argv[]) {
  bool enableRendering = false;
  bool compareJacobian = false;
  int robotNumber = 20; // Default to FR3
  int scenario = 0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--render" || arg == "-r") {
      enableRendering = true;
      std::cout << "Rendering enabled" << std::endl;
    } else if (arg == "--jaco" || arg == "-j") {
      compareJacobian = true;
      std::cout << "Jacobian comparison enabled" << std::endl;
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
      std::cout << "Options:" << std::endl;
      std::cout << "  --robot, -R <num>  Robot type number (default: 20)" << std::endl;
      std::cout << "                     20=FR3, 21=UR5E, 22=Piper, 23=RM75, 24=SO101" << std::endl;
      std::cout << "  --scenario, -s <n> Test scenario: 0=zero (5 steps), 1=lerp (10s) (default: 0)" << std::endl;
      std::cout << "  --jaco, -j         Compare Jacobian matrices (default: EE pose)" << std::endl;
      std::cout << "  --render, -r       Enable rendering (default: off)" << std::endl;
      std::cout << "  --help, -h         Show this help" << std::endl;
      return 0;
    } else {
      std::cerr << "Invalid argument: " << arg << std::endl;
      std::cerr << "Use --help for usage information" << std::endl;
      return 1;
    }
  }

  test::HybridKinematicsTest kineTest(robotNumber);
  kineTest.setPositionTolerance(1e-5);
  kineTest.setOrientationTolerance(1e-5);
  kineTest.setRenderingMode(enableRendering);

  if (!kineTest.initialize()) {
    std::cerr << "Failed to initialize test" << std::endl;
    return 1;
  }

  auto startTime = std::chrono::high_resolution_clock::now();

  bool success = kineTest.runKineComparison(scenario, compareJacobian);

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

  return success ? 0 : 1;
}
