#include "discovery.hpp"

#include <iomanip>
#include <iostream>

using namespace rynn;
using DiscoveredRobotInfo = rynn::DiscoveredRobotInfo;
using DiscoveredSceneInfo = rynn::DiscoveredSceneInfo;

int main() {
  std::cout << "=== Robot Discovery System Test ===" << std::endl;
  std::cout << std::endl;

  // Initialize discovery
  auto& discovery = RobotDiscovery::getInstance();

  try {
    // Scan models directory
    std::string modelsDir = MODEL_DIR;
    std::cout << "Scanning models directory: " << modelsDir << std::endl;
    std::cout << std::endl;

    discovery.scanRobots(modelsDir);

    // Get all robots
    auto robots = discovery.getAllRobots();

    std::cout << std::endl;
    std::cout << "=== Discovered Robots ===" << std::endl;
    std::cout << std::setw(6) << "Number"
              << " | " << std::setw(20) << "Name"
              << " | " << std::setw(10) << "Scenes"
              << " | " << "Aliases" << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    for (const auto& robot : robots) {
      std::cout << std::setw(6) << robot.number
                << " | " << std::setw(20) << robot.canonicalName
                << " | " << std::setw(10) << robot.scenes.size()
                << " | ";

      // Print first 3 aliases
      for (size_t i = 0; i < std::min(size_t(3), robot.aliases.size()); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << robot.aliases[i];
      }
      if (robot.aliases.size() > 3) {
        std::cout << "...";
      }
      std::cout << std::endl;
    }

    std::cout << std::endl;

    // Test specific robot lookups
    std::cout << "=== Testing Robot Lookups ===" << std::endl;

    std::vector<std::string> testNames = {"fr3", "FR3", "franka", "20", "dual_fr3", "panda"};
    for (const auto& testName : testNames) {
      try {
        auto info = discovery.getRobotInfo(testName);
        std::cout << "✓ '" << testName << "' → Robot #" << info.number
                  << " (" << info.canonicalName << ")" << std::endl;
      } catch (const std::exception& e) {
        std::cout << "✗ '" << testName << "' → " << e.what() << std::endl;
      }
    }

    std::cout << std::endl;

    // Test scene lookups
    std::cout << "=== Testing Scene Discovery (FR3) ===" << std::endl;
    try {
      auto fr3Info = discovery.getRobotInfo("fr3");
      std::cout << "FR3 has " << fr3Info.scenes.size() << " scenes:" << std::endl;

      for (const auto& scene : fr3Info.scenes) {
        std::cout << "  Scenes [";
        for (size_t i = 0; i < scene.numbers.size(); ++i) {
          std::cout << scene.numbers[i];
          if (i < scene.numbers.size() - 1) std::cout << ",";
        }
        std::cout << "]: " << std::setw(20) << std::left << scene.name
                  << " (" << scene.filename << ")" << std::endl;
      }

      // Test scene lookup by number and name
      std::cout << std::endl;
      if (!fr3Info.scenes.empty()) {
        auto scene1 = SceneDiscovery::getScene(fr3Info.scenes, "1");
        std::cout << "Scene lookup by number (1): " << scene1.name << std::endl;

        // Try to find tracking scene
        bool hasTracking = false;
        for (const auto& scene : fr3Info.scenes) {
          if (scene.name == "tracking") {
            hasTracking = true;
            break;
          }
        }

        if (hasTracking) {
          auto trackingScene = SceneDiscovery::getScene(fr3Info.scenes, "tracking");
          std::cout << "Scene lookup by name (tracking): " << trackingScene.fullPath << std::endl;
        }
      }

    } catch (const std::exception& e) {
      std::cout << "Error testing FR3 scenes: " << e.what() << std::endl;
    }

    std::cout << std::endl;

    // Test paths
    std::cout << "=== Testing File Paths (FR3) ===" << std::endl;
    try {
      auto fr3Info = discovery.getRobotInfo("fr3");
      std::cout << "Pinocchio XML: " << fr3Info.pinocchioPath << std::endl;
      std::cout << "Robot MJCF:    " << fr3Info.robotMjcfPath << std::endl;
      std::cout << "Base Path:     " << fr3Info.basePath << std::endl;
      if (fr3Info.configPath) {
        std::cout << "Config:        " << *fr3Info.configPath << std::endl;
      } else {
        std::cout << "Config:        (none)" << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Error: " << e.what() << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  std::cout << std::endl;
  std::cout << "=== Test Complete ===" << std::endl;
  return 0;
}
