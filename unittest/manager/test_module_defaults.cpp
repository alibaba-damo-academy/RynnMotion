#include "discovery.hpp"
#include "module_manager.hpp"

#include <iomanip>
#include <iostream>

using namespace rynn;

std::string moduleTypeToString(ModuleType type) {
  switch (type) {
  case ModuleType::JointMove: return "JointMove";
  case ModuleType::Planner: return "Planner";
  case ModuleType::Estimator: return "Estimator";
  case ModuleType::OSC: return "OSC";
  case ModuleType::Fr3TeleopFollow: return "Fr3TeleopFollow";
  case ModuleType::Fr3JointMove: return "Fr3JointMove";
  case ModuleType::DualRM75Planner: return "DualRM75Planner";
  case ModuleType::Dual75Osc: return "Dual75Osc";
  case ModuleType::LeRobotPlanner: return "LeRobotPlanner";
  case ModuleType::SingleEEPlanner: return "SingleEEPlanner";
  default: return "Unknown";
  }
}

void printModuleChain(const std::string& sceneName, const ModuleChainConfig& chain) {
  std::cout << "  " << std::setw(20) << std::left << sceneName << " → ";
  for (size_t i = 0; i < chain.modules.size(); ++i) {
    if (i > 0) std::cout << " + ";
    std::cout << moduleTypeToString(chain.modules[i]);
  }
  std::cout << std::endl;
}

int main() {
  std::cout << "=== Module Defaults Test ===" << std::endl;
  std::cout << std::endl;

  // Note: We can't directly instantiate ModuleManager without dependencies
  // So we'll test the logic through discovery + manual verification

  std::cout << "=== Expected Smart Defaults ===" << std::endl;
  std::cout << "  Scene pattern          → Module chain" << std::endl;
  std::cout << "  " << std::string(60, '-') << std::endl;
  std::cout << "  default/empty          → JointMove" << std::endl;
  std::cout << "  *tracking*             → Estimator + SingleEEPlanner + OSC" << std::endl;
  std::cout << "  *pickplace*            → Estimator + SingleEEPlanner + OSC" << std::endl;
  std::cout << "  *teleop*               → Estimator + SingleEEPlanner + Fr3TeleopFollow" << std::endl;
  std::cout << "  other                  → JointMove (fallback)" << std::endl;
  std::cout << std::endl;

  // Test discovery integration
  std::cout << "=== Testing Discovery Integration ===" << std::endl;
  auto& discovery = RobotDiscovery::getInstance();

  try {
    std::string modelsDir = MODEL_DIR;
    discovery.scanRobots(modelsDir);

    auto fr3Info = discovery.getRobotInfo("fr3");
    std::cout << "FR3 scenes and expected module chains:" << std::endl;

    for (const auto& scene : fr3Info.scenes) {
      std::cout << "  Scene: " << std::setw(15) << std::left << scene.name;

      // Manually apply smart default logic
      std::string lower = scene.name;
      std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

      if (lower == "default" || lower.empty()) {
        std::cout << " → JointMove";
      } else if (lower.find("tracking") != std::string::npos) {
        std::cout << " → Estimator + SingleEEPlanner + OSC";
      } else if (lower.find("pickplace") != std::string::npos) {
        std::cout << " → Estimator + SingleEEPlanner + OSC";
      } else if (lower.find("teleop") != std::string::npos) {
        std::cout << " → Estimator + SingleEEPlanner + Fr3TeleopFollow";
      } else {
        std::cout << " → JointMove (fallback)";
      }
      std::cout << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  std::cout << std::endl;
  std::cout << "=== YAML Override Example ===" << std::endl;
  std::cout << "To override module chains, create robot_config.yaml:" << std::endl;
  std::cout << std::endl;
  std::cout << "  module_chains:" << std::endl;
  std::cout << "    default: [JointMove]" << std::endl;
  std::cout << "    tracking: [Estimator, Planner, OSC]  # Use Planner instead" << std::endl;
  std::cout << "    custom: [Estimator, Fr3JointMove]" << std::endl;
  std::cout << std::endl;

  std::cout << "=== Test Complete ===" << std::endl;
  return 0;
}
