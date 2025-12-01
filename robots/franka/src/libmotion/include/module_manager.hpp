#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "estimator.hpp"
#include "fr3_jointmove.hpp"
#include "fr3_teleopfollow.hpp"
#include "joint_move.hpp"
#include "lcm_logger.hpp"
#include "osc.hpp"
#include "planner.hpp"
#include "runtime_data.hpp"
#include "scene_manager.hpp"

namespace rynn {

/**
 * @brief Enum for module types to be used in module creation virtual table
 */
enum class ModuleType {
  JointMove,
  Planner,
  Estimator,
  OSC,
  Fr3TeleopFollow,
  Fr3JointMove,
  LeRobotPlanner,
};

/**
 * @brief Structure to define a module chain configuration
 */
struct ModuleChainConfig {
  std::vector<ModuleType> modules;
};

// Phase 5: ModuleConfigKey REMOVED - no longer needed with smart defaults

/**
 * @class ModuleManager
 * @brief class for managing all modules, instantiating all modules and linking the data stream
 */

class ModuleManager {
public:
  ModuleManager(const YAML::Node &yamlFile,
                RobotManager &robotManager,
                SceneManager &sceneManager,
                FsmManager &fsmManager,
                data::RuntimeData &runtimeData,
                bool enableLcmLogging = false,
                double dt = 0.001);
  ~ModuleManager() = default;

  void updateAllModules();

  void resetAllModules();


  void setDt(double dt);

  const std::vector<std::unique_ptr<CModuleBase>> &getModules() const {
    return modules_;
  }

private:
  YAML::Node _yamlNode;
  data::RuntimeData &runtimeData_;
  RobotManager &robotManager_;
  SceneManager &sceneManager_;
  FsmManager &fsmManager_;
  std::vector<std::unique_ptr<CModuleBase>> modules_;
  std::shared_ptr<utils::PinKine> sharedPinKine_;
  void initModules();
  void createModules();
  bool enableLcmLogging_{false};
  double dt_{0.001};

  // Phase 5: moduleConfigTable_ and createModuleVTable() REMOVED
  std::unique_ptr<CModuleBase> createModuleByType(ModuleType type);
  void createModuleChain(const ModuleChainConfig &config);

  ModuleChainConfig getSmartDefaultChain(SceneType sceneType);
  std::optional<ModuleChainConfig> loadRobotConfigOverride(const std::string& configPath,
                                                            const std::string& sceneName);
  std::optional<ModuleType> parseModuleType(const std::string& typeName);
  std::string toLower(const std::string& str) const;
  bool contains(const std::string& str, const std::string& substr) const;

  std::shared_ptr<lcm::LCM> lcmPtr{nullptr};
  lcmMotion::lcm_record lcmStruct;
  std::unique_ptr<utils::LcmLogger> logger{nullptr};
  void initLcm();
  void writeLogData(data::RuntimeData &data);
  void publishLcm();
};

} // namespace rynn
