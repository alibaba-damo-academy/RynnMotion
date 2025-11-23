#include "module_manager.hpp"

#include <algorithm>
#include <optional>

#include "discovery.hpp"
#include "robot_manager.hpp"

namespace rynn {

ModuleManager::ModuleManager(const YAML::Node &yamlFile,
                             RobotManager &robotManager,
                             SceneManager &sceneManager,
                             FsmManager &fsmManager,
                             data::RuntimeData &runtimeData,
                             bool enableLcmLogging,
                             double dt) :
    _yamlNode(yamlFile),
    runtimeData_(runtimeData),
    robotManager_(robotManager),
    sceneManager_(sceneManager),
    fsmManager_(fsmManager),
    enableLcmLogging_(enableLcmLogging),
    dt_(dt) {
  std::string pinoMjcfPath = robotManager.getPinoMJCF();
  if (!pinoMjcfPath.empty()) {
    auto siteNames = robotManager.getSiteNames();
    sharedPinKine_ = std::make_shared<utils::PinKine>(pinoMjcfPath, siteNames);
  }

  initLcm();
  initModules();
}

void ModuleManager::initLcm() {
  if (!enableLcmLogging_) {
    return;
  }

  // ttl=0: localhost only (works without network connection)
  // ttl=1: local subnet (requires active network interface)
  lcmPtr = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7667?ttl=0");
  if (!lcmPtr->good()) {
    std::cerr << "Error: LCM initialization failed with URL udpm://239.255.76.67:7667?ttl=0" << std::endl;
    return;
  }
  logger = std::make_unique<utils::LcmLogger>(lcmStruct);
}

void ModuleManager::initModules() {
  modules_.clear();

  createModules();

  for (auto &module : modules_) {
    module->setDt(dt_);
  }
}

void ModuleManager::createModules() {
  auto &discovery = RobotDiscovery::getInstance();
  const auto robotType = robotManager_.getRobotType();
  const auto sceneNumber = sceneManager_.getSceneNumber();

  try {
    auto robotInfo = discovery.getRobotInfo(static_cast<int>(robotType));
    auto scene = SceneDiscovery::getScene(robotInfo.scenes, std::to_string(sceneNumber));

    ModuleChainConfig chainConfig;

    if (robotInfo.configPath.has_value()) {
      auto override = loadRobotConfigOverride(robotInfo.configPath.value(), scene.name);
      if (override.has_value()) {
        chainConfig = override.value();
        std::cout << "✓ Using custom module chain from " << robotInfo.configPath.value()
                  << " for scene '" << scene.name << "'" << std::endl;
        createModuleChain(chainConfig);
        return;
      }
    }

    SceneType sceneType = sceneManager_.getSceneType();
    chainConfig = getSmartDefaultChain(sceneType);
    std::cout << "✓ Using smart default module chain for scene " << sceneNumber << std::endl;
    createModuleChain(chainConfig);

  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to create modules for robot " + std::to_string(static_cast<int>(robotType))
                             + ", scene " + std::to_string(sceneNumber) + ": " + e.what());
  }
}

void ModuleManager::createModuleChain(const ModuleChainConfig &config) {
  modules_.clear();

  // Create shared_ptr to RuntimeData (no-op deleter since ModuleManager owns it by reference)
  auto runtimeDataPtr = std::shared_ptr<data::RuntimeData>(
      &runtimeData_,
      [](void *) {} // No-op deleter - ModuleManager owns the actual object
  );

  for (size_t i = 0; i < config.modules.size(); ++i) {
    auto module = createModuleByType(config.modules[i]);

    // All modules share the same RuntimeData
    module->setRuntimeData(runtimeDataPtr);

    // Initialize module (allocates RuntimeData structures and internal data)
    module->initModule();

    modules_.emplace_back(std::move(module));
  }
}

std::unique_ptr<CModuleBase> ModuleManager::createModuleByType(ModuleType type) {
  std::unique_ptr<CModuleBase> module;

  switch (type) {
  case ModuleType::JointMove:
    module = std::make_unique<CJointMove>(_yamlNode);
    break;
  case ModuleType::Planner:
    module = std::make_unique<CPlanner>(_yamlNode);
    break;
  case ModuleType::Estimator:
    module = std::make_unique<CEstimator>(_yamlNode);
    break;
  case ModuleType::OSC:
    module = std::make_unique<OSC>(_yamlNode);
    break;
  case ModuleType::Fr3TeleopFollow:
    module = std::make_unique<Fr3TeleopFollow>(_yamlNode);
    break;
  case ModuleType::Fr3JointMove:
    module = std::make_unique<Fr3JointMove>(_yamlNode);
    break;
  case ModuleType::LeRobotPlanner:
    // Note: LeRobotPlanner is defined in robot satellites, not core
    throw std::runtime_error("LeRobotPlanner should be loaded from robot-specific module");
    break;
  default:
    throw std::runtime_error("Unknown module type");
  }

  module->setManagers(robotManager_, sceneManager_, fsmManager_);
  if (sharedPinKine_) {
    module->setSharedPinKine(sharedPinKine_);
  }

  return module;
}

void ModuleManager::updateAllModules() {
  // All modules share the same RuntimeData, so just update them sequentially
  for (auto &module : modules_) {
    module->update();
  }

  if (!modules_.empty()) {
    // Update timing data in shared RuntimeData
    runtimeData_.wallTime = modules_.back()->getCurrentTime();
    runtimeData_.duration = modules_.back()->getDurationSinceLastCall();

    if (enableLcmLogging_) {
      writeLogData(runtimeData_);
      publishLcm();
    }
  }
}

void ModuleManager::resetAllModules() {
  for (size_t i = 0; i < modules_.size(); ++i) {
    modules_[i]->resetModule();
  }
}

void ModuleManager::writeLogData(data::RuntimeData &data) {
  if (lcmPtr && lcmPtr->good() && logger) {
    std::shared_ptr<data::RuntimeData> dataPtr(reinterpret_cast<data::RuntimeData *>(&data), [](void *) {});
    logger->writeLogData(dataPtr);
  }
}

void ModuleManager::publishLcm() {
  if (lcmPtr && lcmPtr->good()) {
    lcmPtr->publish("lcm_motion", &lcmStruct);
  }
}

void ModuleManager::setDt(double newDt) {
  dt_ = newDt;
  for (auto &module : modules_) {
    module->setDt(dt_);
  }
}

ModuleChainConfig ModuleManager::getSmartDefaultChain(SceneType sceneType) {
  switch (sceneType) {
  case SceneType::kUI:
  case SceneType::kTracking:
    return {{ModuleType::Estimator, ModuleType::Planner, ModuleType::OSC}};

  case SceneType::kPickPlace:
    return {{ModuleType::Estimator, ModuleType::Planner, ModuleType::OSC}};

  case SceneType::kDefault:
  case SceneType::kKeyframeCycle:
  default:
    return {{ModuleType::JointMove}};
  }
}

std::optional<ModuleChainConfig> ModuleManager::loadRobotConfigOverride(
    const std::string &configPath,
    const std::string &sceneName) {
  try {
    YAML::Node config = YAML::LoadFile(configPath);

    if (!config["module_chains"]) {
      return std::nullopt;
    }

    YAML::Node chains = config["module_chains"];
    if (!chains[sceneName]) {
      return std::nullopt;
    }

    YAML::Node moduleList = chains[sceneName];
    if (!moduleList.IsSequence()) {
      std::cerr << "Warning: module_chains[" << sceneName << "] is not a list" << std::endl;
      return std::nullopt;
    }

    ModuleChainConfig chainConfig;
    for (const auto &moduleNode : moduleList) {
      std::string moduleName = moduleNode.as<std::string>();
      auto moduleType = parseModuleType(moduleName);

      if (moduleType) {
        chainConfig.modules.push_back(*moduleType);
      } else {
        std::cerr << "Warning: Unknown module type '" << moduleName
                  << "' in " << configPath << std::endl;
      }
    }

    if (chainConfig.modules.empty()) {
      return std::nullopt;
    }

    return chainConfig;

  } catch (const YAML::Exception &e) {
    std::cerr << "Warning: Failed to load robot config " << configPath
              << ": " << e.what() << std::endl;
    return std::nullopt;
  }
}

std::optional<ModuleType> ModuleManager::parseModuleType(const std::string &typeName) {
  std::string lower = toLower(typeName);

  if (lower == "jointmove") return ModuleType::JointMove;
  if (lower == "planner") return ModuleType::Planner;
  if (lower == "estimator") return ModuleType::Estimator;
  if (lower == "osc") return ModuleType::OSC;
  if (lower == "fr3teleopfollow") return ModuleType::Fr3TeleopFollow;
  if (lower == "fr3jointmove") return ModuleType::Fr3JointMove;
  if (lower == "lerobotplanner") return ModuleType::LeRobotPlanner;

  return std::nullopt;
}

std::string ModuleManager::toLower(const std::string &str) const {
  std::string result = str;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return result;
}

bool ModuleManager::contains(const std::string &str, const std::string &substr) const {
  return str.find(substr) != std::string::npos;
}

} // namespace rynn
