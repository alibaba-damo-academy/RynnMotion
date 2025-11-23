#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "debug_config.hpp"
#include "robot_types.hpp"

namespace rynn {

namespace fs = std::filesystem;

namespace {
const std::map<std::string, std::vector<std::string>> ROBOT_ALIASES = {
    {"fr3", {"FR3", "franka", "franka-research3", "franka_research_3", "panda"}},
    {"ur5e", {"UR5E", "ur5-e", "ur5_e", "universal-robots-ur5e"}},
    {"piper", {"PIPER", "Piper"}},
    {"rm75", {"RM75", "realman75", "realman-rm75"}},
    {"so101", {"SO101", "so-101", "so_101", "soarm101", "so-arm-101"}},
    {"rizon4s", {"rizon", "Rizon", "RIZON", "RIZON4S", "Rizon4S", "rizon-4s", "rizon_4s"}},
    {"eco65", {"ECO65", "eco-65"}},
    {"dm6dof", {"DM6DOF", "dm-6dof", "dm_6dof"}},
    {"dual_fr3", {"dual-fr3", "dualfr3", "dual_franka", "dual-franka"}},
    {"dual_ur5e", {"dual-ur5e", "dualur5e"}},
    {"dual_piper", {"dual-piper", "dualpiper"}},
    {"dual_rm75", {"dual-rm75", "dualrm75"}},
    {"dual_so101", {"dual-so101", "dualso101"}},
    {"dual_eco65", {"dual-eco65", "dualeco65"}},
    {"elephant", {"ELEPHANT", "Elephant"}},
    {"robotiq", {"ROBOTIQ", "Robotiq"}},
    {"umi", {"UMI", "Umi"}},
    {"lerobot", {"LeRobot", "LEROBOT"}},
    {"lekiwi", {"LeKiwi", "LEKIWI"}},
    {"xlerobot", {"XLeRobot", "XLEROBOT", "xle-robot"}},
    {"diffMobile", {"diffmobile", "diff-mobile", "diff_mobile"}},
    {"diffCar", {"diffcar", "diff-car", "diff_car"}},
    {"diffMobile_Franka", {"mobile_fr3", "diff-mobile-franka"}},
    {"oneLink", {"onelink", "one-link", "one_link"}},
    {"twoLink", {"twolink", "two-link", "two_link"}},
};

const std::map<std::string, std::vector<std::string>> SCENE_ALIASES = {
    {"joint", {"JOINT", "Joint", "default", "DEFAULT", "Default", "jointmove", "jointmotion"}},
    {"keyframe", {"KEYFRAME", "Keyframe", "keyframecycle", "wobble", "cycle"}},
    {"ui", {"UI", "tracking", "TRACKING", "Tracking"}},
    {"predefined", {"PREDEFINED", "Predefined", "workspace"}},
    {"pickplace", {"PICKPLACE", "PickPlace", "pick", "pick-place"}},
};

const std::map<std::string, std::map<std::string, std::vector<int>>> LEGACY_SCENE_MAPPINGS = {
    {"fr3", {{"scene.xml", {1, 2, 3, 4}}, {"scene_pickplace.xml", {5}}}},
    {"ur5e", {{"scene.xml", {1, 2, 3, 4}}, {"scene_pickplace.xml", {5}}}},
    {"piper", {{"scene.xml", {1, 2, 3, 4}}, {"scene_pickplace.xml", {5}}}},
    {"rm75", {{"scene.xml", {1, 2, 3, 4}}, {"scene_pickplace.xml", {5}}}},
    {"so101", {{"scene.xml", {1, 2, 3, 4}}, {"scene_pickplace.xml", {5}}}},
    {"rizon4s", {{"scene.xml", {1, 2, 3, 4, 5}}}},
    {"onelink", {{"scene.xml", {1, 2}}}},
    {"twolink", {{"scene.xml", {1, 2}}}},
    {"dual_fr3", {{"dual_fr3_scene.xml", {1, 2, 3, 4}}, {"dual_fr3_pickplace.xml", {5}}}},
    {"dual_ur5e", {{"dual_ur5e_scene.xml", {1, 2, 3, 4}}, {"dual_ur5e_pickplace.xml", {5}}}},
    {"dual_piper", {{"dual_piper_scene.xml", {1, 2, 3, 4}}, {"dual_piper_pickplace.xml", {5}}}},
    {"dual_rm75", {{"dual_rm75_scene.xml", {1, 2, 3, 4}}, {"dual_rm75_pickplace.xml", {5}}}},
    {"dual_so101", {{"dual_so101_scene.xml", {1, 2, 3, 4}}, {"dual_so101_pickplace.xml", {5}}}},
};
}

// Forward declaration
struct DiscoveredSceneInfo;

/**
 * @brief Information about a discovered robot
 */
struct DiscoveredRobotInfo {
  int number;
  std::string canonicalName;
  std::vector<std::string> aliases;
  std::string basePath;
  std::string pinocchioPath;
  std::string robotMjcfPath;
  std::vector<DiscoveredSceneInfo> scenes;
  std::optional<std::string> configPath;

  RobotType getRobotType() const {
    return static_cast<RobotType>(number);
  }
};

/**
 * @brief Information about a discovered scene
 *
 * Supports backward compatibility where multiple scene numbers map to the same XML file
 */
struct DiscoveredSceneInfo {
  std::vector<int> numbers;
  std::string name;
  std::string filename;
  std::string fullPath;
  std::string description;

  int getPrimaryNumber() const {
    return numbers.empty() ? -1 : numbers[0];
  }

  bool hasNumber(int num) const {
    return std::find(numbers.begin(), numbers.end(), num) != numbers.end();
  }
};

/**
 * @class RobotDiscovery
 * @brief Auto-discovers robots from models/ directory structure
 *
 * Singleton that scans models/ to find all available robots and scenes.
 * Provides case-insensitive lookup with alias support.
 */
class RobotDiscovery {
public:
  static RobotDiscovery &getInstance();

  void scanRobots(const std::string &modelsDir);

  DiscoveredRobotInfo getRobotInfo(const std::string &nameOrNumber) const;
  DiscoveredRobotInfo getRobotInfo(int number) const;

  std::vector<DiscoveredRobotInfo> getAllRobots() const;

  bool robotExists(const std::string &nameOrNumber) const;
  bool robotExists(int number) const;

  std::vector<std::string> getAllRobotNames() const;

private:
  RobotDiscovery() = default;

  std::pair<int, std::string> parseRobotDirName(const std::string &dirname) const;

  std::string findPinocchioXml(const std::filesystem::path &robotDir) const;
  std::string findRobotMjcf(const std::filesystem::path &robotDir) const;
  std::optional<std::string> findRobotConfig(const std::filesystem::path &robotDir) const;

  std::vector<DiscoveredSceneInfo> scanScenes(const std::filesystem::path &robotDir,
                                              const std::string &robotName) const;

  std::vector<std::string> getCommonAliases(const std::string &canonicalName) const;
  void registerRobot(const DiscoveredRobotInfo &info);
  std::string toLower(const std::string &str) const;

  std::map<int, DiscoveredRobotInfo> robotsByNumber_;
  std::map<std::string, int> nameToNumber_;  // lowercase canonical name -> number
  std::map<std::string, int> aliasToNumber_; // lowercase alias -> number
};

/**
 * @class SceneDiscovery
 * @brief Auto-discovers scenes for a robot
 */
class SceneDiscovery {
public:
  static std::vector<DiscoveredSceneInfo> scanScenes(const std::filesystem::path &robotPath,
                                                     const std::string &robotName = "");

  static DiscoveredSceneInfo getScene(const std::vector<DiscoveredSceneInfo> &scenes,
                                      const std::string &numberOrName);

  static std::string parseSceneName(const std::string &filename);

private:
  static std::string toLower(const std::string &str);
};

inline RobotDiscovery &RobotDiscovery::getInstance() {
  static RobotDiscovery instance;
  return instance;
}

inline void RobotDiscovery::scanRobots(const std::string &modelsDir) {
  if (!robotsByNumber_.empty()) {
    return;
  }

  robotsByNumber_.clear();
  nameToNumber_.clear();
  aliasToNumber_.clear();

  if (!fs::exists(modelsDir) || !fs::is_directory(modelsDir)) {
    throw std::runtime_error("Models directory not found: " + modelsDir);
  }

  for (const auto &categoryEntry : fs::directory_iterator(modelsDir)) {
    if (!categoryEntry.is_directory()) continue;

    std::string categoryName = categoryEntry.path().filename().string();
    if (categoryName[0] == '.') continue;

    for (const auto &robotEntry : fs::directory_iterator(categoryEntry)) {
      if (!robotEntry.is_directory()) continue;

      std::string robotDirName = robotEntry.path().filename().string();
      if (robotDirName[0] == '.') continue;

      const std::vector<std::string> skipDirs = {
          "meshes", "assets", "human", "urdf", "stl", "textures", "scenes"};
      if (std::find(skipDirs.begin(), skipDirs.end(), robotDirName) != skipDirs.end()) {
        continue;
      }

      try {
        auto [number, name] = parseRobotDirName(robotDirName);

        DiscoveredRobotInfo info;
        info.number = number;
        info.canonicalName = name;
        info.basePath = robotEntry.path().string();
        info.pinocchioPath = findPinocchioXml(robotEntry.path());
        info.robotMjcfPath = findRobotMjcf(robotEntry.path());
        info.configPath = findRobotConfig(robotEntry.path());
        info.scenes = scanScenes(robotEntry.path(), name);
        info.aliases = getCommonAliases(name);
        registerRobot(info);

        DEBUG_LOG("Discovered robot: " << number << "." << name
                                       << " (" << info.scenes.size() << " scenes)");

      } catch (const std::exception &e) {
        continue;
      }
    }
  }

  DEBUG_LOG("Total robots discovered: " << robotsByNumber_.size());
}

inline std::pair<int, std::string> RobotDiscovery::parseRobotDirName(const std::string &dirname) const {
  std::regex pattern(R"(^(\d+)\.(.+)$)");
  std::smatch match;

  if (std::regex_match(dirname, match, pattern)) {
    int number = std::stoi(match[1].str());
    std::string name = match[2].str();
    return {number, name};
  }

  throw std::runtime_error("Robot directory must follow 'NUMBER.name' format: " + dirname);
}

inline std::string RobotDiscovery::findPinocchioXml(const fs::path &robotDir) const {
  std::vector<std::string> searchPaths = {
      robotDir.string(),
      (robotDir / "mjcf").string(),
  };

  for (const auto &searchPath : searchPaths) {
    if (!fs::exists(searchPath)) continue;

    for (const auto &entry : fs::directory_iterator(searchPath)) {
      if (!entry.is_regular_file()) continue;

      std::string filename = entry.path().filename().string();
      if (filename.find("_pinocchio.xml") != std::string::npos) {
        return entry.path().string();
      }
    }
  }

  // Not required for all robots (e.g., low DOF test robots)
  return "";
}

inline std::string RobotDiscovery::findRobotMjcf(const fs::path &robotDir) const {
  std::vector<std::string> searchPaths = {
      robotDir.string(),
      (robotDir / "mjcf").string(),
  };

  for (const auto &searchPath : searchPaths) {
    if (!fs::exists(searchPath)) continue;

    for (const auto &entry : fs::directory_iterator(searchPath)) {
      if (!entry.is_regular_file()) continue;

      std::string filename = entry.path().filename().string();
      if (filename.find("_robot.xml") != std::string::npos || filename == "robot.xml") {
        return entry.path().string();
      }
    }
  }

  return "";
}

inline std::optional<std::string> RobotDiscovery::findRobotConfig(const fs::path &robotDir) const {
  fs::path configPath = robotDir / "robot_config.yaml";
  if (fs::exists(configPath)) {
    return configPath.string();
  }
  return std::nullopt;
}

inline std::vector<DiscoveredSceneInfo> RobotDiscovery::scanScenes(const fs::path &robotDir,
                                                            const std::string &robotName) const {
  return SceneDiscovery::scanScenes(robotDir, robotName);
}

inline std::vector<std::string> RobotDiscovery::getCommonAliases(const std::string &canonicalName) const {
  auto it = ROBOT_ALIASES.find(canonicalName);
  if (it != ROBOT_ALIASES.end()) {
    return it->second;
  }
  return {};
}

inline void RobotDiscovery::registerRobot(const DiscoveredRobotInfo &info) {
  robotsByNumber_[info.number] = info;

  std::string lowerName = toLower(info.canonicalName);
  nameToNumber_[lowerName] = info.number;

  for (const auto &alias : info.aliases) {
    std::string lowerAlias = toLower(alias);
    aliasToNumber_[lowerAlias] = info.number;
  }
}

inline DiscoveredRobotInfo RobotDiscovery::getRobotInfo(const std::string &nameOrNumber) const {
  try {
    int number = std::stoi(nameOrNumber);
    return getRobotInfo(number);
  } catch (...) {
  }

  std::string lower = toLower(nameOrNumber);

  auto nameIt = nameToNumber_.find(lower);
  if (nameIt != nameToNumber_.end()) {
    return robotsByNumber_.at(nameIt->second);
  }

  auto aliasIt = aliasToNumber_.find(lower);
  if (aliasIt != aliasToNumber_.end()) {
    return robotsByNumber_.at(aliasIt->second);
  }

  throw std::runtime_error("Robot not found: " + nameOrNumber);
}

inline DiscoveredRobotInfo RobotDiscovery::getRobotInfo(int number) const {
  auto it = robotsByNumber_.find(number);
  if (it != robotsByNumber_.end()) {
    return it->second;
  }
  throw std::runtime_error("Robot number not found: " + std::to_string(number));
}

inline std::vector<DiscoveredRobotInfo> RobotDiscovery::getAllRobots() const {
  std::vector<DiscoveredRobotInfo> robots;
  robots.reserve(robotsByNumber_.size());
  for (const auto &[number, info] : robotsByNumber_) {
    robots.push_back(info);
  }
  std::sort(robots.begin(), robots.end(),
            [](const DiscoveredRobotInfo &a, const DiscoveredRobotInfo &b) {
              return a.number < b.number;
            });
  return robots;
}

inline bool RobotDiscovery::robotExists(const std::string &nameOrNumber) const {
  try {
    getRobotInfo(nameOrNumber);
    return true;
  } catch (...) {
    return false;
  }
}

inline bool RobotDiscovery::robotExists(int number) const {
  return robotsByNumber_.find(number) != robotsByNumber_.end();
}

inline std::vector<std::string> RobotDiscovery::getAllRobotNames() const {
  std::vector<std::string> names;
  names.reserve(robotsByNumber_.size());
  for (const auto &[number, info] : robotsByNumber_) {
    names.push_back(info.canonicalName);
  }
  std::sort(names.begin(), names.end());
  return names;
}

inline std::string RobotDiscovery::toLower(const std::string &str) const {
  std::string result = str;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return result;
}

inline std::vector<DiscoveredSceneInfo> SceneDiscovery::scanScenes(const fs::path &robotPath,
                                                            const std::string &robotName) {
  std::vector<DiscoveredSceneInfo> scenes;

  fs::path sceneDir = robotPath / "scene";
  if (!fs::exists(sceneDir)) {
    sceneDir = robotPath;
  }

  if (!fs::exists(sceneDir) || !fs::is_directory(sceneDir)) {
    return scenes;
  }

  std::vector<fs::path> sceneFiles;
  for (const auto &entry : fs::directory_iterator(sceneDir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".xml") {
      std::string filename = entry.path().filename().string();
      if (filename.find("_robot.xml") != std::string::npos || filename.find("_pinocchio.xml") != std::string::npos) {
        continue;
      }
      sceneFiles.push_back(entry.path());
    }
  }

  std::sort(sceneFiles.begin(), sceneFiles.end());

  std::string lowerRobotName = toLower(robotName);
  auto legacyIt = LEGACY_SCENE_MAPPINGS.find(lowerRobotName);
  bool hasLegacyMapping = (legacyIt != LEGACY_SCENE_MAPPINGS.end());

  for (size_t i = 0; i < sceneFiles.size(); ++i) {
    DiscoveredSceneInfo info;
    info.fullPath = sceneFiles[i].string();
    info.filename = sceneFiles[i].filename().string();
    info.name = parseSceneName(info.filename);
    info.description = "";

    if (hasLegacyMapping) {
      auto &fileMapping = legacyIt->second;
      auto fileIt = fileMapping.find(info.filename);
      if (fileIt != fileMapping.end()) {
        info.numbers = fileIt->second;
      } else {
        // High number to avoid conflicts with legacy mappings
        info.numbers = {static_cast<int>(i + 100)};
      }
    } else {
      info.numbers = {static_cast<int>(i + 1)};
    }

    scenes.push_back(info);
  }

  return scenes;
}

inline DiscoveredSceneInfo SceneDiscovery::getScene(const std::vector<DiscoveredSceneInfo> &scenes,
                                             const std::string &numberOrName) {
  if (scenes.empty()) {
    throw std::runtime_error("No scenes available");
  }

  try {
    int sceneNumber = std::stoi(numberOrName);
    for (const auto &scene : scenes) {
      if (scene.hasNumber(sceneNumber)) {
        return scene;
      }
    }
    throw std::runtime_error("Scene number out of range: " + numberOrName);
  } catch (const std::invalid_argument &) {
  }

  std::string lower = toLower(numberOrName);

  for (const auto &scene : scenes) {
    if (toLower(scene.name) == lower) {
      return scene;
    }
  }

  std::string canonicalName = lower;
  for (const auto &[canonical, aliases] : SCENE_ALIASES) {
    if (toLower(canonical) == lower) {
      canonicalName = canonical;
      break;
    }
    for (const auto &alias : aliases) {
      if (toLower(alias) == lower) {
        canonicalName = canonical;
        break;
      }
    }
    if (canonicalName != lower) break;
  }

  if (canonicalName != lower) {
    for (const auto &scene : scenes) {
      if (toLower(scene.name) == canonicalName) {
        return scene;
      }
    }
  }

  throw std::runtime_error("Scene not found: " + numberOrName);
}

inline std::string SceneDiscovery::parseSceneName(const std::string &filename) {
  std::string base = filename;
  if (base.size() > 4 && base.substr(base.size() - 4) == ".xml") {
    base = base.substr(0, base.size() - 4);
  }

  if (base == "scene") {
    return "default";
  }

  if (base.find("scene_") == 0) {
    return base.substr(6);
  }

  std::regex pattern(R"(^scene\d+_(.+)$)");
  std::smatch match;
  if (std::regex_match(base, match, pattern)) {
    return match[1].str();
  }

  return base;
}

inline std::string SceneDiscovery::toLower(const std::string &str) {
  std::string result = str;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return result;
}

} // namespace rynn
