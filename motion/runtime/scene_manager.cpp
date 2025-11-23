#include "scene_manager.hpp"

#include <algorithm>
#include <iostream>

#include "discovery.hpp"

namespace rynn {

SceneManager::SceneManager(int robotNumber, int sceneNumber) :
    _robotNumber(robotNumber),
    _sceneNumber(sceneNumber),
    _robotType(static_cast<RobotType>(robotNumber)) {
  loadScenePath();
}

void SceneManager::loadScenePath() {
  // Use auto-discovery system with backward-compatible scene numbering
  auto &discovery = RobotDiscovery::getInstance();

  static bool initialized = false;
  if (!initialized) {
    discovery.scanRobots(MODEL_DIR);
    initialized = true;
  }

  try {
    auto robotInfo = discovery.getRobotInfo(_robotNumber);
    auto scene = SceneDiscovery::getScene(robotInfo.scenes, std::to_string(_sceneNumber));
    _sceneMjcf = scene.fullPath;
    _sceneName = sceneNumberToName(_sceneNumber);

  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to load scene for robot " + std::to_string(_robotNumber) + ", scene " + std::to_string(_sceneNumber) + ": " + e.what());
  }
}

SceneManager::SceneObject *SceneManager::getSceneObjectByName(const std::string &name) {
  for (auto &obj : sceneObjects) {
    if (obj.name == name) {
      return &obj;
    }
  }
  return nullptr;
}

void SceneManager::updateObjectTransform(const std::string &objectName,
                                         const Eigen::Vector3d &position,
                                         const Eigen::Quaterniond &orientation) {
  SceneObject *obj = getSceneObjectByName(objectName);
  if (obj && obj->movable) {
    obj->position = position;
    obj->orientation = orientation;
  }
}

Eigen::Vector3d SceneManager::getOriginOffset() const {
  return _robotOriginPos;
}

std::string SceneManager::getSceneName() const {
  return _sceneName;
}

SceneType SceneManager::sceneNumberToType(int number) {
  switch (number) {
  case 1:
    return SceneType::kDefault;
  case 2:
    return SceneType::kKeyframeCycle;
  case 3:
    return SceneType::kUI;
  case 4:
    return SceneType::kTracking;
  case 5:
    return SceneType::kPickPlace;
  default:
    return SceneType::kDefault;
  }
}

std::string SceneManager::sceneNumberToName(int number) {
  switch (number) {
  case 1:
    return "joint";
  case 2:
    return "keyframe";
  case 3:
    return "ui";
  case 4:
    return "predefined";
  case 5:
    return "pickplace";
  default:
    return "joint";
  }
}

SceneType SceneManager::sceneNameToType(const std::string &name) {
  std::string lower = toLower(name);

  if (lower == "joint" || lower == "default" || lower == "jointmove" || lower == "jointmotion") {
    return SceneType::kDefault;
  }
  if (lower == "keyframe" || lower == "keyframecycle" || lower == "wobble" || lower == "cycle") {
    return SceneType::kKeyframeCycle;
  }
  if (lower == "ui" || lower == "tracking") {
    return SceneType::kUI;
  }
  if (lower == "predefined" || lower == "workspace") {
    return SceneType::kTracking;
  }
  if (lower == "pickplace" || lower == "pick" || lower == "pick-place") {
    return SceneType::kPickPlace;
  }

  return SceneType::kDefault;
}

std::string SceneManager::toLower(const std::string &str) {
  std::string result = str;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return result;
}

} // namespace rynn
