#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot_types.hpp"

namespace rynn {

/**
 * @enum SceneType
 * @brief Semantic scene types for module chain selection
 *
 * Maps scene numbers to their functional purpose.
 * Scene 1-4 all load scene.xml, scene 5 loads scene_pickplace.xml
 */
enum class SceneType {
  kDefault = 1,       // Joint range motion
  kKeyframeCycle = 2, // Keyframe cycling
  kUI = 3,            // Keyboard UI / manual control
  kTracking = 4,      // End-effector tracking
  kPickPlace = 5      // Pick and place tasks
};

/**
 * @class SceneManager
 * @brief Manages simulation scene configurations including robot positions and objects
 */
class SceneManager {
public:
  SceneManager(int robotNumber, int sceneNumber = 1);
  ~SceneManager() = default;

  /**
   * @brief Get the scene number
   * @return The scene number
   */
  int getSceneNumber() const {
    return _sceneNumber;
  }

  /**
   * @brief Get the scene type (semantic alias for scene number)
   * @return The scene type enum
   */
  SceneType getSceneType() const {
    return sceneNumberToType(_sceneNumber);
  }

  /**
   * @brief Get the canonical scene name
   * @return The scene name (e.g., "joint", "ui", "pickplace")
   */
  std::string getSceneName() const;

  /**
   * @brief Convert scene number to SceneType enum
   * @param number Scene number (1-5)
   * @return Corresponding SceneType enum value
   */
  static SceneType sceneNumberToType(int number);

  /**
   * @brief Convert scene name to SceneType enum
   * @param name Scene name (e.g., "joint", "ui", "pickplace")
   * @return Corresponding SceneType enum value
   */
  static SceneType sceneNameToType(const std::string &name);

  /**
   * @brief Convert scene number to canonical scene name
   * @param number Scene number (1-5)
   * @return Canonical scene name
   */
  static std::string sceneNumberToName(int number);

  /**
   * @brief Get the MJCF file path
   * @return The MJCF file path for the current robot type and scene
   */
  std::string getSceneMJCF() const {
    return _sceneMjcf;
  }

  /**
   * @brief Defines a scene object for simulation
   */
  struct SceneObject {
    std::string name;
    std::string type;
    Eigen::Vector3d position{0, 0, 0};
    Eigen::Quaterniond orientation{1, 0, 0, 0};
    bool movable = false;
    Eigen::Vector3d scale{1, 1, 1};
  };

  /**
   * @brief Get all scene objects
   * @return Vector of scene objects
   */
  const std::vector<SceneObject> &getSceneObjects() const {
    return sceneObjects;
  }

  /**
   * @brief Get a specific scene object by name
   * @param name The name of the object to find
   * @return Pointer to the object if found, nullptr otherwise
   */
  SceneObject *getSceneObjectByName(const std::string &name);

  /**
   * @brief Update the transform of a scene object
   * @param objectName Name of the object to update
   * @param position New position
   * @param orientation New orientation
   */
  void updateObjectTransform(const std::string &objectName,
                             const Eigen::Vector3d &position,
                             const Eigen::Quaterniond &orientation);

  /**
   * @brief Get the currently selected object name
   * @return The selected object name
   */
  std::string getSelectedObjectName() const {
    return uiObjectName;
  }

  /**
   * @brief Set the currently selected object
   * @param name The name of the object to select
   */
  void setSelectedObjectName(const std::string &name) {
    uiObjectName = name;
  }

  /**
   * @brief Get the origin offset for the current robot
   * @return The origin offset vector
   */
  Eigen::Vector3d getOriginOffset() const;

private:
  void loadScenePath();
  static std::string toLower(const std::string &str);

  int _robotNumber;
  int _sceneNumber;
  rynn::RobotType _robotType;  // Alias for _robotNumber (kept for API compatibility)
  std::string _sceneMjcf;
  std::string _sceneName;      // Canonical scene name (e.g., "joint", "ui", "pickplace")

  Eigen::Vector3d _robotOriginPos{0.0, 0.0, 0.0};
  std::vector<SceneObject> sceneObjects;
  std::string uiObjectName;
};

} // namespace rynn
