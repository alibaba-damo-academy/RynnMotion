#pragma once
#include "mj_interface.hpp"
#include "scene_manager.hpp"
#ifdef __APPLE__
#include "mujoco.h"
#else
#include "mujoco/mujoco.h"
#endif

namespace mujoco {

class MujocoInterface;

/**
 * @brief Handles scene objects, cameras, and robot initialization in MuJoCo
 *
 * This class manages scene-specific logic including object tracking, camera updates,
 * and robot initialization. It centralizes scene management that was previously
 * scattered across different parts of the interface.
 */
class MujocoScene {
public:
  /**
   * @brief Constructor that takes a reference to MujocoInterface
   * @param mj Reference to the parent MujocoInterface object
   */
  explicit MujocoScene(MujocoInterface &mj);

  /**
   * @brief Initialize the entire scene including robot, objects, and cameras
   *
   * This method initializes the robot to its home position using stored keyframes,
   * detects and initializes scene objects, and sets up cameras. It should be called
   * once during setup.
   */
  void initScene();

  /**
   * @brief Update all scene elements (objects, cameras, robot-specific logic)
   *
   * This method should be called during getFeedbacks() to update object positions,
   * camera states, and handle any robot/scene-specific logic.
   */
  void update();

  /**
   * @brief Render tracking visuals (cube + RGB frame) to mjvScene
   *
   * Called after mjv_updateScene() to re-add custom geometry that gets cleared.
   */
  void renderTrackingVisuals();

private:
  /** @brief Reference to the parent MujocoInterface */
  MujocoInterface &mj_;

  /** @brief Robot type from the robot manager */
  rynn::RobotType robotType_;

  /** @brief Scene number from the scene manager */
  int sceneNumber_;

  /** @brief Scene type (cached for efficient access) */
  rynn::SceneType sceneType_;

  /** @brief Robot base origin position (default: 0,0,0) */
  Eigen::Vector3d baseOrigin_{0.0, 0.0, 0.0};

  /** @brief Index of first tracking visual geom in mjvScene */
  int trackGeomIdx_{-1};

  /** @brief Number of tracking visual geoms (cube + 3 cylinders) */
  int trackGeomCount_{0};

  /** @brief Number of end effectors */
  int numEE_{0};

  /** @brief Dual-arm flag */
  bool isDualArm_{false};

  /** @brief Robot base offset from world origin (for dual-arm) */
  Eigen::Vector3d robotBaseOffset_{0.0, 0.0, 0.0};

  /**
   * @brief Update object positions and states from MuJoCo data
   */
  void updateObjects();

  /**
   * @brief Update camera positions and orientations from MuJoCo data
   */
  void updateCameras();

  /**
   * @brief Initialize scene objects and add them to the data stream
   *
   * This method detects objects in the scene and initializes the corresponding
   * data structures in the data stream for tracking their states.
   */
  void initObjects();

  /**
   * @brief Initialize scene cameras and add them to the data stream
   *
   * This method detects cameras in the scene and initializes the corresponding
   * data structures in the data stream for tracking their states.
   */
  void initCameras();
};

} // namespace mujoco
