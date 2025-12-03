#pragma once

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>

#include "glfw3.h"
#ifdef __APPLE__
#include "mujoco.h"
#else
#include "mujoco/mujoco.h"
#endif
#include "orient_tools.hpp"
#include "robot_manager.hpp"
#include "scene_manager.hpp"

namespace mujoco {

class MujocoInterface;

class MujocoUI {
public:
  MujocoUI(MujocoInterface &mj);
  ~MujocoUI() = default;

  void initUI();
  void updateUI();
  void updateScreenMessage(mjrRect viewport, const mjrContext *context);

private:
  void getKeyboardAction();
  void getKeyboardActionRecording();
  void getKeyboardActionFranka();
  void getKeyboardActionDiffCar();

  void getMouseAction();

  void updateRobotUI();
  void updateTrackingObjectUI();
  void updateDiffCarUI();

  Eigen::Vector3d getMjDataPos(int adr) const;
  Eigen::Quaterniond getMjDataQuat(int adr) const;
  void setMjDataPos(int adr, const Eigen::Vector3d &xyz);
  void setMjDataQuat(int adr, const Eigen::Quaterniond &q);

private:
  MujocoInterface &mj_;
  rynn::SceneType sceneType_;  // Cached scene type for efficient access
  std::stringstream mjScreenMsg_;
  std::stringstream mjInteractionMsg_;
  mjrRect box1{0, 0, 100, 50};
  mjrRect box2{0, 0, 460, 380};
  Eigen::Vector3d last_xyz_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_q_{Eigen::Quaterniond::Identity()};
  bool last_pos_is_valid_ = false;
  std::string uiObjectName = "";
  double radius_robot_arm = 1.1;

  Eigen::Vector3d motion_dxyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d motion_drpy{Eigen::Vector3d::Zero()};

  Eigen::Vector2d velCmd{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velStep{0.005, 0.002};

  // Dual-arm support
  int numEE_ = 0;
  bool isDualArm_ = false;
  Eigen::Vector3d robotBaseOffset_{Eigen::Vector3d::Zero()};

  // Recording key states (for edge detection)
  bool keyR_pressed_{false};
  bool keyN_pressed_{false};
};

} // namespace mujoco
