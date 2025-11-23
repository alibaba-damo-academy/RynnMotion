#include "mj_ui.hpp"

#include "mj_interface.hpp"

namespace mujoco {

MujocoUI::MujocoUI(MujocoInterface &mj) :
    mj_(mj),
    sceneType_(rynn::SceneType::kDefault) {
}

void MujocoUI::initUI() {
  uiObjectName = mj_._mujocoYaml["uiObject_name"].as<std::string>();

  sceneType_ = mj_.sceneManager->getSceneType();

  // Detect dual-arm configuration
  numEE_ = mj_.robotManager->getNumEndEffectors();
  isDualArm_ = (numEE_ >= 2);

  if (isDualArm_) {
    robotBaseOffset_ = Eigen::Vector3d(-0.25, 0.0, 0.75);
  }
}

Eigen::Vector3d MujocoUI::getMjDataPos(int adr) const {
  return Eigen::Vector3d(
      mj_.mjData_->qpos[adr],
      mj_.mjData_->qpos[adr + 1],
      mj_.mjData_->qpos[adr + 2]);
}

Eigen::Quaterniond MujocoUI::getMjDataQuat(int adr) const {
  Eigen::Quaterniond q(
      mj_.mjData_->qpos[adr + 3], // w
      mj_.mjData_->qpos[adr + 4], // x
      mj_.mjData_->qpos[adr + 5], // y
      mj_.mjData_->qpos[adr + 6]  // z
  );
  q.normalize();
  return q;
}

void MujocoUI::setMjDataPos(int adr, const Eigen::Vector3d &xyz) {
  mj_.mjData_->qpos[adr] = xyz.x();
  mj_.mjData_->qpos[adr + 1] = xyz.y();
  mj_.mjData_->qpos[adr + 2] = xyz.z();
}

void MujocoUI::setMjDataQuat(int adr, const Eigen::Quaterniond &q) {
  mj_.mjData_->qpos[adr + 3] = q.w();
  mj_.mjData_->qpos[adr + 4] = q.x();
  mj_.mjData_->qpos[adr + 5] = q.y();
  mj_.mjData_->qpos[adr + 6] = q.z();
}

void MujocoUI::updateUI() {
  getMouseAction();
  getKeyboardAction();
  updateRobotUI();
}

void MujocoUI::updateRobotUI() {
  auto robotType = mj_.robotManager->getRobotType();

  if (sceneType_ == rynn::SceneType::kUI || sceneType_ == rynn::SceneType::kTracking) {
    updateTrackingObjectUI();
    return;
  }

  if (robotType == rynn::RobotType::diffcar) {
    updateDiffCarUI();
    return;
  }
}

void MujocoUI::getKeyboardAction() {
  auto robotType = mj_.robotManager->getRobotType();

  motion_dxyz.setZero();
  motion_drpy.setZero();

  if (sceneType_ == rynn::SceneType::kUI || sceneType_ == rynn::SceneType::kTracking) {
    getKeyboardActionFranka();
    return;
  }

  if (robotType == rynn::RobotType::diffcar) {
    getKeyboardActionDiffCar();
    return;
  }
}

void MujocoUI::getKeyboardActionDiffCar() {
  int key_up = glfwGetKey(mj_.glfwWindow, GLFW_KEY_UP);
  int key_down = glfwGetKey(mj_.glfwWindow, GLFW_KEY_DOWN);
  int key_left = glfwGetKey(mj_.glfwWindow, GLFW_KEY_LEFT);
  int key_right = glfwGetKey(mj_.glfwWindow, GLFW_KEY_RIGHT);

  if (key_up == GLFW_PRESS) {
    velCmd[0] += velStep[0];
  } else if (key_down == GLFW_PRESS) {
    velCmd[0] -= velStep[0];
  }

  if (key_left == GLFW_PRESS) {
    velCmd[1] += velStep[1];
  } else if (key_right == GLFW_PRESS) {
    velCmd[1] -= velStep[1];
  }

  velCmd[0] = std::clamp(velCmd[0], -1.0, 1.0);
  velCmd[1] = std::clamp(velCmd[1], -1.0, 1.0);
}

void MujocoUI::getKeyboardActionFranka() {
  int key_up = glfwGetKey(mj_.glfwWindow, GLFW_KEY_UP);
  int key_down = glfwGetKey(mj_.glfwWindow, GLFW_KEY_DOWN);
  int key_left = glfwGetKey(mj_.glfwWindow, GLFW_KEY_LEFT);
  int key_right = glfwGetKey(mj_.glfwWindow, GLFW_KEY_RIGHT);
  int key_page_up = glfwGetKey(mj_.glfwWindow, GLFW_KEY_PAGE_UP);
  int key_page_down = glfwGetKey(mj_.glfwWindow, GLFW_KEY_PAGE_DOWN);

  int key_q = glfwGetKey(mj_.glfwWindow, GLFW_KEY_Q);
  int key_a = glfwGetKey(mj_.glfwWindow, GLFW_KEY_A);
  int key_w = glfwGetKey(mj_.glfwWindow, GLFW_KEY_W);
  int key_s = glfwGetKey(mj_.glfwWindow, GLFW_KEY_S);
  int key_e = glfwGetKey(mj_.glfwWindow, GLFW_KEY_E);
  int key_d = glfwGetKey(mj_.glfwWindow, GLFW_KEY_D);

  double move_step = 0.0005;
  double rotate_step = 0.001;

  if (key_right == GLFW_PRESS) {
    motion_dxyz[0] = move_step; // +X (right)
  } else if (key_left == GLFW_PRESS) {
    motion_dxyz[0] = -move_step; // -X (left)
  }

  if (key_up == GLFW_PRESS) {
    motion_dxyz[1] = move_step; // +Y (forward)
  } else if (key_down == GLFW_PRESS) {
    motion_dxyz[1] = -move_step; // -Y (backward)
  }

  if (key_page_up == GLFW_PRESS) {
    motion_dxyz[2] = move_step; // +Z (up)
  } else if (key_page_down == GLFW_PRESS) {
    motion_dxyz[2] = -move_step; // -Z (down)
  }

  if (key_q == GLFW_PRESS) {
    motion_drpy[0] = rotate_step; // +Roll
  } else if (key_a == GLFW_PRESS) {
    motion_drpy[0] = -rotate_step; // -Roll
  }

  if (key_w == GLFW_PRESS) {
    motion_drpy[1] = rotate_step; // +Pitch
  } else if (key_s == GLFW_PRESS) {
    motion_drpy[1] = -rotate_step; // -Pitch
  }

  if (key_e == GLFW_PRESS) {
    motion_drpy[2] = rotate_step; // +Yaw
  } else if (key_d == GLFW_PRESS) {
    motion_drpy[2] = -rotate_step; // -Yaw
  }

  static bool enterKeyPressed = false;
  bool enterKeyCurrentlyPressed = (glfwGetKey(mj_.glfwWindow, GLFW_KEY_ENTER) == GLFW_PRESS);
  if (enterKeyCurrentlyPressed && !enterKeyPressed) {
    int numEE = mj_.robotManager->getNumEndEffectors();
    for (int i = 0; i < numEE; i++) {
      mj_.runtimeData_.gripperCommands[i].toggleGripper = true;
    }
  }
  enterKeyPressed = enterKeyCurrentlyPressed;
}

void MujocoUI::getMouseAction() {
  if (glfwGetMouseButton(mj_.glfwWindow, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && (glfwGetKey(mj_.glfwWindow, GLFW_KEY_LEFT_SHIFT) || glfwGetKey(mj_.glfwWindow, GLFW_KEY_RIGHT_SHIFT))) {
    double mouse_x, mouse_y;
    glfwGetCursorPos(mj_.glfwWindow, &mouse_x, &mouse_y);

    int window_width, window_height;
    glfwGetWindowSize(mj_.glfwWindow, &window_width, &window_height);

    float rel_x = mouse_x / window_width;
    float rel_y = mouse_y / window_height;

    mjvOption opt;
    mjv_defaultOption(&opt);
    opt.flags[mjVIS_STATIC] = 1; // Show static geometries

    mjtNum select_point[3];
    int geom_id[1], flex_id[1], skin_id[1];

    int selected_id = mjv_select(
        mj_.mjModel_,                           // Model
        mj_.mjData_,                            // Data
        &opt,                                   // Visualization options
        (mjtNum)(window_width / window_height), // Aspect ratio
        rel_x,                                  // Mouse relative X coordinate
        rel_y,                                  // Mouse relative Y coordinate
        &mj_.mjScene_,                          // Scene structure
        select_point,                           // Output: selected point's world coordinates
        geom_id,                                // Output: selected geometry ID
        flex_id,
        skin_id);

    if (selected_id >= 0) {
      std::string selected_obj_name = std::string(mj_id2name(mj_.mjModel_, mjOBJ_BODY, selected_id));
      if (selected_obj_name != uiObjectName) {
        uiObjectName = selected_obj_name;
        last_pos_is_valid_ = false;
        std::cout << "Selected object: " << selected_obj_name << std::endl;
      }
    } else {
      if (!uiObjectName.empty()) {
        std::cout << "No geometry selected." << std::endl;
        uiObjectName = "";
        last_pos_is_valid_ = false;
      }
    }
  }
}

void MujocoUI::updateTrackingObjectUI() {
  if (fabs(motion_dxyz[0]) > 0 || fabs(motion_dxyz[1]) > 0 || fabs(motion_dxyz[2]) > 0) {
    Eigen::Vector3d testPos = last_xyz_ + motion_dxyz;
    double dis = testPos.squaredNorm();
    if (dis > radius_robot_arm * radius_robot_arm) {
      motion_dxyz.setZero();
    }
  }

  if (last_pos_is_valid_) {
    Eigen::Vector3d xyz = last_xyz_ + motion_dxyz;
    Eigen::Quaterniond dq = utils::EulerZYX2Quaternion(motion_drpy);
    Eigen::Quaterniond q = last_q_ * dq;

    last_xyz_ = xyz;
    last_q_ = q;
  } else {
    if (isDualArm_) {
      last_xyz_ = Eigen::Vector3d(0.3, 0.0, 0.4);
    } else {
      last_xyz_ = Eigen::Vector3d(0.2, 0.0, 0.2);
    }
    last_q_ = Eigen::Quaterniond::Identity();
    last_pos_is_valid_ = true;
  }

  auto *trackObj = mj_.runtimeData_.getObjectByName("cube");
  if (trackObj) {
    trackObj->pos = last_xyz_;
    trackObj->quat = last_q_;
  }
}

void MujocoUI::updateDiffCarUI() {
  std::string car_name = "diffcar";
  int body_id = mj_name2id(mj_.mjModel_, mjOBJ_BODY, car_name.c_str());

  if (body_id == -1) {
    car_name = "car";
    body_id = mj_name2id(mj_.mjModel_, mjOBJ_BODY, car_name.c_str());

    if (body_id == -1) {
      int num_actuators = mj_.mjModel_->nu;
      if (num_actuators >= 2) {
        mj_.mjData_->ctrl[0] = velCmd[0] + velCmd[1];
        mj_.mjData_->ctrl[1] = velCmd[0] - velCmd[1];
        return;
      }
      return;
    }
  }

  int jnt_adr = mj_.mjModel_->body_jntadr[body_id];
  if (jnt_adr == -1) {
    return;
  }

  int qpos_adr = mj_.mjModel_->jnt_qposadr[jnt_adr];

  Eigen::Vector3d current_pos = getMjDataPos(qpos_adr);
  Eigen::Quaterniond current_quat = getMjDataQuat(qpos_adr);

  Eigen::Matrix3d rotation_matrix = current_quat.toRotationMatrix();
  Eigen::Vector3d forward_dir = rotation_matrix.col(0); // Assuming X is forward

  double position_step = 0.001;
  Eigen::Vector3d position_change = forward_dir * velCmd[0] * position_step;

  double angle_step = 0.001;
  Eigen::Vector3d rotation_axis(0, 0, 1); // Rotate around Z-axis
  Eigen::Quaterniond rotation_delta = Eigen::Quaterniond(
      Eigen::AngleAxisd(velCmd[1] * angle_step, rotation_axis));

  Eigen::Vector3d new_pos = current_pos + position_change;
  Eigen::Quaterniond new_quat = rotation_delta * current_quat;
  new_quat.normalize();

  setMjDataPos(qpos_adr, new_pos);
  setMjDataQuat(qpos_adr, new_quat);

  mj_forward(mj_.mjModel_, mj_.mjData_);
}

void MujocoUI::updateScreenMessage(mjrRect viewport, const mjrContext *context) {
  mjScreenMsg_.clear();
  mjScreenMsg_.str("");
  mjInteractionMsg_.clear();
  mjInteractionMsg_.str("");

  mjScreenMsg_ << "time: " << std::fixed << std::setprecision(2) << mj_.mjData_->time;
  mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, box1, mjScreenMsg_.str().data(), 0, context);

  auto robotType = mj_.robotManager->getRobotType();

  if (sceneType_ == rynn::SceneType::kUI || sceneType_ == rynn::SceneType::kTracking) {
    mjInteractionMsg_ << "Object Tracking Scene with Keyboard UI";

    mjInteractionMsg_ << "\n\nGRIPPER:";
    mjInteractionMsg_ << "\n  Enter: Toggle open/close";

    mjInteractionMsg_ << "\n\ntracking eePos:";
    mjInteractionMsg_ << "\n  Arrow Keys: Move X/Y (horizontal plane)";
    mjInteractionMsg_ << "\n  PageUp/PageDown: Move Z (vertical)";

    mjInteractionMsg_ << "\n\ntracking eeRPY:";
    mjInteractionMsg_ << "\n  Q/A: Roll (X-axis) +/-";
    mjInteractionMsg_ << "\n  W/S: Pitch (Y-axis) +/-";
    mjInteractionMsg_ << "\n  E/D: Yaw (Z-axis) +/-";

    mjInteractionMsg_ << "\n\nCurrent Reference eeXYZRPY: ";
    mjInteractionMsg_ << "\n"
                      << std::fixed << std::setprecision(3);
    if (!uiObjectName.empty() && last_pos_is_valid_) {
      Eigen::Vector3d rpy = utils::quat2RPY(last_q_.coeffs());
      mjInteractionMsg_ << "[" << last_xyz_[0] << ", " << last_xyz_[1] << ", " << last_xyz_[2]
                        << ", " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << "]";
    } else {
      mjInteractionMsg_ << "N/A";
    }

    box2.left = viewport.width - box2.width - 10;
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, box2, mjInteractionMsg_.str().data(), 0, context);
  } else if (robotType == rynn::RobotType::diffcar) { // diffCar
    mjInteractionMsg_ << "Differential Car Controls:";
    mjInteractionMsg_ << "\n\nVELOCITY CONTROLS:";
    mjInteractionMsg_ << "\nUp/Down: Forward/Backward";
    mjInteractionMsg_ << "\nLeft/Right: Turn Left/Right";
    mjInteractionMsg_ << "\n\nCurrent Values:";
    mjInteractionMsg_ << "\nForward: " << std::fixed << std::setprecision(2) << velCmd[0];
    mjInteractionMsg_ << "\nAngular: " << std::fixed << std::setprecision(2) << velCmd[1];

    box2.left = viewport.width - box2.width - 10;

    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, box2, mjInteractionMsg_.str().data(), 0, context);
  }
}

} // namespace mujoco
