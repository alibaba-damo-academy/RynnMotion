#include "mj_interface.hpp"

#include <chrono>
#include <thread>

#include "mjcf_parser.hpp"

namespace mujoco {

MujocoInterface::MujocoInterface(const YAML::Node &mujocoYaml, const YAML::Node &motionYaml, int robotNumber, int sceneNumber) :
    InterfaceBase(motionYaml, robotNumber, sceneNumber),
    _mujocoYaml(mujocoYaml) {
  timingManager = std::make_unique<rynn::TimingManager>(rynn::TimingManager::TimeSource::SimTime);
  loadYaml();
  mjActuator = std::make_unique<MujocoActuator>(*this);
  mjSensor = std::make_unique<MujocoSensor>(*this);
  mjUI = std::make_unique<MujocoUI>(*this);
  mjScene = std::make_unique<MujocoScene>(*this);
  initMuJoCo();
  initDataStream();
  pinkine = std::make_unique<utils::PinKine>(robotManager->getPinoMJCF(), robotManager->getSiteNames());
}

MujocoInterface::~MujocoInterface() {
  closeMuJoCo();
}

void MujocoInterface::loadYaml() {
  YAML::Node visualNode;
  visualNode = _mujocoYaml["visualization"];

  double worldRate = _mujocoYaml["world_rate"].as<double>();
  double ctrlFreq = _mujocoYaml["control_rate"].as<double>();
  double renderFreq = _mujocoYaml["render_rate"].as<double>();

  physicsTimestep_ = 1.0 / worldRate;

  timingManager->addChannel("control", ctrlFreq);
  timingManager->addChannel("render", renderFreq);
  timingManager->addChannel("camera", renderFreq);

  _show_ee_frame = _mujocoYaml["show_ee_frame"].as<bool>();
  _show_ee_fk_result = _mujocoYaml["show_ee_fk_result"].as<bool>();
  _show_motor_qFB = _mujocoYaml["show_motor_qFB"].as<bool>();
  _show_motor_qdFB = _mujocoYaml["show_motor_qdFB"].as<bool>();
  _show_motor_qCmd = _mujocoYaml["show_motor_qCmd"].as<bool>();
  _show_motor_modes = _mujocoYaml["show_motor_modes"].as<bool>();
  _show_force_sensor = _mujocoYaml["show_force_sensor"].as<bool>();
  _show_traj = _mujocoYaml["show_traj"].as<bool>();
  _show_camera_pic = _mujocoYaml["show_camera_pic"].as<bool>();
  _show_sensor_indexarray = _mujocoYaml["show_sensor_indexarray"].as<bool>();
  _show_imu_data = _mujocoYaml["show_imu_data"].as<bool>();
  _show_gripper_debug = _mujocoYaml["show_gripper_debug"].as<bool>();
  _maxgeom = _mujocoYaml["maxgeom"].as<int>();

  auto ballSizeYaml = visualNode["ball_size"].as<std::vector<double>>();
  for (size_t i = 0; i < 3; ++i) {
    _visualData.lineSize_[i] = static_cast<mjtNum>(ballSizeYaml[i]);
  }
  auto ballRGBAYaml = visualNode["ball_rgba"].as<std::vector<float>>();
  for (size_t i = 0; i < 4; ++i) {
    _visualData.ballRGBA_[i] = ballRGBAYaml[i];
  }

  _save_mjModel = _mujocoYaml["save_mjModel"].as<bool>();
  _save_mjData = _mujocoYaml["save_mjData"].as<bool>();

  // Performance optimization settings
  if (_mujocoYaml["vsync_enabled"]) {
    _vsync_enabled = _mujocoYaml["vsync_enabled"].as<bool>();
  }
  if (_mujocoYaml["enable_busy_wait"]) {
    _enable_busy_wait = _mujocoYaml["enable_busy_wait"].as<bool>();
    busyWait_.store(_enable_busy_wait);
  }
  if (_mujocoYaml["busy_wait_threshold_us"]) {
    _busy_wait_threshold_us = _mujocoYaml["busy_wait_threshold_us"].as<int>();
  }
}

void MujocoInterface::getJointFeedbacks() {
  runtimeData_.simTime = mjData_->time;
  mjActuator->update();
  mjSensor->update();
  mjScene->update();

  printInfo();
}

void MujocoInterface::getEEFeedbacks() {
}

void MujocoInterface::setJointCommands() {
  mjActuator->setCommand();
}

void MujocoInterface::setEECommands() {
}

void MujocoInterface::step() {
  mj_step(mjModel_, mjData_);
}

void MujocoInterface::initDataStream() {
  runtimeData_.wallTime = mjData_->time;
}

void MujocoInterface::keyboard(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    switch (key) {
    case GLFW_KEY_SPACE: // Space to pause/unpause simulation
      pauseSim_ = !pauseSim_;
      break;

    case GLFW_KEY_BACKSPACE: // Backspace to reset simulation
      resetSim_ = true;
      break;

    case GLFW_KEY_ESCAPE: // Escape to exit
      exitRequest_.store(true);
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      break;

    case GLFW_KEY_B: // Toggle busy wait
      busyWait_.store(!busyWait_.load());
      break;

    default:
      break;
    }
  }
}

void MujocoInterface::mouse_move(GLFWwindow *window, double xpos, double ypos) {
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(mjModel_, action, dx / height, dy / height, &mjScene_, &mjCamera_);
}

void MujocoInterface::mouse_button(GLFWwindow *window, int button, int act, int mods) {
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  glfwGetCursorPos(window, &lastx, &lasty);
}

void MujocoInterface::scroll(GLFWwindow *window, double xoffset, double yoffset) {
  mjv_moveCamera(mjModel_, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &mjScene_, &mjCamera_);
}

void MujocoInterface::keyboardCallback(GLFWwindow *window, int key, int scancode, int act, int mods) {
  MujocoInterface *interface = static_cast<MujocoInterface *>(glfwGetWindowUserPointer(window));
  if (interface) {
    interface->keyboard(window, key, scancode, act, mods);
  }
}

void MujocoInterface::mouseMoveCallback(GLFWwindow *window, double xpos, double ypos) {
  MujocoInterface *interface = static_cast<MujocoInterface *>(glfwGetWindowUserPointer(window));
  if (interface) {
    interface->mouse_move(window, xpos, ypos);
  }
}

void MujocoInterface::mouseButtonCallback(GLFWwindow *window, int button, int act, int mods) {
  MujocoInterface *interface = static_cast<MujocoInterface *>(glfwGetWindowUserPointer(window));
  if (interface) {
    interface->mouse_button(window, button, act, mods);
  }
}

void MujocoInterface::scrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
  MujocoInterface *interface = static_cast<MujocoInterface *>(glfwGetWindowUserPointer(window));
  if (interface) {
    interface->scroll(window, xoffset, yoffset);
  }
}

void MujocoInterface::initMuJoCo() {
  char error[1000] = "Could not load XML model";

  _sceneMjcf = sceneManager->getSceneMJCF();
  mjModel_ = mj_loadXML(_sceneMjcf.c_str(), 0, error, 0);
  if (!mjModel_) {
    std::cerr << "Could not load scene MJCF: " << _sceneMjcf << std::endl;
    return;
  }

  mjData_ = mj_makeData(mjModel_);
  runtimeData_.mjData_ = mjData_;
  mjModel_->opt.timestep = physicsTimestep_;
  mj_forward(mjModel_, mjData_);
  double controlPeriod = timingManager->getPeriod("control");
  moduleManager->setDt(controlPeriod);
  saveModelInfo();
  mjActuator->initActuatorSystem();
  mjSensor->initSensors();
  mjScene->initScene();
  initRenderingComponents();
}

void MujocoInterface::initRenderingComponents() {
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }
  glfwWindow = glfwCreateWindow(1200, 800, "Damo Mujoco", NULL, NULL);
  glfwMakeContextCurrent(glfwWindow);
  glfwSwapInterval(_vsync_enabled ? 1 : 0); // Use config setting for V-Sync
  mjSetOptionSceneContext();
  setGlfwCallbacks();
  mjUI->initUI();
}

void MujocoInterface::setGlfwCallbacks() {
  glfwSetWindowUserPointer(glfwWindow, this);
  glfwSetKeyCallback(glfwWindow, keyboardCallback);
  glfwSetCursorPosCallback(glfwWindow, mouseMoveCallback);
  glfwSetMouseButtonCallback(glfwWindow, mouseButtonCallback);
  glfwSetScrollCallback(glfwWindow, scrollCallback);
}

void MujocoInterface::mjSetOptionSceneContext() {
  mjv_defaultPerturb(&mjPerturb_);
  mjv_defaultOption(&mjvOption_);
  mjv_defaultScene(&mjScene_);
  mjr_defaultContext(&mjContext_);
  mjv_makeScene(mjModel_, &mjScene_, _maxgeom);
  mjr_makeContext(mjModel_, &mjContext_, mjFONTSCALE_100);
}

void MujocoInterface::runApplication() {
  size_t physicsStepCount = 0;
  size_t controllerUpdateCount = 0;
  size_t renderUpdateCount = 0;
  auto debugStartTime = std::chrono::steady_clock::now();
  auto lastDebugTime = debugStartTime;
  double lastSimTime = 0.0;
  double lastDebugSimTime = 0.0;

  const double physicsTimestep = mjModel_->opt.timestep;
  const double physicsFreq = 1.0 / physicsTimestep;
  const auto physicsPeriod = std::chrono::microseconds(static_cast<int>(physicsTimestep * 1e6));
  auto nextPhysicsTime = std::chrono::steady_clock::now();

  std::cout << "=== MuJoCo Interface Timing Configuration ===" << std::endl;
  std::cout << "Physics frequency: " << physicsFreq << " Hz" << std::endl;
  std::cout << "Control frequency: " << 1.0 / timingManager->getPeriod("control") << " Hz (period: " << timingManager->getPeriod("control") << " s)" << std::endl;
  std::cout << "Render frequency: " << 1.0 / timingManager->getPeriod("render") << " Hz (period: " << timingManager->getPeriod("render") << " s)" << std::endl;
  std::cout << "==========================================\n"
            << std::endl;

  std::thread physicsThread([this, &physicsStepCount, &controllerUpdateCount, &renderUpdateCount,
                             &debugStartTime, &lastDebugTime, &lastSimTime, &lastDebugSimTime, &nextPhysicsTime, physicsPeriod]() {
    perfMonitor_.init();

    while (!exitRequest_.load() && !glfwWindowShouldClose(glfwWindow)) {
      if (pauseSim_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        nextPhysicsTime = std::chrono::steady_clock::now();
        continue;
      }

      if (resetSim_) {
        resetRoutine();
        resetSim_ = false;
        nextPhysicsTime = std::chrono::steady_clock::now();
        physicsStepCount = 0;
        controllerUpdateCount = 0;
        renderUpdateCount = 0;
        lastSimTime = 0.0;
        lastDebugSimTime = 0.0;
        debugStartTime = std::chrono::steady_clock::now();
        lastDebugTime = debugStartTime;
        perfMonitor_.init();
        continue;
      }

      mjUI->updateUI();

      // Measure controller time
      auto ctrlStart = std::chrono::steady_clock::now();
      callController();
      auto ctrlEnd = std::chrono::steady_clock::now();
      perfMonitor_.recordController(
          std::chrono::duration<double, std::milli>(ctrlEnd - ctrlStart).count());

      // Measure physics step time
      auto physicsStart = std::chrono::steady_clock::now();
      stepPhysics();
      auto physicsEnd = std::chrono::steady_clock::now();
      perfMonitor_.recordPhysics(
          std::chrono::duration<double, std::milli>(physicsEnd - physicsStart).count());

      physicsStepCount++;

      if (mjData_->time > lastSimTime + timingManager->getPeriod("control") * 0.99) {
        controllerUpdateCount++;
        lastSimTime = mjData_->time;
      }

      nextPhysicsTime += physicsPeriod;

      // Hybrid busy-wait for high precision timing
      if (busyWait_.load()) {
        const auto kBusyWaitThreshold = std::chrono::microseconds(_busy_wait_threshold_us);
        auto wakeTime = nextPhysicsTime - kBusyWaitThreshold;
        auto now = std::chrono::steady_clock::now();

        if (now < wakeTime) {
          std::this_thread::sleep_until(wakeTime);
        }

        while (std::chrono::steady_clock::now() < nextPhysicsTime) {
          std::this_thread::yield();
        }
      } else {
        // Standard sleep - lower CPU usage, lower precision
        std::this_thread::sleep_until(nextPhysicsTime);
      }

      auto now = std::chrono::steady_clock::now();
      if (now > nextPhysicsTime) {
        double drift = std::chrono::duration<double, std::milli>(now - nextPhysicsTime).count();
        perfMonitor_.recordSleepDrift(drift);
      }

      if (nextPhysicsTime < now) {
        nextPhysicsTime = now;
      }

      perfMonitor_.print(mjData_->time);
    }
  });

  renderLoop();
  exitRequest_.store(true);
  physicsThread.join();
}

void MujocoInterface::resetRoutine() {
  std::lock_guard<std::recursive_mutex> lock(mjMutex_);
  mj_resetData(mjModel_, mjData_);
  mjActuator->initFromKeyframe(0);
  mjScene->initScene();
  mj_forward(mjModel_, mjData_);
  resetController();
  updateDataToVisualize();
  resetRendering();
}

void MujocoInterface::callController() {
  if (pauseSim_) return;

  std::lock_guard<std::recursive_mutex> lock(mjMutex_);
  getFeedbacks();

  if (!timingManager->shouldTrigger("control", mjData_->time)) {
    return;
  }

  moduleManager->updateAllModules();
  setActuatorCommands();
}

void MujocoInterface::resetController() {
  getFeedbacks();
  moduleManager->resetAllModules();
  moduleManager->updateAllModules();
  timingManager->reset("control", mjData_->time);
  resetFsm();
}

void MujocoInterface::updateDataToVisualize() {
  mjv_updateScene(mjModel_, mjData_, &mjvOption_, NULL, &mjCamera_, mjCAT_ALL, &mjScene_);

  if (mjScene) {
    mjScene->renderTrackingVisuals();
  }

  if (!_show_traj) return;

  int adof = robotManager->getActionDOF();
  while (actualEEtraj.size() < static_cast<size_t>(adof)) {
    actualEEtraj.emplace_back(std::vector<Eigen::Vector3d>());
  }
  while (planEEtraj.size() < static_cast<size_t>(adof)) {
    planEEtraj.emplace_back(std::vector<Eigen::Vector3d>());
  }

  lastRenderedActualIndex = 0;
  lastRenderedPlanIndex = 0;

  recordActualEETraj();
  recordPlanEETraj();
  renderActualEETraj();
  renderPlanEETraj();
}

void MujocoInterface::addBaseOffsetPlanTraj(int branchId, Eigen::Vector3d &pos) {
  Eigen::Vector3d origin_offset = sceneManager->getOriginOffset();
  pos += origin_offset;
}

void MujocoInterface::addBaseOffsetActualTraj(int branchId, Eigen::Vector3d &pos) {
  auto robotType = robotManager->getRobotType();

  if (robotType == rynn::RobotType::dual_rm75) {
    if (branchId == 0) {
      pos = utils::rotmx(-M_PI * 0.5) * pos;
    } else if (branchId == 1) {
      pos = utils::rotmx(+M_PI * 0.5) * pos;
    }
  }

  Eigen::Vector3d originOffset = sceneManager->getOriginOffset();
  pos += originOffset;
}

void MujocoInterface::updateGeomConnector(mjvGeom &newGeom, const std::vector<Eigen::Vector3d> &path,
                                          int i, bool ignore_height_limit_flag, bool is_drawing_flag) {
  mjtNum from[3] = {newGeom.pos[0], newGeom.pos[1], newGeom.pos[2]};
  mjtNum to[3] = {newGeom.pos[0], newGeom.pos[1], newGeom.pos[2]};
  if (is_drawing_flag) {
    from[2] = 0.0;
    to[2] = 0.0;
  }
  if (i > 0) {
    if ((newGeom.pos[2] < 0.002 && path[i - 1][2] < 0.002) || ignore_height_limit_flag) {
      from[0] = path[i - 1][0];
      from[1] = path[i - 1][1];
      if (!is_drawing_flag) {
        from[2] = path[i - 1][2];
      }
    }
  }
  mjv_connector(&newGeom, mjGEOM_CAPSULE, 0.002, from, to);
}

void MujocoInterface::recordActualEETraj() {
  Eigen::Vector3d eePosFb = Eigen::Vector3d::Zero();
  if (runtimeData_.bodyStates.size() == 0) {
    int mdof = robotManager->getMotionDOF();
    Eigen::VectorXd qFb, qdFb_unused, qtauFb_unused;
    runtimeData_.getJointsFeedback(qFb, qdFb_unused, qtauFb_unused);
    pinkine->update(qFb.head(mdof));
    eePosFb = pinkine->getEEPos();
  } else {
    eePosFb = runtimeData_.bodyStates[0].pos;
  }
  addBaseOffsetActualTraj(0, eePosFb);

  actualEEtraj[0].emplace_back(eePosFb);
  utils::limitBufferSize(actualEEtraj[0], 5000);
}

void MujocoInterface::renderActualEETraj() {
  if (actualEEtraj.empty() || actualEEtraj[0].empty()) return;

  const auto &path = actualEEtraj[0];

  // Render all trajectory points (happens only at render frequency)
  for (size_t i = 0; i < path.size() && mjScene_.ngeom < mjScene_.maxgeom; ++i) {
    const Eigen::Vector3d &point = path[i];

    mjvGeom newGeom;
    float fadedOrange[4] = {_visualData.orange[0], _visualData.orange[1], _visualData.orange[2], 0.8f};
    if (point[2] > 0.0015) {
      fadedOrange[3] = 0.05f;
    }

    mjv_initGeom(&newGeom, mjGEOM_SPHERE, _visualData.lineSize_, NULL, NULL, fadedOrange);
    newGeom.pos[0] = static_cast<float>(point[0]);
    newGeom.pos[1] = static_cast<float>(point[1]);
    newGeom.pos[2] = static_cast<float>(point[2]);

    if (point[2] > 0.0015) {
      updateGeomConnector(newGeom, path, i, true, false);
    } else {
      updateGeomConnector(newGeom, path, i, true, true);
    }

    newGeom.shininess = 0.0;
    newGeom.reflectance = 0.0;
    newGeom.emission = 0.0;
    newGeom.specular = 0.0;
    mjScene_.geoms[mjScene_.ngeom] = newGeom;
    mjScene_.ngeom++;
  }
}

void MujocoInterface::recordPlanEETraj() {
  Eigen::Vector3d eePosCmd = Eigen::Vector3d::Zero();
  if (runtimeData_.bodyStates.size() == 0) {
    int mdof = robotManager->getMotionDOF();
    Eigen::VectorXd qCmd, qdCmd_unused, qtauCmd_unused;
    runtimeData_.getJointsCommand(qCmd, qdCmd_unused, qtauCmd_unused);
    pinkine->update(qCmd.head(mdof));
    eePosCmd = pinkine->getEEPos();
  } else {
    eePosCmd = runtimeData_.bodyPlans[0].pos;
  }
  addBaseOffsetPlanTraj(0, eePosCmd);

  planEEtraj[0].emplace_back(eePosCmd);
  utils::limitBufferSize(planEEtraj[0], 100);
}

void MujocoInterface::renderPlanEETraj() {
  if (planEEtraj.empty() || planEEtraj[0].empty()) return;

  const auto &path = planEEtraj[0];

  // Render all trajectory points (happens only at render frequency)
  for (size_t i = 0; i < path.size() && mjScene_.ngeom < mjScene_.maxgeom; ++i) {
    const Eigen::Vector3d &point = path[i];

    mjvGeom newGeom;
    float fadedCyan[4] = {_visualData.cyan[0], _visualData.cyan[1], _visualData.cyan[2], 0.8f};
    if (point[2] > 0.0005) {
      fadedCyan[3] = 0.05f;
    }

    mjv_initGeom(&newGeom, mjGEOM_SPHERE, _visualData.lineSize_, NULL, NULL, fadedCyan);
    newGeom.pos[0] = static_cast<float>(point[0]);
    newGeom.pos[1] = static_cast<float>(point[1]);
    newGeom.pos[2] = static_cast<float>(point[2]);

    if (point[2] > 0.0005) {
      updateGeomConnector(newGeom, path, i, true, false);
    } else {
      updateGeomConnector(newGeom, path, i, true, true);
    }

    newGeom.shininess = 0.0;
    newGeom.reflectance = 0.0;
    newGeom.emission = 0.0;
    newGeom.specular = 0.0;
    mjScene_.geoms[mjScene_.ngeom] = newGeom;
    mjScene_.ngeom++;
  }
}

void MujocoInterface::callRendering() {
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(glfwWindow, &viewport.width, &viewport.height);
  mjr_render(viewport, &mjScene_, &mjContext_);
  mjUI->updateScreenMessage(viewport, &mjContext_);
  glfwSwapBuffers(glfwWindow);
}

void MujocoInterface::resetRendering() {
  timingManager->reset("render", mjData_->time);
}

void MujocoInterface::printInfo() {
  if (_show_motor_qFB || _show_motor_qdFB || _show_ee_fk_result) {
    Eigen::VectorXd qFb, qdFb, qtauFb;
    runtimeData_.getJointsFeedback(qFb, qdFb, qtauFb);

    if (_show_motor_qFB) {
      std::cout << "qFB: " << qFb.transpose() << std::endl;
    }
    if (_show_motor_qdFB) {
      std::cout << "time: " << runtimeData_.simTime
                << " qdFB: " << qdFb.transpose() << std::endl;
    }
    if (_show_ee_fk_result) {
      pinkine->update(qFb);
      std::cout << "mjTime: " << std::fixed << std::setprecision(5) << mjData_->time
                << " sec, pinocchio eePos: " << pinkine->getEEPos().transpose() + sceneManager->getOriginOffset().transpose()
                << "; pinocchio eeQuat(x,y,z,w): " << pinkine->getEEQuat().transpose() << std::endl;
    }
  }
}

void MujocoInterface::closeMuJoCo() {
  if (mjData_) mj_deleteData(mjData_);
  if (mjModel_) mj_deleteModel(mjModel_);

  mjv_freeScene(&mjScene_);
  mjr_freeContext(&mjContext_);
  if (glfwWindow) glfwDestroyWindow(glfwWindow);
  glfwTerminate();
}

void MujocoInterface::printLinkPosition(const char *link_name) {
  int body_id = mj_name2id(mjModel_, mjOBJ_BODY, link_name);
  if (body_id == -1) {
    std::cerr << "Link name not found: " << link_name << std::endl;
    return;
  }
  const double *link_pos = mjData_->xpos + 3 * body_id;
  std::cout << "Position of link '" << link_name << "': ("
            << link_pos[0] << ", " << link_pos[1] << ", " << link_pos[2] << ")" << std::endl;
}

void MujocoInterface::renderLoop() {
  while (!glfwWindowShouldClose(glfwWindow) && !exitRequest_.load()) {
    if (timingManager->shouldTrigger("render", mjData_->time)) {
      auto renderStart = std::chrono::steady_clock::now();

      std::lock_guard<std::recursive_mutex> lock(mjMutex_);
      updateDataToVisualize();

      callRendering();

      mjSensor->updateRGB();

      auto renderEnd = std::chrono::steady_clock::now();
      perfMonitor_.recordRender(
          std::chrono::duration<double, std::milli>(renderEnd - renderStart).count());
    }
    glfwPollEvents();

    // Prevent busy-wait - add small sleep to reduce CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  exitRequest_.store(true);
}

void MujocoInterface::stepPhysics() {
  std::lock_guard<std::recursive_mutex> lock(mjMutex_);
  step();
}

void MujocoInterface::saveModelInfo() {
  if (_save_mjModel) {
    mj_printModel(mjModel_, "mjModel.txt");
  }
  if (_save_mjData) {
    mj_printData(mjModel_, mjData_, "mjData.txt");
  }
}

} // namespace mujoco
