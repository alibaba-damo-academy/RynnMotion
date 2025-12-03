#pragma once
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "debug_config.hpp"
#include "glfw3.h"
#include "interface_base.hpp"
#include "kine_pin.hpp"
#include "math_tools.hpp"
#include "mj_actuator.hpp"
#include "mj_recorder.hpp"
#include "mj_scene.hpp"
#include "mj_sensor.hpp"
#include "mj_ui.hpp"
#include "module_base.hpp"
#include "module_manager.hpp"
#include "runtime_data.hpp"
#ifdef __APPLE__
#include "mujoco.h"
#else
#include "mujoco/mujoco.h"
#endif
#include "scene_manager.hpp"

namespace mujoco {

class Timer;
class MujocoActuator;
class MujocoSensor;
class MujocoUI;
class MujocoScene;
class MujocoRecorder;

struct PerformanceMonitor {
  std::chrono::steady_clock::time_point startTime;
  std::chrono::steady_clock::time_point lastPrintTime;

  double physicsTimeMs = 0.0;
  double controllerTimeMs = 0.0;
  double renderTimeMs = 0.0;
  double sleepDriftMs = 0.0;

  double maxPhysicsMs = 0.0;
  double maxControllerMs = 0.0;
  double maxRenderMs = 0.0;
  double maxSleepDriftMs = 0.0;

  size_t samples = 0;

  void init() {
    startTime = std::chrono::steady_clock::now();
    lastPrintTime = startTime;
  }

  void recordPhysics(double ms) {
    physicsTimeMs += ms;
    maxPhysicsMs = std::max(maxPhysicsMs, ms);
  }

  void recordController(double ms) {
    controllerTimeMs += ms;
    maxControllerMs = std::max(maxControllerMs, ms);
  }

  void recordRender(double ms) {
    renderTimeMs += ms;
    maxRenderMs = std::max(maxRenderMs, ms);
  }

  void recordSleepDrift(double ms) {
    sleepDriftMs += ms;
    maxSleepDriftMs = std::max(maxSleepDriftMs, ms);
  }

  void print(double simTime) {
    samples++;
    if (samples >= 1000) {
      auto now = std::chrono::steady_clock::now();
      double wallTime = std::chrono::duration<double>(now - startTime).count();
      double rtRatio = (wallTime > 0.0) ? (simTime / wallTime) : 0.0;

      if (utils::DebugConfig::getInstance().isVerbose()) {
        std::cout << "\n=== Performance Report (1000 iterations) ===" << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Real-time ratio: " << rtRatio << " (Target: 1.000)" << std::endl;
        std::cout << "Avg Physics:     " << (physicsTimeMs / samples)
                  << " ms  (Max: " << maxPhysicsMs << " ms)" << std::endl;
        std::cout << "Avg Controller:  " << (controllerTimeMs / samples)
                  << " ms  (Max: " << maxControllerMs << " ms)" << std::endl;
        std::cout << "Avg Render:      " << (renderTimeMs / samples)
                  << " ms  (Max: " << maxRenderMs << " ms)" << std::endl;
        std::cout << "Avg Sleep Drift: " << (sleepDriftMs / samples)
                  << " ms  (Max: " << maxSleepDriftMs << " ms)" << std::endl;
        std::cout << "================================================\n"
                  << std::endl;
      }

      samples = 0;
      physicsTimeMs = controllerTimeMs = renderTimeMs = sleepDriftMs = 0.0;
      maxPhysicsMs = maxControllerMs = maxRenderMs = maxSleepDriftMs = 0.0;
      lastPrintTime = now;
    }
  }
};

class MujocoInterface : public InterfaceBase {
  friend class MujocoActuator;
  friend class MujocoSensor;
  friend class MujocoUI;
  friend class MujocoScene;
  friend class MujocoRecorder;

public:
  MujocoInterface(const YAML::Node &mujocoYaml, const YAML::Node &motionYaml, int robotNumber, int sceneNumber = 1);
  ~MujocoInterface();

  void runApplication() override;
  void renderLoop();
  void stepPhysics();

  bool isPaused() const {
    return pauseSim_;
  }
  bool isResetRequested() const {
    return resetSim_;
  }
  bool isExitRequested() const {
    return exitRequest_.load();
  }
  bool isBusyWait() const {
    return busyWait_;
  }
  void clearResetRequest() {
    resetSim_ = false;
  }

protected:
  void callController() override;
  void resetController() override;
  void loadYaml() override;
  void getJointFeedbacks() override;
  void getEEFeedbacks() override;
  void setJointCommands() override;
  void setEECommands() override;
  void step() override;

  void resetRoutine();

private:
  void keyboard(GLFWwindow *window, int key, int scancode, int action, int mods);
  void mouse_move(GLFWwindow *window, double xpos, double ypos);
  void mouse_button(GLFWwindow *window, int button, int act, int mods);
  void scroll(GLFWwindow *window, double xoffset, double yoffset);

  static void keyboardCallback(GLFWwindow *window, int key, int scancode, int act, int mods);
  static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos);
  static void mouseButtonCallback(GLFWwindow *window, int button, int act, int mods);
  static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);

protected:
  YAML::Node _mujocoYaml;
  std::string _sceneMjcf;

  mjModel *mjModel_{nullptr};
  mjData *mjData_{nullptr};
  GLFWwindow *glfwWindow{nullptr};
  mjvCamera mjCamera_;
  mjvPerturb mjPert_;
  mjvOption mjvOption_;
  mjvScene mjScene_;
  mjrContext mjContext_;
  mjrRect mjRect_;
  mjvPerturb mjPerturb_;
  mjrRect msgBox{0, 0, 100, 50};

  std::recursive_mutex mjMutex_;
  std::atomic<bool> exitRequest_{false};
  std::atomic<bool> busyWait_{false};
  bool pauseSim_ = false;
  bool resetSim_ = false;

  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;
  double lastx = 0;
  double lasty = 0;

  void initMuJoCo();
  void initRenderingComponents();
  void mjSetOptionSceneContext();
  void updateDataToVisualize();
  void setGlfwCallbacks();
  void closeMuJoCo();
  void printLinkPosition(const char *link_name);
  void saveModelInfo();

  void callRendering();
  void resetRendering();

protected:
  void initDataStream();
  void printInfo();
  bool _show_ee_frame{false};
  bool _show_ee_fk_result{false};
  bool _show_motor_qFB{false};
  bool _show_motor_qdFB{false};
  bool _show_motor_qCmd{false};
  bool _show_motor_modes{false};
  bool _show_force_sensor{false};
  bool _show_traj{true};
  bool _show_camera_pic{true};
  bool _show_sensor_indexarray{false};
  bool _show_imu_data{false};
  bool _show_gripper_debug{false};
  bool _save_mjModel{false};
  bool _save_mjData{false};
  int _maxgeom{1500}; // Increased from 1000 to accommodate tracking visuals
  std::unique_ptr<MujocoActuator> mjActuator;
  std::unique_ptr<MujocoSensor> mjSensor;
  std::unique_ptr<MujocoUI> mjUI;
  std::unique_ptr<MujocoScene> mjScene;
  std::unique_ptr<MujocoRecorder> mjRecorder;
  std::unique_ptr<utils::PinKine> pinkine;

protected:
  struct VisualData {
    float ballRGBA_[4]{1, 0, 0, 1};
    float yellow[4]{1.0f, 1.0f, 0.0f, 1.0f};
    float orange[4]{1.0f, 0.647f, 0.0f, 1.0f};
    float cyan[4]{0.0f, 1.0f, 1.0f, 1.0f};
    mjtNum lineSize_[3]{0.02, 0.02, 0.02};
    mjtNum eeSize_[3]{0.01, 0.01, 0.01};
    std::vector<Eigen::Vector3d> pointBuffer_;
  };
  VisualData _visualData;
  std::vector<std::vector<Eigen::Vector3d>> planEEtraj;
  std::vector<std::vector<Eigen::Vector3d>> actualEEtraj;

  size_t lastRenderedActualIndex{0};
  size_t lastRenderedPlanIndex{0};

  std::vector<double> multiRayDistances_;

  void updateGeomConnector(mjvGeom &newGeom, const std::vector<Eigen::Vector3d> &path,
                           int i, bool ignore_height_limit_flag, bool is_drawing_flag);
  void recordActualEETraj();
  void renderActualEETraj();
  void recordPlanEETraj();
  void renderPlanEETraj();
  void addBaseOffsetPlanTraj(int branchId, Eigen::Vector3d &pos);
  void addBaseOffsetActualTraj(int branchId, Eigen::Vector3d &pos);

private:
  double physicsTimestep_{0.001};

  PerformanceMonitor perfMonitor_;
  bool _vsync_enabled{false};
  bool _enable_busy_wait{false};
  int _busy_wait_threshold_us{100};

  void initRecorder();
};

} // namespace mujoco
