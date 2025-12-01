#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

#include "fsm_manager.hpp"
#include "module_manager.hpp"
#include "robot_manager.hpp"
#include "runtime_data.hpp"
#include "scene_manager.hpp"
#include "timing_manager.hpp"

/**
 * @class InterfaceBase
 * @brief Base class for all robot interfaces (simulation and hardware)
 *
 * This class provides a uniform interface for robot control systems,
 * standardizing how robots are configured, initialized, and controlled.
 * It can be specialized for hardware robots or simulation environments.
 */
class InterfaceBase {
public:
  /**
   * @brief Constructor for InterfaceBase
   *
   * @param motionYaml Motion configuration YAML node
   * @param robotNumber robot number to initialize
   * @param sceneNumber Scene configuration number
   */
  InterfaceBase(const YAML::Node &motionYaml,
                int robotNumber,
                int sceneNumber = 1);

  virtual ~InterfaceBase() = default;

  /**
   * @brief Run the main control application
   *
   * This method contains the main control loop that handles
   * sensor feedback, controller updates, and actuator commands.
   */
  virtual void runApplication() = 0;

protected:
  /**
   * @brief Load parameters from YAML configuration
   */
  virtual void loadYaml() = 0;

  /**
   * @brief Get feedback from sensors or simulation
   * Calls getJointFeedbacks() and getEEFeedbacks()
   */
  virtual void getFeedbacks();

  /**
   * @brief Get joint feedback from sensors or simulation
   */
  virtual void getJointFeedbacks() = 0;

  /**
   * @brief Get end-effector feedback (optional, default: empty)
   */
  virtual void getEEFeedbacks();

  /**
   * @brief Send commands to actuators or simulation
   * Calls setJointCommands() and setEECommands()
   */
  virtual void setActuatorCommands();

  /**
   * @brief Send joint commands to actuators or simulation
   */
  virtual void setJointCommands() = 0;

  /**
   * @brief Send end-effector commands (optional, default: empty)
   */
  virtual void setEECommands();

  /**
   * @brief Advance physics simulation by one step
   * For simulation: calls physics step (e.g., mj_step)
   * For real robot: no-op
   */
  virtual void step() = 0;

  /**
   * @brief Call the controller to update commands based on feedback
   */
  virtual void callController();

  /**
   * @brief Reset the controller to initial state
   */
  virtual void resetController();

  /**
   * @brief Reset the FSM to initial state
   */
  virtual void resetFsm();

  YAML::Node _motionYaml;
  int _robotNumber;
  int _sceneNumber;
  double _controlFreq{1000.0};

  std::unique_ptr<rynn::RobotManager> robotManager;
  std::unique_ptr<rynn::SceneManager> sceneManager;
  std::unique_ptr<rynn::ModuleManager> moduleManager;
  std::unique_ptr<rynn::FsmManager> fsmManager;
  data::RuntimeData runtimeData_;
  std::unique_ptr<rynn::TimingManager> timingManager;

private:
  /**
   * @brief Initialize the interface with proper DOFs and gains
   *
   * This method is final and not overridable by subclasses to ensure
   * consistent initialization. It creates the dataStream and moduleManager,
   * and sets up actuators with proper gains.
   */
  void initInterface();
};
