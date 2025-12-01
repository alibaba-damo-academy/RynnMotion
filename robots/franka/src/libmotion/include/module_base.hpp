#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <memory>
#include <vector>

#include "runtime_data.hpp"
#include "fsm_manager.hpp"
#include "kine_pin.hpp"
#include "robot_manager.hpp"
#include "robot_types.hpp"
#include "scene_manager.hpp"

namespace rynn {

/**
 * @class CModuleBase
 * @brief Base class for all modules with dependency injection
 *
 * All modules share a single RuntimeData instance and operate on it sequentially.
 * Each module should document what it reads from and writes to RuntimeData.
 */
class CModuleBase {
public:
  explicit CModuleBase(const YAML::Node &yamlNode);
  virtual ~CModuleBase() = default;

  virtual void loadYaml() = 0;
  virtual void update() = 0;

  /**
   * @brief Set the shared RuntimeData for this module
   * @param data Shared pointer to RuntimeData (shared across all modules)
   *
   * All modules in the chain share the same RuntimeData instance.
   * Modules read from and write to different parts of RuntimeData.
   */
  virtual void setRuntimeData(std::shared_ptr<data::RuntimeData> data) {
    runtimeData_ = data;
  }

  /**
   * @brief Initialize module (allocates RuntimeData structures and internal data)
   *
   * Called ONCE after setRuntimeData() and setManagers().
   * All dependencies are valid at this point:
   * - runtimeData_ is set
   * - robotManager, sceneManager, fsmManager are available
   * - pinKine_ is available (if set)
   *
   * Use this to:
   * - Add structures to RuntimeData (e.g., bodyStates, jacobians, bodyPlans)
   * - Allocate internal vectors/matrices
   * - Create sub-controllers, QP solvers, trajectory generators
   * - Initialize kinematics objects
   * - Set up filters, signal generators
   *
   * Do NOT:
   * - Access sensor feedback (not available until first update())
   * - Check initialization flags in update() (defeats the purpose!)
   *
   * RULE: update() should NEVER check if initialization is complete.
   * All data structures MUST be ready when update() is first called.
   */
  virtual void initModule() {
    // Default: no module-specific initialization needed
  }

  void setManagers(RobotManager &robotMgr, SceneManager &sceneMgr, FsmManager &fsmMgr) {
    robotManager = &robotMgr;
    sceneManager = &sceneMgr;
    fsmManager_ = &fsmMgr;
  }

  void setSharedPinKine(std::shared_ptr<utils::PinKine> kine) {
    pinKine_ = kine;
  }

  std::shared_ptr<utils::PinKine> getSharedPinKine() const {
    return pinKine_;
  }

  double getCurrentTime() const noexcept;
  double getDurationSinceLastCall() const noexcept;
  double getDt() const noexcept {
    return updateDt;
  }
  void setDt(double dt) {
    updateDt = dt;
  }
  virtual bool resetModule();

protected:
  YAML::Node _yamlNode;
  RobotManager *robotManager{nullptr};
  SceneManager *sceneManager{nullptr};
  FsmManager *fsmManager_{nullptr};

  std::shared_ptr<utils::PinKine> pinKine_;

  // Shared RuntimeData instance (same pointer across all modules in the chain)
  std::shared_ptr<data::RuntimeData> runtimeData_;

  std::chrono::time_point<std::chrono::steady_clock> startTime;
  mutable std::chrono::time_point<std::chrono::steady_clock> lastCallTime;
  double updateDt{0.001};
};

} // namespace rynn
