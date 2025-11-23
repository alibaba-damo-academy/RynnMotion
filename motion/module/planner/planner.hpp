#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "runtime_data.hpp"
#include "interpolate.hpp"
#include "kine_pin.hpp"
#include "module_base.hpp"
#include "motion_fsm.hpp"
#include "pickplace_controller.hpp"
#include "scene_manager.hpp"

namespace rynn {

/**
 * @class CPlanner
 * @brief Multi-end-effector trajectory planner
 *
 * Reads:  qFb (joint feedback), objectStates[] (scene objects), simTime
 * Writes: bodyPlans[] (desired end-effector poses), gripperCommands[]
 *
 * Generates trajectories based on scene type:
 * - kUI: Follow objects with teleoperation
 * - kTracking: Predefined circular motions
 * - kPickPlace: Automated pick-and-place sequences
 * - kDefault: Simple joint motions
 *
 * Supports single-arm, dual-arm, and N-arm robots with automatic EE detection.
 */
class CPlanner : public CModuleBase {
public:
  explicit CPlanner(const YAML::Node &yamlNode);
  ~CPlanner() override = default;

  void loadYaml() override;
  void update() override;
  void initModule() override;
  bool resetModule() override;

protected:
  // Inherits: std::shared_ptr<data::RuntimeData> runtimeData_;
  double _t{0.};
  double freq_{0.2};
  Eigen::VectorXd qStand_;
  Eigen::VectorXd qInit_;

  int _numEE{1};
  int _sceneNumber{-1};
  SceneType _sceneType{SceneType::kDefault};

  struct CircleMotionState {
    Eigen::Vector3d center;
    double phaseOffset;
    bool initialized{false};
  };
  std::vector<CircleMotionState> _circleStates;

  // Dual-arm pick-place controllers (support up to 2 arms)
  std::unique_ptr<PickPlaceController> _pickPlaceControllers[2];
  double _lastPickPlaceUpdateTime[2]{0.0, 0.0};
  bool _pickPlaceInitialized[2]{false, false};

  virtual void setStandPos();
  virtual void move2Stand();
  virtual void move2Home();
  virtual void eePlanner();

  void followObject();
  void predefinedMotion();
  void pickAndPlace();
  void pickAndPlaceSingleArm();
  std::vector<data::Pose> getOfflineGraspPose(const data::Pose &eePose);
  std::pair<std::vector<data::Pose>, std::vector<data::Pose>>
  assignCubesToArms(const std::vector<data::Pose> &allCubes,
                    const Eigen::Vector3d &leftArmPos,
                    const Eigen::Vector3d &rightArmPos);

private:
  void twoLinkPlanner();
};

} // namespace rynn
