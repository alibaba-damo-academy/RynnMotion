#pragma once

#include <ruckig/ruckig.hpp>

#include "filter.hpp"
#include "teleop_follow.hpp"

namespace rynn {

class Fr3TeleopFollow : public TeleopFollow {
public:
  explicit Fr3TeleopFollow(const YAML::Node &yamlNode);
  ~Fr3TeleopFollow() override = default;

  void initModule() override;
  bool setTeleopPose(const Eigen::Vector3d &opPos, const Eigen::Quaterniond &opQuat);
  bool setJointStateFb(const Eigen::VectorXd &JointState);
  bool resetModule() override;

protected:
  void _taskTeleopFollow() override;
  void _DLSDiffIkCalculation();
  void _DLSVelLimitDiffIkCalculation();
  void _DlsQrefDiffIkCalculation();
  void _IkJointRefCalculation();
  void _DlsQrefDiffIkVlimitCalculation();

private:
  void _initFr3State();
  void _initFirstRunState();
  void _CartVellimitFollowFilter();
  void _setSimPlanPose();
  void _setSimStateFb();
  bool _CollisionCheck();
  void _DlsQrefCalculation();
  void _qdotLimitFollow();
  void _RuckigMotionFollow();
  bool _jointLimitSaturation(Eigen::VectorXd &joint_in);

  bool _first_start;
  int _follow_mode{0};
  Eigen::VectorXd _safeJointRef;
  Eigen::VectorXd _IkJointRef;
  // std::unique_ptr<utils::DiffIKQP> _dIKqp;
  Eigen::VectorXd _qddCmd;
  Eigen::VectorXd _qCmd_last, _qdCmd_last, _qddCmd_last;
  Eigen::VectorXd _qTarget, _qTarget_last, _qdTarget, _qdTarget_last, _qddTarget, _qddTarget_last;
  Eigen::VectorXd _eeVel_plan_last, _eeVel_des_last;
  Eigen::Vector3d _eePos_des, _eePos_des_last;
  // std::vector<Eigen::Quaterniond> _eeQuat_des, _eeQuat_des_last;
  Eigen::Vector3d _eePos_teleop, _eePos_state_fb;
  Eigen::Quaterniond _eeQuat_teleop, _eeQuat_state_fb;

  std::shared_ptr<utils::SecondLowPass> velocity_filter_;

  // alg parameters
  double _lambda;
  Eigen::MatrixXd _qd_effector;
  Eigen::MatrixXd _qref_effector;
  double _cart_vel_limit;
  double _cart_acc_limit;
  // Joint limits stored as Eigen vectors
  Eigen::VectorXd _jointLimitLower;
  Eigen::VectorXd _jointLimitUpper;
  Eigen::VectorXd _jointVelLimit;
  Eigen::VectorXd _jointAccLimit;
  Eigen::VectorXd _jointJerkLimit;
  Eigen::VectorXd _kpGainInner;
  Eigen::VectorXd _kdGainInner;

  ruckig::Ruckig<7> _ruckig_otg;
  ruckig::InputParameter<7> _ruckig_input;
  ruckig::OutputParameter<7> _ruckig_output;
};

} // namespace rynn
