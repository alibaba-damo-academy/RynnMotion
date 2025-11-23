#include "fr3_teleopfollow.hpp"

namespace rynn {

Fr3TeleopFollow::Fr3TeleopFollow(const YAML::Node &yamlNode) :
    TeleopFollow(yamlNode) {
}

void Fr3TeleopFollow::initModule() {
  TeleopFollow::initModule();

  int mdof = robotManager->getMotionDOF();
  _qFb = Eigen::VectorXd::Zero(mdof);
  _qdFb = Eigen::VectorXd::Zero(mdof);
  _qCmd = Eigen::VectorXd::Zero(mdof);
  _qCmd_last = Eigen::VectorXd::Zero(mdof);
  _qdCmd = Eigen::VectorXd::Zero(mdof);
  _qddCmd = Eigen::VectorXd::Zero(mdof);
  _qdCmd_last = Eigen::VectorXd::Zero(mdof);
  _qddCmd_last = Eigen::VectorXd::Zero(mdof);
  _eePos_teleop = Eigen::Vector3d::Zero();
  _eePos_state_fb = Eigen::Vector3d::Zero();
  _eePos_des_last = Eigen::Vector3d::Zero();
  _eePos_des = Eigen::Vector3d::Zero();
  _eeQuat_teleop = Eigen::Quaterniond::Identity();
  _eeQuat_state_fb = Eigen::Quaterniond::Identity();
  _eeVel_des = Eigen::VectorXd::Zero(6);
  _eeVel_plan_last = Eigen::VectorXd::Zero(6);
  _eeVel_des_last = Eigen::VectorXd::Zero(6);
  _eeVel_fb = Eigen::VectorXd::Zero(6);
  _eeJacoFb = Eigen::MatrixXd::Zero(6, mdof);
  _qTask = Eigen::VectorXd::Zero(mdof);
  _qTarget = Eigen::VectorXd::Zero(mdof);
  _qTarget_last = Eigen::VectorXd::Zero(mdof);
  _qdTarget = Eigen::VectorXd::Zero(mdof);
  _qdTarget_last = Eigen::VectorXd::Zero(mdof);
  _qddTarget = Eigen::VectorXd::Zero(mdof);
  _qddTarget_last = Eigen::VectorXd::Zero(mdof);

  _ruckig_otg.delta_time = _dt;

  _initFr3State();
}

void Fr3TeleopFollow::_initFr3State() {
  _follow_mode = _yamlNode["module"]["teleopfollow"]["follow_type"].as<int>();
  _safeJointRef.resize(7);
  _IkJointRef.resize(7);
  _jointLimitLower.resize(7);
  _jointLimitUpper.resize(7);
  _jointVelLimit.resize(7);
  _jointAccLimit.resize(7);
  _jointJerkLimit.resize(7);
  _kpGainInner.resize(7);
  _kdGainInner.resize(7);

  double k_ratio = 1.0;
  _kpGainInner << 450.0, 450.0, 450.0, 450.0, 200.0, 300.0, 200.0;
  _kdGainInner << 45.0, 45.0, 45.0, 45.0, 20.0, 30.0, 20.0;
  _kpGainInner = _kpGainInner * k_ratio;
  _kdGainInner = _kdGainInner * k_ratio;

  _safeJointRef << 0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0;

  _jointLimitLower = robotManager->getJointPosMin();
  _jointLimitUpper = robotManager->getJointPosMax();
  _jointVelLimit = robotManager->getJointVelMax();

  _IkJointRef = _safeJointRef;

  _jointAccLimit = Eigen::VectorXd::Constant(7, 62.8);
  _jointJerkLimit = Eigen::VectorXd::Constant(7, 628.0);

  _first_start = true;

  _lambda = _yamlNode["module"]["teleopfollow"]["dls_lambda"].as<double>();
  _qd_effector = Eigen::MatrixXd::Identity(7, 7) * _lambda;
  _qd_effector(1, 1) *= 0.25;
  _qd_effector(2, 2) *= 9.0;
  _qd_effector(3, 3) *= 0.25;

  double qref_l = _yamlNode["module"]["teleopfollow"]["qref_lambda"].as<double>() / _dt;
  _qref_effector = Eigen::MatrixXd::Identity(7, 7) * qref_l;
  _qref_effector(1, 1) *= 1.5;
  _qref_effector(2, 2) *= 5.0;
  _qref_effector(3, 3) *= 1.5;

  double ratio = 0.1;
  _cart_acc_limit = 10.0 * ratio * _dt;
  _cart_vel_limit = 1.0 * ratio;
  _jointVelLimit *= ratio;
  _jointAccLimit *= ratio;
  _jointJerkLimit *= ratio;

  velocity_filter_ = std::make_shared<utils::SecondLowPass>(20.0, 0.002, 0.86, 7);

  _ruckig_input.max_velocity = {_jointVelLimit(0), _jointVelLimit(1), _jointVelLimit(2),
                                _jointVelLimit(3), _jointVelLimit(4), _jointVelLimit(5), _jointVelLimit(6)};
  _ruckig_input.max_acceleration = {_jointAccLimit(0), _jointAccLimit(1), _jointAccLimit(2), _jointAccLimit(3),
                                    _jointAccLimit(4), _jointAccLimit(5), _jointAccLimit(6)};
  _ruckig_input.max_jerk = {_jointJerkLimit(0), _jointJerkLimit(1), _jointJerkLimit(2),
                            _jointJerkLimit(3), _jointJerkLimit(4), _jointJerkLimit(5), _jointJerkLimit(6)};
}

bool Fr3TeleopFollow::setTeleopPose(const Eigen::Vector3d &opPos, const Eigen::Quaterniond &opQuat) {
  // TODO: input cmd check
  _eePos_teleop = opPos;
  _eeQuat_teleop = opQuat;
  return true;
}

bool Fr3TeleopFollow::setJointStateFb(const Eigen::VectorXd &JointState) {
  // TODO: input joint state check
  _qFb = JointState;
  _pinKine->update(_qFb);
  _eePos_state_fb = _pinKine->getEEPos();
  // Convert Vector4d (x,y,z,w) to Quaterniond (w,x,y,z)
  Eigen::Vector4d eeQuatVec = _pinKine->getEEQuat();
  _eeQuat_state_fb = Eigen::Quaterniond(eeQuatVec(3), eeQuatVec(0), eeQuatVec(1), eeQuatVec(2));
  _eeJacoFb = _pinKine->getEEJaco();
  return true;
}

bool Fr3TeleopFollow::resetModule() {
  _first_start = true;
  return true;
}

void Fr3TeleopFollow::_setSimPlanPose() {
  _eePos_teleop = runtimeData_->bodyPlans[0].pos;
  _eeQuat_teleop = runtimeData_->bodyPlans[0].quat;
}

void Fr3TeleopFollow::_setSimStateFb() {
  Eigen::VectorXd qtauFb_unused;
  runtimeData_->getJointsFeedback(_qFb, _qdFb, qtauFb_unused);
  _eeJacoFb = runtimeData_->jacobians[0].jaco;
  _eePos_state_fb = runtimeData_->bodyStates[0].pos;
  _eeQuat_state_fb = runtimeData_->bodyStates[0].quat;
}

bool Fr3TeleopFollow::_jointLimitSaturation(Eigen::VectorXd &joint_in) {
  bool move_limit = false;
  for (int i = 0; i < 7; i++) {
    if (joint_in(i) > _jointLimitUpper(i) || joint_in(i) < _jointLimitLower(i)) {
      move_limit = true;
      joint_in(i) = std::max(std::min(joint_in(i), _jointLimitUpper(i)), _jointLimitLower(i));
    }
  }
  return move_limit;
}

void Fr3TeleopFollow::_initFirstRunState() {
  // limit desired pose
  if (_first_start) {
    _eePos_des_last = _eePos_state_fb;
    _eeVel_plan_last.setZero();
    _eeVel_des_last.setZero();

    _qCmd_last = _qFb;
    _qdCmd_last.setZero();
    _qddCmd_last.setZero();
    // init target
    _qTarget = _pinKine->ikPos(_eePos_teleop, _eeQuat_teleop, _safeJointRef);
    _qTarget_last = _qTarget;
    _qdTarget.setZero();
    _qdTarget_last.setZero();
    _qddTarget.setZero();
    _qddTarget_last.setZero();

    //
    _ruckig_input.current_position = {_qCmd_last(0), _qCmd_last(1), _qCmd_last(2), _qCmd_last(3),
                                      _qCmd_last(4), _qCmd_last(5), _qCmd_last(6)};
    _ruckig_input.current_velocity = {_qdCmd_last(0), _qdCmd_last(1), _qdCmd_last(2), _qdCmd_last(3),
                                      _qdCmd_last(4), _qdCmd_last(5), _qdCmd_last(6)};
    _ruckig_input.current_acceleration = {_qddCmd_last(0), _qddCmd_last(1), _qddCmd_last(2), _qddCmd_last(3),
                                          _qddCmd_last(4), _qddCmd_last(5), _qddCmd_last(6)};
    _first_start = false;
  } else {
    _eeVel_des_last = _eeVel_des;
  }
}

void Fr3TeleopFollow::_CartVellimitFollowFilter() {
  // desired pose follow
  Eigen::Vector3d pos_cmd_delta = _eePos_teleop - _eePos_des_last;
  for (int i = 0; i < 3; i++) {
    double vel_ref = pos_cmd_delta[i] / _dt; // delta v in dt
    double vel_ref_max = _eeVel_plan_last(i) + _cart_acc_limit;
    double vel_ref_min = _eeVel_plan_last(i) - _cart_acc_limit;
    vel_ref = std::max(std::min(vel_ref, vel_ref_max), vel_ref_min);
    vel_ref = std::max(std::min(vel_ref, _cart_vel_limit), -1.0 * _cart_vel_limit);
    _eePos_des(i) = _eePos_des_last(i) + vel_ref * _dt;
    _eeVel_plan_last(i) = vel_ref;
    _eePos_des_last(i) = _eePos_des(i);
  }
}

void Fr3TeleopFollow::_qdotLimitFollow() {
  Eigen::VectorXd qd_ref = (_qCmd - _qCmd_last) / _dt;
  for (int i = 0; i < 7; i++) {
    double q_ref_max = _qdCmd_last(i) + _jointAccLimit[i] * _dt;
    double q_ref_min = _qdCmd_last(i) - _jointAccLimit[i] * _dt;
    qd_ref[i] = std::max(std::min(qd_ref[i], q_ref_max), q_ref_min);
    qd_ref[i] = std::max(std::min(qd_ref[i], _jointVelLimit[i]), -1.0 * _jointVelLimit[i]);
  }
  _qCmd = _qCmd_last + qd_ref * _dt;
  _qdCmd = qd_ref;
  _qdCmd_last = qd_ref;
}

void Fr3TeleopFollow::_RuckigMotionFollow() {
  // Set input parameters
  //_ruckig_input.current_position = {_qCmd_last(0), _qCmd_last(1), _qCmd_last(2), _qCmd_last(3),
  //                                  _qCmd_last(4), _qCmd_last(5), _qCmd_last(6)};
  //_ruckig_input.current_velocity = {_qdCmd_last(0), _qdCmd_last(1), _qdCmd_last(2), _qdCmd_last(3),
  //                                  _qdCmd_last(4), _qdCmd_last(5), _qdCmd_last(6)};
  //_ruckig_input.current_acceleration = {_qddCmd_last(0), _qddCmd_last(1), _qddCmd_last(2), _qddCmd_last(3),
  //                                      _qddCmd_last(4), _qddCmd_last(5), _qddCmd_last(6)};

  _ruckig_input.target_position = {_qTarget(0), _qTarget(1), _qTarget(2), _qTarget(3),
                                   _qTarget(4), _qTarget(5), _qTarget(6)};
  _ruckig_input.target_velocity = {_qdTarget(0), _qdTarget(1), _qdTarget(2), _qdTarget(3),
                                   _qdTarget(4), _qdTarget(5), _qdTarget(6)};
  _ruckig_input.target_acceleration = {_qddTarget(0), _qddTarget(1), _qddTarget(2), _qddTarget(3),
                                       _qddTarget(4), _qddTarget(5), _qddTarget(6)};

  auto result = _ruckig_otg.update(_ruckig_input, _ruckig_output);
  if (result == ruckig::Result::Working) {
    for (int i = 0; i < 7; i++) {
      _qCmd(i) = _ruckig_output.new_position.at(i);
      _qdCmd(i) = _ruckig_output.new_velocity.at(i);
      _qddCmd(i) = _ruckig_output.new_acceleration.at(i);
    }
    _ruckig_output.pass_to_input(_ruckig_input);
  } else {
    _qCmd = _qCmd_last;
    _qdCmd = _qdCmd_last;
    _qddCmd = _qddCmd_last;
    // if (_qdTarget.norm() > 1e-6) {
    //   std::cout << "Ruckig motion follow error " << _qdTarget.norm() << " " << result << std::endl;
    // }
  }
}

void Fr3TeleopFollow::_DLSDiffIkCalculation() {
  Eigen::Vector3d posError = _eePos_teleop - _eePos_state_fb;
  Eigen::Quaterniond quat_err = _eeQuat_teleop * _eeQuat_state_fb.inverse();
  Eigen::AngleAxisd angleAxisErr(quat_err);
  Eigen::Vector3d orientError = angleAxisErr.angle() * angleAxisErr.axis();
  Eigen::VectorXd error(6);
  error.head<3>() = posError;
  error.tail<3>() = orientError;
  // Convert position error to velocity
  if (_dt <= 1e-10) {
    _eeVel_des.setZero();
  } else {
    _eeVel_des = error / _dt;
  }

  Eigen::MatrixXd cod = _eeJacoFb.transpose() * _eeJacoFb;
  cod = cod + _qd_effector;
  cod = cod.inverse();
  Eigen::MatrixXd J_pseudo_inv = cod * _eeJacoFb.transpose();
  //_qdCmd = J_pseudo_inv * _eeVel_des;
  //_qCmd = _qFb + _qdCmd * _dt;
  Eigen::VectorXd qfb_step = J_pseudo_inv * _eeVel_des;
  _qCmd = _qFb + qfb_step * _dt; //_qdCmd * _dt;
  _qdCmd = (_qCmd - _qCmd_last) / _dt;
  _qCmd_last = _qCmd;
}

void Fr3TeleopFollow::_DLSVelLimitDiffIkCalculation() {
  _CartVellimitFollowFilter();

  Eigen::Vector3d posError = _eePos_des - _eePos_state_fb; //runtimeData_->bodyFeedbacks[0].pos;
  Eigen::Quaterniond quat_err = _eeQuat_teleop * _eeQuat_state_fb.inverse();
  Eigen::AngleAxisd angleAxisErr(quat_err);
  // if (angleAxisErr.angle() > 0.05) {
  //   angleAxisErr.angle() = 0.05;
  // }
  Eigen::Vector3d orientError = angleAxisErr.angle() * angleAxisErr.axis();
  Eigen::VectorXd error(6);
  error.head<3>() = posError;
  error.tail<3>() = orientError;

  // Convert position error to velocity
  if (_dt <= 1e-10) {
    _eeVel_des.setZero();
  } else {
    _eeVel_des = error / _dt;
  }

  Eigen::MatrixXd cod = _eeJacoFb.transpose() * _eeJacoFb;
  cod = cod + _qd_effector;
  cod = cod.inverse();
  Eigen::MatrixXd J_pseudo_inv = cod * _eeJacoFb.transpose();
  Eigen::VectorXd qref_err = cod * (_safeJointRef - _qFb);
  Eigen::VectorXd qfb_step = J_pseudo_inv * _eeVel_des + _qref_effector * qref_err;
  //_qdCmd = J_pseudo_inv * _eeVel_des + _qref_effector * qref_err;
  qfb_step = velocity_filter_->filtering(qfb_step);
  _qCmd = _qFb + qfb_step * _dt; //_qdCmd * _dt;
  _qdCmd = (_qCmd - _qCmd_last) / _dt;
  _qCmd_last = _qCmd;
}

void Fr3TeleopFollow::_DlsQrefDiffIkVlimitCalculation() {
  Eigen::Vector3d posError = _eePos_teleop - _eePos_state_fb;
  Eigen::Quaterniond quat_err = _eeQuat_teleop * _eeQuat_state_fb.inverse();
  Eigen::AngleAxisd angleAxisErr(quat_err);
  Eigen::Vector3d orientError = angleAxisErr.angle() * angleAxisErr.axis();
  Eigen::VectorXd error(6);
  error.head<3>() = posError;
  error.tail<3>() = orientError;
  // Convert position error to velocity
  if (_dt <= 1e-10) {
    _eeVel_des.setZero();
  } else {
    _eeVel_des = error / _dt;
  }

  Eigen::MatrixXd cod = _eeJacoFb.transpose() * _eeJacoFb;
  cod = cod + _qd_effector;
  cod = cod.inverse();
  Eigen::MatrixXd J_pseudo_inv = cod * _eeJacoFb.transpose();
  Eigen::VectorXd qref_err = cod * (_safeJointRef - _qFb);
  Eigen::VectorXd qfb_step = J_pseudo_inv * _eeVel_des
                             + _qref_effector * qref_err;
  _qTarget = _qFb + qfb_step * _dt;
  _qdTarget = (_qTarget - _qTarget_last) / _dt;
  _qddTarget = (_qdTarget - _qdTarget_last) / _dt;
  bool move_limit = _jointLimitSaturation(_qTarget);
  if (move_limit) {
    _qdTarget.setZero();
    _qddTarget.setZero();
  }
  _RuckigMotionFollow();
  _qCmd_last = _qCmd;
  _qdCmd_last = _qdCmd;
  _qddCmd_last = _qddCmd;
  _qTarget_last = _qTarget;
  _qdTarget_last = _qdTarget;
  _qddTarget_last = _qddTarget;
}

void Fr3TeleopFollow::_DlsQrefDiffIkCalculation() {
  Eigen::Vector3d posError = _eePos_teleop - _eePos_state_fb;
  Eigen::Quaterniond quat_err = _eeQuat_teleop * _eeQuat_state_fb.inverse();
  Eigen::AngleAxisd angleAxisErr(quat_err);
  Eigen::Vector3d orientError = angleAxisErr.angle() * angleAxisErr.axis();
  Eigen::VectorXd error(6);
  error.head<3>() = posError;
  error.tail<3>() = orientError;
  // Convert position error to velocity
  if (_dt <= 1e-10) {
    _eeVel_des.setZero();
  } else {
    _eeVel_des = error / _dt;
  }

  Eigen::VectorXd q_target = Eigen::VectorXd::Zero(7);
  q_target = _pinKine->ikPos(_eePos_teleop, _eeQuat_teleop, _qCmd);

  Eigen::MatrixXd cod = _eeJacoFb.transpose() * _eeJacoFb;
  cod = cod + _qd_effector;
  cod = cod.inverse();
  Eigen::MatrixXd J_pseudo_inv = cod * _eeJacoFb.transpose();
  Eigen::VectorXd qtgt_err = cod * (q_target - _qFb);
  Eigen::VectorXd qref_err = cod * (_safeJointRef - _qFb);
  Eigen::VectorXd qfb_step = J_pseudo_inv * _eeVel_des
                             + _qref_effector * qref_err
                             + 1.0 * _qref_effector * qtgt_err;
  _qCmd = _qFb + qfb_step * _dt; //_qdCmd * _dt;
  _qdCmd = (_qCmd - _qCmd_last) / _dt;
  _qCmd_last = _qCmd;
  _qdCmd_last = _qdCmd;
}

void Fr3TeleopFollow::_DlsQrefCalculation() {
  Eigen::Vector3d posError = _eePos_teleop - _eePos_state_fb;
  Eigen::Quaterniond quat_err = _eeQuat_teleop * _eeQuat_state_fb.inverse();
  Eigen::AngleAxisd angleAxisErr(quat_err);
  Eigen::Vector3d orientError = angleAxisErr.angle() * angleAxisErr.axis();
  Eigen::VectorXd error(6);
  error.head<3>() = posError;
  error.tail<3>() = orientError;
  // Convert position error to velocity
  if (_dt <= 1e-10) {
    _eeVel_des.setZero();
  } else {
    _eeVel_des = error / _dt;
  }

  Eigen::MatrixXd cod = _eeJacoFb.transpose() * _eeJacoFb;
  cod = cod + _qd_effector;
  cod = cod.inverse();
  Eigen::MatrixXd J_pseudo_inv = cod * _eeJacoFb.transpose();
  Eigen::VectorXd qref_err = cod * (_safeJointRef - _qFb);
  Eigen::VectorXd qfb_step = J_pseudo_inv * _eeVel_des
                             + _qref_effector * qref_err;
  _IkJointRef = _qFb + qfb_step * _dt;
}

void Fr3TeleopFollow::_IkJointRefCalculation() {
  // Eigen::VectorXd q_ref = _safeJointRef;
  // q_ref(0) = _qCmd(0);
  _DlsQrefCalculation();
  _qTarget = _pinKine->ikPos(_eePos_teleop, _eeQuat_teleop, _IkJointRef);
  _qdTarget = (_qTarget - _qTarget_last) / _dt;
  _qddTarget = (_qdTarget - _qdTarget_last) / _dt;
  bool move_limit = _jointLimitSaturation(_qTarget);
  if (move_limit) {
    _qdTarget.setZero();
    _qddTarget.setZero();
  }
  for (int i = 0; i < 7; i++) {
    _qdTarget(i) = std::max(std::min(_qdTarget(i), _jointVelLimit(i)), -_jointVelLimit(i));
    _qddTarget(i) = std::max(std::min(_qdTarget(i), _jointAccLimit(i)), -_jointAccLimit(i));
  }

  _RuckigMotionFollow();

  _qCmd_last = _qCmd;
  _qdCmd_last = _qdCmd;
  _qddCmd_last = _qddCmd;
  _qTarget_last = _qTarget;
  _qdTarget_last = _qdTarget;
  _qddTarget_last = _qddTarget;
}

bool Fr3TeleopFollow::_CollisionCheck() {
  // TODO: collision check
  return false;
}

void Fr3TeleopFollow::_taskTeleopFollow() {
  _setSimPlanPose();
  _setSimStateFb();

  _initFirstRunState();

  switch (_follow_mode) {
  case 0:
    _IkJointRefCalculation();
    break;

  case 1:
    _DlsQrefDiffIkVlimitCalculation();
    break;

  case 2:
    _DlsQrefDiffIkCalculation();
    break;

  case 3:
    _DLSVelLimitDiffIkCalculation();
    break;

  case 4:
    _DLSDiffIkCalculation();
    break;

  default:
    _IkJointRefCalculation();
    break;
  }

  //_jointLimitSaturation(_qCmd);

  // Set joint commands using batch API
  Eigen::VectorXd qtauCmd = Eigen::VectorXd::Zero(7);
  runtimeData_->setJointsCommand(_qCmd, _qdCmd, qtauCmd);

  // Note: kp/kd gains - direct vector access
  for (int i = 0; i < 7; ++i) {
    runtimeData_->kp[i] = _kpGainInner(i);
    runtimeData_->kd[i] = _kdGainInner(i);
  }

  // if (_t < 2.0) {
  //   for (int i = 0; i < 7; ++i) {
  //     runtimeData_->motorCommand(i).qCmd = _safeJointRef[i] * _t / 2.0;
  //   }
  // } else {
  //   for (int i = 0; i < 7; ++i) {
  //     runtimeData_->motorCommand(i).qCmd = _qCmd(i);
  //     runtimeData_->motorCommand(i).qdCmd = _qdCmd(i);
  //   }
  // }
}

} // namespace rynn
