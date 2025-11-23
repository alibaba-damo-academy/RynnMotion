#include "fr3_jointmove.hpp"

#include <iostream>

namespace rynn {

Fr3JointMove::Fr3JointMove(const YAML::Node &yamlNode) :
    CJointMove(yamlNode) {
}

#ifdef READ_HDF5_FILE
void Fr3JointMove::_ReadHdf5File() {
  YAML::Node robotParam = YAML::LoadFile(robotManager->getRobotParamPath());

  _replay_dt = robotParam["replay_data_dt"].as<double>();
  _replay_duration = robotParam["replay_duration"].as<double>();

  std::string file_name = robotParam["replay_file"].as<std::string>();
  std::string file_path = MOTION_DIR + file_name; // hdf5 file path for replay
  H5::H5File file(file_path, H5F_ACC_RDONLY);     // 打开 HDF5 文件

  H5::DataSet velocity_dataset = file.openDataSet("/action/arm_desired_joint/velocity");
  H5::DataSet position_dataset = file.openDataSet("/action/arm_desired_joint/position");

  // 获取数据集的维度
  H5::DataSpace velocity_space = velocity_dataset.getSpace();
  H5::DataSpace position_space = position_dataset.getSpace();

  velocity_space.getSimpleExtentDims(_dims_velocity, NULL);
  position_space.getSimpleExtentDims(_dims_position, NULL);

  if (_dims_velocity[1] != _dof || _dims_position[1] != _dof) {
    std::cerr << "Error: Velocity or Position dof is error!" << std::endl;
  }
  if (_dims_velocity[0] != _dims_position[0]) {
    std::cerr << "Error: Velocity and Position length not equral!" << std::endl;
  }

  // 创建数据存储的容器
  _JointPos_des[0].resize(_dims_position[0] * _dims_position[1]);
  _JointVel_des[0].resize(_dims_velocity[0] * _dims_velocity[1]);

  // 读取数据集
  velocity_dataset.read(_JointVel_des[0].data(), H5::PredType::NATIVE_DOUBLE);
  position_dataset.read(_JointPos_des[0].data(), H5::PredType::NATIVE_DOUBLE);
}
#endif

void Fr3JointMove::initModule() {
#ifdef READ_HDF5_FILE
  _JointPos_des.resize(1);
  _JointVel_des.resize(1);
  _JointPos_plan_last.resize(1);
  _JointVel_plan_last.resize(1);
  _ReadHdf5File();
#else
  _JointPos_des.resize(1);
  _JointVel_des.resize(1);
  _JointPos_plan_last.resize(1);
  _JointVel_plan_last.resize(1);
  _JointPos_des[0] = Eigen::VectorXd::Zero(7);
  _JointVel_des[0] = Eigen::VectorXd::Zero(7);
  _JointPos_plan_last[0] = Eigen::VectorXd::Zero(7);
  _JointVel_plan_last[0] = Eigen::VectorXd::Zero(7);
  _plan_t = 0.0;
  _start_t = 0.0;
#endif
}

bool Fr3JointMove::resetModule() {
  _plan_t = 0.0;
  _start_t = runtimeData_->simTime;
  return true;
}

void Fr3JointMove::update() {
  _plan_t = runtimeData_->simTime - _start_t;
  if (_plan_t >= 100.0) {
    _plan_t = 100.0;
  }
  _Fr3JointMotion();
}

void Fr3JointMove::_Fr3JointMotion() {
  double k_ratio = 0.5;
  Eigen::VectorXd kq_gain(7);
  Eigen::VectorXd kd_gain(7);
  kq_gain << 450.0, 450.0, 450.0, 450.0, 200.0, 300.0, 200.0;
  kd_gain << 45.0, 45.0, 45.0, 45.0, 20.0, 30.0, 20.0;

  // Set kp/kd gains - direct vector access
  for (int i = 0; i < 7; i++) {
    runtimeData_->kp[i] = k_ratio * kq_gain(i);
    runtimeData_->kd[i] = k_ratio * kd_gain(i);
  }

  // Prepare command vectors for batch API
  Eigen::VectorXd qCmd(7), qdCmd(7), qtauCmd(7);
  qtauCmd.setZero();

#ifdef READ_HDF5_FILE
  if (_plan_t < _replay_duration) {
    double t = _plan_t / _replay_duration;
    for (int i = 0; i < 7; i++) {
      qCmd(i) = utils::lerp(0.0, _JointPos_des[0](i), t);
      qdCmd(i) = _JointPos_des[0](i) / _replay_duration;
    }
  } else {
    int index = (_plan_t - _replay_duration) / _replay_dt;
    if (index >= _dims_position[0]) {
      index = _dims_position[0] - 1;
    }
    for (int i = 0; i < 7; i++) {
      qCmd(i) = _JointPos_des[0](index * 7 + i);
      qdCmd(i) = _JointVel_des[0](index * 7 + i);
    }
  }
#else
  Eigen::VectorXd q_des(7);
  q_des << 0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0;
  Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(7);
  qCmd = utils::lerp(q_zero, q_des, _plan_t * 0.25);
  qdCmd.setZero();
#endif

  // Set joint commands using batch API
  runtimeData_->setJointsCommand(qCmd, qdCmd, qtauCmd);
}

} // namespace rynn
