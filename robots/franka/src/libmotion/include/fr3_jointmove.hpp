#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "runtime_data.hpp"
#include "interpolate.hpp"
#include "joint_move.hpp"

#ifdef READ_HDF5_FILE
#include <H5Cpp.h>
#endif

namespace rynn {

/**
 * @class Fr3JointMove
 * @brief robot joint move module, used for testing models
 */
class Fr3JointMove : public CJointMove {
public:
  explicit Fr3JointMove(const YAML::Node &yamlNode);
  ~Fr3JointMove() override = default;

  void update() override;
  void initModule() override;
  bool resetModule() override;

private:
  void _Fr3JointMotion();

  double _start_t;
  double _plan_t;
  std::vector<Eigen::VectorXd> _JointPos_des;
  std::vector<Eigen::VectorXd> _JointVel_des;
  std::vector<Eigen::VectorXd> _JointPos_plan_last;
  std::vector<Eigen::VectorXd> _JointVel_plan_last;

#ifdef READ_HDF5_FILE
  void _ReadHdf5File();
  hsize_t _dims_position[2];
  hsize_t _dims_velocity[2];
  double _replay_dt;
  double _replay_duration;
  // std::vector<double> _JointPos_des_data;
  // std::vector<double> _JointVel_des_data;
#endif
};

} // namespace rynn