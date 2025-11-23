#include "lcm_logger.hpp"

#include <iostream>

namespace utils {

LcmLogger::LcmLogger(lcmMotion::lcm_record &lcmStruct) :
    lcmStruct(lcmStruct) {
  lcmStruct.counter = 0;
}

void LcmLogger::writeLogData(std::shared_ptr<data::RuntimeData> data) {
  if (!data) {
    std::cerr << "Error: Null data pointer passed to writeLogData" << std::endl;
    return;
  }

  // Update basic counters and timing
  lcmStruct.simTime = static_cast<float>(data->simTime);
  lcmStruct.wallTime = static_cast<float>(data->wallTime);

  if (lcmStruct.counter == 0) {
    lcmStruct.duration = 0.001;
  } else {
    lcmStruct.duration = static_cast<float>(data->duration);
  }

  lcmStruct.counter++;

  // Get motor feedback and commands - these are necessary for all robots
  Eigen::VectorXd qFb, qdFb, qtauFb;
  data->getJointsFeedback(qFb, qdFb, qtauFb);

  Eigen::VectorXd qCmd, qdCmd, qtauCmd;
  data->getJointsCommand(qCmd, qdCmd, qtauCmd);

  // Write motor data to LCM struct
  if (qFb.size() > 0 && qFb.size() <= 20) {
    Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.qFb).head(qFb.size()) = qFb.cast<float>();
    Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.qdFb).head(qdFb.size()) = qdFb.cast<float>();
    Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.qtauFb).head(qtauFb.size()) = qtauFb.cast<float>();
    Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.qCmd).head(qCmd.size()) = qCmd.cast<float>();
    Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.qdCmd).head(qdCmd.size()) = qdCmd.cast<float>();
    Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.qtauCmd).head(qtauCmd.size()) = qtauCmd.cast<float>();
  } else {
    std::cerr << "Warning: Motor data size out of range" << std::endl;
  }

  // Handle planned state data if available
  if (data->bodyPlans.size() > 0) {
    try {
      const auto &plannedState = data->bodyPlans[0];
      Eigen::Vector3d eePosDes = plannedState.pos;
      Eigen::Quaterniond eeQuatDes = plannedState.quat;
      Eigen::Vector3d eeRPYDes = quatToRPY(eeQuatDes.coeffs());
      Eigen::VectorXd eeVelDes = plannedState.velocity;

      Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.eeXDes).head(3) = eePosDes.cast<float>();
      Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.eeXDes).segment<3>(3) = eeRPYDes.cast<float>();
      Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.eeXdotDes).head(6) = eeVelDes.cast<float>();
    } catch (const std::exception &e) {
      std::cerr << "Warning: Error processing planned state data: " << e.what() << std::endl;
    }
  }

  // Handle link state feedback data if available (from estimator)
  if (data->bodyStates.size() > 0) {
    try {
      const auto &linkState = data->bodyStates[0];
      Eigen::Vector3d eePosFb = linkState.pos;
      Eigen::Vector3d eeRPYFb = quatToRPY(linkState.quat.coeffs());
      Eigen::VectorXd eeVelFb = linkState.velocity;

      Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.eeXFb).head(3) = eePosFb.cast<float>();
      Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.eeXFb).segment<3>(3) = eeRPYFb.cast<float>();
      Eigen::Map<Eigen::Array<float, 20, 1>>(lcmStruct.eeXdotFb).head(eeVelFb.size()) = eeVelFb.cast<float>();
    } catch (const std::exception &e) {
      // std::cerr << "Warning: Link state data not available" << std::endl;
    }
  }

  // Note: F/T sensor data is not copied to RuntimeData - stays in MuJoCo data structures
  // Set to zero for now
  Eigen::Map<Eigen::Array<float, 3, 1>>(lcmStruct.ftForce).setZero();
  Eigen::Map<Eigen::Array<float, 3, 1>>(lcmStruct.ftTorque).setZero();
}

} // namespace utils
