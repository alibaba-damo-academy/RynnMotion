#pragma once
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

#include "mj_interface.hpp"
#ifdef __APPLE__
#include "mujoco.h"
#else
#include "mujoco/mujoco.h"
#endif

namespace mujoco {

class MujocoInterface;

class MujocoSensor {
public:
  MujocoSensor(MujocoInterface &mj);

  /**
   * Initialize sensors by detecting available types in MuJoCo model
   */
  void initSensors();

  /**
   * Initialize RGB camera in mujoco
   */
  void initRGB();

  /**
   * Main entry point to update all sensor data from MuJoCo
   */
  void update();

  /**
   * Update RGB image, calling from mj_interface.cpp
   */
  void updateRGB();

private:
  /**
   * Update frame sensors
   */
  void updateFrameSensor();

  /**
   * Update force and torque sensors (FT sensors)
   */
  void updateFTSensor();

  /**
   * Update gyro and velocimeter sensors
   */
  void updateRawIMU();

  /**
   * Update updateMultiRay that provide distance measurements
   */
  void updateMultiRay();

  /**
   * Update sites information in mujoco
   */
  void updateSites();

private:
  MujocoInterface &mj_;
  rynn::RobotType robotType_;
  mjvScene mjSceneFrontCamera_;
  mjvCamera frontCamera_;

  std::vector<int> framePosIndices_;      // mjSENS_FRAMEPOS
  std::vector<int> frameQuatIndices_;     // mjSENS_FRAMEQUAT
  std::vector<int> frameLinVelIndices_;   // mjSENS_FRAMELINVEL
  std::vector<int> frameAngVelIndices_;   // mjSENS_FRAMEANGVEL
  std::vector<int> frameLinAccIndices_;   // mjSENS_FRAMELINACC
  std::vector<int> frameAngAccIndices_;   // mjSENS_FRAMEANGACC
  std::vector<int> gyroIndices_;          // mjSENS_GYRO
  std::vector<int> velocimeterIndices_;   // mjSENS_VELOCIMETER
  std::vector<int> accelerometerIndices_; // mjSENS_ACCELEROMETER
  std::vector<int> forceIndices_;         // mjSENS_FORCE
  std::vector<int> torqueIndices_;        // mjSENS_TORQUE
};

} // namespace mujoco
