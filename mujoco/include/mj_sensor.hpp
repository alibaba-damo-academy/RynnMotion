#pragma once
#include <map>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

#include "image_buffer.hpp"
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
  ~MujocoSensor();

  void initSensors();
  void initAllCameras();
  void update();

  std::map<std::string, data::ImageFrame> captureAllCameras();
  const std::vector<data::CameraConfig> &getCameraConfigs() const;

private:
  void updateFrameSensor();
  void updateFTSensor();
  void updateRawIMU();
  void updateMultiRay();

  MujocoInterface &mj_;
  rynn::RobotType robotType_;

  std::vector<int> framePosIndices_;
  std::vector<int> frameQuatIndices_;
  std::vector<int> frameLinVelIndices_;
  std::vector<int> frameAngVelIndices_;
  std::vector<int> frameLinAccIndices_;
  std::vector<int> frameAngAccIndices_;
  std::vector<int> gyroIndices_;
  std::vector<int> velocimeterIndices_;
  std::vector<int> accelerometerIndices_;
  std::vector<int> forceIndices_;
  std::vector<int> torqueIndices_;

  std::vector<data::CameraConfig> cameraConfigs_;
  std::map<std::string, mjvScene> cameraScenes_;
  std::map<std::string, mjvCamera> cameraCameras_;
};

} // namespace mujoco
