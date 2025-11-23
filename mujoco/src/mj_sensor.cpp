#include "mj_sensor.hpp"

namespace mujoco {

MujocoSensor::MujocoSensor(MujocoInterface &mj) :
    mj_(mj),
    robotType_(mj.robotManager->getRobotType()) {
}

void MujocoSensor::initSensors() {
  for (int i = 0; i < mj_.mjModel_->nsensor; i++) {
    auto sensorType = mj_.mjModel_->sensor_type[i];
    switch (sensorType) {
    case mjSENS_FRAMEPOS:
      framePosIndices_.push_back(i);
      break;
    case mjSENS_FRAMEQUAT:
      frameQuatIndices_.push_back(i);
      break;
    case mjSENS_FRAMELINVEL:
      frameLinVelIndices_.push_back(i);
      break;
    case mjSENS_FRAMEANGVEL:
      frameAngVelIndices_.push_back(i);
      break;
    case mjSENS_FRAMELINACC:
      frameLinAccIndices_.push_back(i);
      break;
    case mjSENS_FRAMEANGACC:
      frameAngAccIndices_.push_back(i);
      break;
    case mjSENS_TOUCH: break;
    case mjSENS_ACCELEROMETER:
      accelerometerIndices_.push_back(i);
      break;
    case mjSENS_VELOCIMETER:
      velocimeterIndices_.push_back(i);
      break;
    case mjSENS_GYRO:
      gyroIndices_.push_back(i);
      break;
    case mjSENS_FORCE:
      forceIndices_.push_back(i);
      break;
    case mjSENS_TORQUE:
      torqueIndices_.push_back(i);
      break;
    case mjSENS_MAGNETOMETER: break;
    case mjSENS_RANGEFINDER: break;
    case mjSENS_JOINTPOS: break;
    case mjSENS_JOINTVEL: break;
    case mjSENS_TENDONPOS: break;
    case mjSENS_TENDONVEL: break;
    case mjSENS_ACTUATORPOS: break;
    case mjSENS_ACTUATORVEL: break;
    case mjSENS_ACTUATORFRC: break;
    }

    bool debug = false;
    if (debug) {
      const char *name = mj_id2name(mj_.mjModel_, mjOBJ_SENSOR, i);
      std::string sensor_name = name ? name : "unnamed";
      int dim = mj_.mjModel_->sensor_dim[i];
      int adr = mj_.mjModel_->sensor_adr[i];

      std::cout << "Sensor " << i << ": " << sensor_name << " (Type: " << sensorType
                << ", Dim: " << dim << ")" << std::endl;
      std::cout << "Sensor dim: " << dim << std::endl;
      std::cout << "Sensor adr: " << adr << std::endl;
    }
  }

  // Debug output of sensor indices arrays
  if (mj_._show_sensor_indexarray) {
    if (framePosIndices_.size() > 0) {
      std::cout << "there are " << framePosIndices_.size() << " FRAMEPOS sensors, indices list: [ ";
      for (auto idx : framePosIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }

    if (frameQuatIndices_.size() > 0) {
      std::cout << "there are " << frameQuatIndices_.size() << " FRAMEQUAT sensors, indices list: [ ";
      for (auto idx : frameQuatIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }

    if (frameLinVelIndices_.size() > 0) {
      std::cout << "there are " << frameLinVelIndices_.size() << " FRAMELINVEL sensors, indices list: [ ";
      for (auto idx : frameLinVelIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }

    if (frameAngVelIndices_.size() > 0) {
      std::cout << "there are " << frameAngVelIndices_.size() << " FRAMEANGVEL sensors, indices list: [ ";
      for (auto idx : frameAngVelIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }

    if (gyroIndices_.size() > 0) {
      std::cout << "there are " << gyroIndices_.size() << " GYRO sensors, indices list: [ ";
      for (auto idx : gyroIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }

    if (velocimeterIndices_.size() > 0) {
      std::cout << "there are " << velocimeterIndices_.size() << " VELOCIMETER sensors, indices list: [ ";
      for (auto idx : velocimeterIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }

    if (accelerometerIndices_.size() > 0) {
      std::cout << "there are " << accelerometerIndices_.size() << " ACCELEROMETER sensors, indices list: [ ";
      for (auto idx : accelerometerIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }

    if (forceIndices_.size() > 0) {
      std::cout << "there are " << forceIndices_.size() << " FORCE sensors, indices list: [ ";
      for (auto idx : forceIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }

    if (torqueIndices_.size() > 0) {
      std::cout << "there are " << torqueIndices_.size() << " TORQUE sensors, indices list: [ ";
      for (auto idx : torqueIndices_) std::cout << idx << " ";
      std::cout << "]" << std::endl;
    }
  }

  initRGB();
}

void MujocoSensor::initRGB() {
  int cam_id = -1;
  frontCamera_.fixedcamid = cam_id;
  for (int i = 0; i < mj_.mjModel_->ncam; i++) {
    const char *camName = mj_id2name(mj_.mjModel_, mjOBJ_CAMERA, i);
    if (camName && strcmp(camName, "front_cam") == 0) {
      cam_id = i;
      break;
    }
  }
  if (cam_id >= 0) {
    mjv_defaultCamera(&frontCamera_);
    frontCamera_.type = mjCAMERA_FIXED;
    frontCamera_.fixedcamid = cam_id;
    mjv_defaultScene(&mjSceneFrontCamera_);
    mjv_makeScene(mj_.mjModel_, &mjSceneFrontCamera_, mj_._maxgeom);
  }
}

void MujocoSensor::update() {
  mj_forward(mj_.mjModel_, mj_.mjData_);
  updateFrameSensor();
  updateRawIMU();
  updateFTSensor();
  updateMultiRay();
}

void MujocoSensor::updateFrameSensor() {
  for (size_t frameIndex = 0; frameIndex < framePosIndices_.size(); frameIndex++) {
    int idx = framePosIndices_[frameIndex];
    const char *name = mj_id2name(mj_.mjModel_, mjOBJ_SENSOR, idx);
    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;
    std::string sensorName(name ? name : "");

    auto &frameSensor = mj_.runtimeData_.frameSensor(frameIndex);
    frameSensor.name = sensorName;
    frameSensor.pos = Eigen::Vector3d(data[0], data[1], data[2]);
  }

  for (size_t frameIndex = 0; frameIndex < frameQuatIndices_.size(); frameIndex++) {
    int idx = frameQuatIndices_[frameIndex];
    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;

    if (frameIndex < framePosIndices_.size()) {
      auto &frameSensor = mj_.runtimeData_.frameSensor(frameIndex);
      frameSensor.quat = Eigen::Quaterniond(data[0], data[1], data[2], data[3]);
    }
  }

  for (size_t frameIndex = 0; frameIndex < frameLinVelIndices_.size(); frameIndex++) {
    int idx = frameLinVelIndices_[frameIndex];
    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;

    if (frameIndex < framePosIndices_.size()) {
      auto &frameSensor = mj_.runtimeData_.frameSensor(frameIndex);
      frameSensor.velocity.head<3>() = Eigen::Vector3d(data[0], data[1], data[2]);
    }
  }

  for (size_t frameIndex = 0; frameIndex < frameAngVelIndices_.size(); frameIndex++) {
    int idx = frameAngVelIndices_[frameIndex];
    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;

    if (frameIndex < framePosIndices_.size()) {
      auto &frameSensor = mj_.runtimeData_.frameSensor(frameIndex);
      frameSensor.velocity.tail<3>() = Eigen::Vector3d(data[0], data[1], data[2]);
    }
  }

  for (size_t frameIndex = 0; frameIndex < frameLinAccIndices_.size(); frameIndex++) {
    int idx = frameLinAccIndices_[frameIndex];
    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;

    if (frameIndex < framePosIndices_.size()) {
      auto &frameSensor = mj_.runtimeData_.frameSensor(frameIndex);
      frameSensor.acceleration.head<3>() = Eigen::Vector3d(data[0], data[1], data[2]);
    }
  }

  for (size_t frameIndex = 0; frameIndex < frameAngAccIndices_.size(); frameIndex++) {
    int idx = frameAngAccIndices_[frameIndex];
    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;

    if (frameIndex < framePosIndices_.size()) {
      auto &frameSensor = mj_.runtimeData_.frameSensor(frameIndex);
      frameSensor.acceleration.tail<3>() = Eigen::Vector3d(data[0], data[1], data[2]);
    }
  }

  // Debug output
  if (mj_._show_ee_frame && !mj_.runtimeData_.frameSensors.empty()) {
    const auto &frameSensor = mj_.runtimeData_.frameSensors[0];
    std::cout << "mjTime: " << std::fixed << std::setprecision(5) << mj_.mjData_->time
              << " sec, mj : " << frameSensor.name << ": " << frameSensor.pos.transpose()
              << "; mj   eeQuat: " << frameSensor.quat.coeffs().transpose() << std::endl;
  }
}

void MujocoSensor::updateFTSensor() {
  for (size_t forceIndex = 0; forceIndex < forceIndices_.size(); forceIndex++) {
    int idx = forceIndices_[forceIndex];
    const char *name = mj_id2name(mj_.mjModel_, mjOBJ_SENSOR, idx);
    if (!name) continue;

    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;
    std::string sensorName(name);

    auto &ftSensor = mj_.runtimeData_.ftSensor(forceIndex);
    ftSensor.name = sensorName;
    ftSensor.force = Eigen::Vector3d(data[0], data[1], data[2]);
  }

  for (size_t torqueIndex = 0; torqueIndex < torqueIndices_.size(); torqueIndex++) {
    int idx = torqueIndices_[torqueIndex];
    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;

    auto &ftSensor = mj_.runtimeData_.ftSensor(torqueIndex);
    ftSensor.torque = Eigen::Vector3d(data[0], data[1], data[2]);
  }
}

void MujocoSensor::updateRawIMU() {
  for (size_t gyroIndex = 0; gyroIndex < gyroIndices_.size(); gyroIndex++) {
    int idx = gyroIndices_[gyroIndex];
    const char *name = mj_id2name(mj_.mjModel_, mjOBJ_SENSOR, idx);
    if (!name) continue;

    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;
    std::string sensorName(name);

    auto &imuSensor = mj_.runtimeData_.imuSensor(gyroIndex);
    imuSensor.name = sensorName;
    imuSensor.angVel = Eigen::Vector3d(data[0], data[1], data[2]);
  }

  for (size_t accIndex = 0; accIndex < accelerometerIndices_.size(); accIndex++) {
    int idx = accelerometerIndices_[accIndex];
    const char *name = mj_id2name(mj_.mjModel_, mjOBJ_SENSOR, idx);
    if (!name) continue;

    int adr = mj_.mjModel_->sensor_adr[idx];
    const mjtNum *data = mj_.mjData_->sensordata + adr;

    auto &imuSensor = mj_.runtimeData_.imuSensor(accIndex);
    imuSensor.linAcc = Eigen::Vector3d(data[0], data[1], data[2]);
  }

  // Debug output all IMU data if enabled
  if (mj_._show_imu_data && !mj_.runtimeData_.imuSensors.empty()) {
    for (size_t i = 0; i < mj_.runtimeData_.imuSensors.size(); i++) {
      const auto &imu = mj_.runtimeData_.imuSensors[i];
      std::cout << "mjTime: " << std::fixed << std::setprecision(5) << mj_.mjData_->time
                << " sec, IMU: " << imu.name
                << " gyro: " << imu.angVel.transpose()
                << " acc: " << imu.linAcc.transpose() << std::endl;
    }
  }
}

void MujocoSensor::updateMultiRay() {
  if (robotType_ != rynn::RobotType::diffmobile && robotType_ != rynn::RobotType::mobile_fr3) return;

  if (!mj_.timingManager->shouldTrigger("navigation", mj_.mjData_->time)) return;

  if (mj_.runtimeData_.frameSensors.empty()) return;

  double multi_ray_point[3] = {
      mj_.runtimeData_.frameSensors[0].pos[0],
      mj_.runtimeData_.frameSensors[0].pos[1] + 0.2,
      mj_.runtimeData_.frameSensors[0].pos[2] + 0.15};

  constexpr int num_rays = 271;
  mjtNum directions[num_rays * 3];
  int num_rays_half = (num_rays - 1) / 2;
  double angle_range = (num_rays - 1) / 180.0 * M_PI; // 1 degree per ray

  Eigen::Vector3d chassis_rpy = utils::quat2RPY(mj_.runtimeData_.frameSensors[0].quat.coeffs());
  for (int i = 0; i < num_rays; ++i) {
    double angle = ((num_rays - 1 - i) - num_rays_half) * angle_range / (num_rays - 1) - chassis_rpy[2];
    directions[3 * i] = sin(angle);
    directions[3 * i + 1] = cos(angle);
    directions[3 * i + 2] = 0.0;
  }

  bool include_static_flag = true;
  double max_dis_check = 20.0;

  int geomids[num_rays];
  mjtNum dists[num_rays];

  mj_multiRay(mj_.mjModel_, mj_.mjData_, multi_ray_point, directions, NULL,
              include_static_flag ? 1 : 0, -1, geomids, dists, num_rays, max_dis_check);
  mj_.multiRayDistances_.clear();
  mj_.multiRayDistances_.assign(dists, dists + num_rays);
}

void MujocoSensor::updateRGB() {
  if (robotType_ != rynn::RobotType::diffmobile && robotType_ != rynn::RobotType::mobile_fr3) return;

  if (frontCamera_.fixedcamid < 0) return;

  const int width = 640;
  const int height = 480;
  mjrRect viewport = {0, 0, width, height};

  mjv_updateScene(mj_.mjModel_, mj_.mjData_, &mj_.mjvOption_, NULL, &frontCamera_, mjCAT_ALL, &mjSceneFrontCamera_);

  mjr_render(viewport, &mjSceneFrontCamera_, &mj_.mjContext_);

  unsigned char rgb[width * height * 3];
  float depth[width * height];
  mjr_readPixels(rgb, depth, viewport, &mj_.mjContext_);

  cv::Mat img(height, width, CV_8UC3, rgb);

  // Optional: Convert RGB to BGR since OpenCV uses BGR by default
  cv::Mat bgr;
  cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
  cv::flip(bgr, bgr, 1);

  if (!mj_._show_camera_pic) return;
  cv::imshow("MuJoCo Offscreen Render", bgr);
  cv::waitKey(1);
}

void MujocoSensor::updateSites() {
  for (int i = 0; i < mj_.mjModel_->nsite; i++) {
    const char *siteName = mj_id2name(mj_.mjModel_, mjOBJ_SITE, i);
    std::string name = siteName ? siteName : "site" + std::to_string(i);
    mjtNum quat[4];
    mju_mat2Quat(quat, mj_.mjData_->site_xmat + 9 * i);
    mjtNum velocity[6];
    mj_objectVelocity(mj_.mjModel_, mj_.mjData_, mjOBJ_SITE, i, velocity, 0);
  }
}

} // namespace mujoco
