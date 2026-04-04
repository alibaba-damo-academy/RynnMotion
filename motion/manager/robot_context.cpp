#include "robot_context.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "robot_manager.hpp"

namespace rynn {

RobotContext::RobotContext(const RobotManager &rm)
    : mdof_(rm.getMotionDOF()),
      adof_(rm.getActionDOF()),
      numEE_(rm.getNumEndEffectors()),
      isDexHand_(rm.isDexHand()),
      jointNames_(rm.getJointNames()),
      eeNames_(rm.getEENames()),
      cameraNames_(rm.getCameraNames()) {

  auto toGroup = [](const std::vector<SensorInfo> &infos) {
    std::vector<SensorGroup> groups;
    for (const auto &s : infos) {
      groups.push_back({s.name, s.dim});
    }
    return groups;
  };

  auto force = rm.getForceSensors();
  auto torque = rm.getTorqueSensors();
  auto gyro = rm.getGyroSensors();
  auto accel = rm.getAccelerometerSensors();
  auto rangefinder = rm.getRangefinderSensors();
  auto touch = rm.getTouchSensors();

  if (!force.empty()) sensors_["force"] = toGroup(force);
  if (!torque.empty()) sensors_["torque"] = toGroup(torque);
  if (!gyro.empty()) sensors_["gyro"] = toGroup(gyro);
  if (!accel.empty()) sensors_["accel"] = toGroup(accel);
  if (!rangefinder.empty()) sensors_["rangefinder"] = toGroup(rangefinder);
  if (!touch.empty()) sensors_["touch"] = toGroup(touch);
}

FeatureMap RobotContext::getHwFeatures(
    const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs,
    bool useVideos) const {

  FeatureMap features;

  if (!jointNames_.empty()) {
    features["observation.state"] = {"float32", {mdof_}, jointNames_};
  }

  std::vector<std::string> actionNames = jointNames_;
  actionNames.insert(actionNames.end(), eeNames_.begin(), eeNames_.end());
  if (!actionNames.empty()) {
    features["action"] = {"float32", {static_cast<int>(actionNames.size())}, actionNames};
  }

  if (!eeNames_.empty()) {
    features["observation.state.gripper"] = {"float32", {static_cast<int>(eeNames_.size())}, eeNames_};
  }

  // Cameras: use provided configs, or auto-generate defaults if cameras detected
  auto effectiveConfigs = cameraConfigs;
  if (effectiveConfigs.empty() && !cameraNames_.empty()) {
    for (const auto &cam : cameraNames_) {
      effectiveConfigs[cam] = {480, 640, 3};
    }
  }
  for (const auto &[camName, res] : effectiveConfigs) {
    auto [h, w, c] = res;
    features["observation.images." + camName] = {useVideos ? "video" : "image", {h, w, c}, {"height", "width", "channels"}};
  }

  addSensorFeatures(features);

  return features;
}

FeatureMap RobotContext::getRecordingFeatures(
    const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs,
    bool useVideos) const {

  FeatureMap features;

  features["observation.state"] = {"float32", {mdof_}, jointNames_};
  features["observation.velocity"] = {"float32", {mdof_}, jointNames_};
  features["observation.torque"] = {"float32", {mdof_}, jointNames_};

  std::vector<std::string> actionNames = jointNames_;
  actionNames.insert(actionNames.end(), eeNames_.begin(), eeNames_.end());
  int actionDim = static_cast<int>(actionNames.size());
  features["action"] = {"float32", {actionDim}, actionNames};
  features["action.velocity"] = {"float32", {mdof_}, jointNames_};
  features["action.torque"] = {"float32", {mdof_}, jointNames_};

  if (!eeNames_.empty()) {
    features["observation.state.gripper"] = {"float32", {static_cast<int>(eeNames_.size())}, eeNames_};
  }

  addEEPoseFeatures(features, "observation");
  addEEPoseFeatures(features, "action");

  // Cameras: use provided configs, or auto-generate defaults
  auto effectiveConfigs = cameraConfigs;
  if (effectiveConfigs.empty() && !cameraNames_.empty()) {
    for (const auto &cam : cameraNames_) {
      effectiveConfigs[cam] = {480, 640, 3};
    }
  }
  for (const auto &[camName, res] : effectiveConfigs) {
    auto [h, w, c] = res;
    features["observation.images." + camName] = {useVideos ? "video" : "image", {h, w, c}, {"height", "width", "channels"}};
  }

  // Camera poses
  addCameraPoseFeatures(features, effectiveConfigs);

  addSensorFeatures(features);
  addMetadataFeatures(features);

  return features;
}

void RobotContext::addEEPoseFeatures(FeatureMap &features, const std::string &prefix) const {
  if (numEE_ <= 0) return;

  std::vector<std::string> poseNames = {"x", "y", "z", "qx", "qy", "qz", "qw"};

  if (numEE_ == 1) {
    features[prefix + ".end_effector_pose"] = {"float32", {7}, poseNames};
  } else {
    for (int i = 0; i < numEE_; ++i) {
      std::string eeName = (i < static_cast<int>(eeNames_.size())) ? eeNames_[i] : "ee" + std::to_string(i);
      features[prefix + "." + eeName + "_pose"] = {"float32", {7}, poseNames};
    }
  }
}

void RobotContext::addCameraPoseFeatures(FeatureMap &features,
    const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs) const {
  std::vector<std::string> poseNames = {"x", "y", "z", "qx", "qy", "qz", "qw"};
  for (const auto &[camName, _] : cameraConfigs) {
    features["observation.camera_poses." + camName] = {"float32", {7}, poseNames};
    features["action.camera_poses." + camName] = {"float32", {7}, poseNames};
  }
}

void RobotContext::addSensorFeatures(FeatureMap &features) const {
  for (const auto &[category, sensorList] : sensors_) {
    for (const auto &sensor : sensorList) {
      std::string key = "observation.sensor." + category + "." + sensor.name;
      std::vector<std::string> names;

      if (category == "force") {
        for (int i = 0; i < sensor.dim; ++i) names.push_back("f" + std::to_string(i));
      } else if (category == "torque") {
        for (int i = 0; i < sensor.dim; ++i) names.push_back("t" + std::to_string(i));
      } else if (category == "gyro") {
        std::vector<std::string> gyroNames = {"wx", "wy", "wz"};
        for (int i = 0; i < sensor.dim && i < 3; ++i) names.push_back(gyroNames[i]);
      } else if (category == "accel") {
        std::vector<std::string> accelNames = {"ax", "ay", "az"};
        for (int i = 0; i < sensor.dim && i < 3; ++i) names.push_back(accelNames[i]);
      } else if (category == "rangefinder") {
        names.push_back("distance");
      } else if (category == "touch") {
        names.push_back("force");
      }

      features[key] = {"float32", {sensor.dim}, names};
    }
  }
}

void RobotContext::addMetadataFeatures(FeatureMap &features) const {
  features["timestamp"] = {"float64", {1}, {}};
  features["frame_index"] = {"int64", {1}, {}};
  features["episode_index"] = {"int64", {1}, {}};
  features["index"] = {"int64", {1}, {}};
  features["task_index"] = {"int64", {1}, {}};
}

std::string RobotContext::toSchemaJson(const FeatureMap &features) const {
  std::ostringstream oss;
  oss << "{\n  \"features\": {\n";

  bool first = true;
  for (const auto &[name, info] : features) {
    if (!first) oss << ",\n";
    first = false;

    oss << "    \"" << name << "\": {\n";
    oss << "      \"dtype\": \"" << info.dtype << "\",\n";
    oss << "      \"shape\": [";
    for (size_t i = 0; i < info.shape.size(); ++i) {
      if (i > 0) oss << ", ";
      oss << info.shape[i];
    }
    oss << "]";

    if (!info.names.empty()) {
      oss << ",\n      \"names\": [";
      for (size_t i = 0; i < info.names.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << "\"" << info.names[i] << "\"";
      }
      oss << "]";
    }

    oss << "\n    }";
  }

  oss << "\n  }\n}\n";
  return oss.str();
}

std::string RobotContext::toInfoJson(const FeatureMap &features, int fps,
                                     const std::string &robotType,
                                     bool useVideos) const {
  std::ostringstream oss;
  oss << "{\n";
  oss << "    \"codebase_version\": \"v3.0\",\n";
  oss << "    \"robot_type\": " << (robotType.empty() ? "null" : "\"" + robotType + "\"") << ",\n";
  oss << "    \"total_episodes\": 0,\n";
  oss << "    \"total_frames\": 0,\n";
  oss << "    \"total_tasks\": 0,\n";
  oss << "    \"chunks_size\": 1000,\n";
  oss << "    \"data_files_size_in_mb\": 100,\n";
  oss << "    \"video_files_size_in_mb\": 200,\n";
  oss << "    \"fps\": " << fps << ",\n";
  oss << "    \"splits\": {},\n";
  oss << "    \"data_path\": \"data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet\",\n";
  if (useVideos) {
    oss << "    \"video_path\": \"videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4\",\n";
  } else {
    oss << "    \"video_path\": null,\n";
  }

  // Features
  oss << "    \"features\": {\n";
  bool first = true;
  for (const auto &[name, info] : features) {
    if (!first) oss << ",\n";
    first = false;

    oss << "        \"" << name << "\": {\n";
    oss << "            \"dtype\": \"" << info.dtype << "\",\n";
    oss << "            \"shape\": [";
    for (size_t i = 0; i < info.shape.size(); ++i) {
      if (i > 0) oss << ", ";
      oss << info.shape[i];
    }
    oss << "],\n";
    oss << "            \"names\": ";
    if (info.names.empty()) {
      oss << "null";
    } else {
      oss << "[";
      for (size_t i = 0; i < info.names.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << "\"" << info.names[i] << "\"";
      }
      oss << "]";
    }
    oss << "\n        }";
  }
  oss << "\n    }\n";
  oss << "}\n";
  return oss.str();
}

void RobotContext::writeEmptyDataset(const std::string &outputDir, int fps,
                                     const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs,
                                     bool useVideos) const {
  namespace fs = std::filesystem;

  fs::create_directories(outputDir + "/meta/episodes");
  fs::create_directories(outputDir + "/data");
  if (useVideos && !cameraNames_.empty()) {
    fs::create_directories(outputDir + "/videos");
  }

  auto features = getRecordingFeatures(cameraConfigs, useVideos);

  // Derive robot type from joint names heuristic
  std::string robotType;
  if (!jointNames_.empty()) {
    std::string first = jointNames_[0];
    auto pos = first.find('_');
    if (pos != std::string::npos) {
      robotType = first.substr(0, pos);
    }
  }

  // Write info.json
  {
    std::ofstream f(outputDir + "/meta/info.json");
    f << toInfoJson(features, fps, robotType, useVideos);
  }

  // Write stats.json (empty)
  {
    std::ofstream f(outputDir + "/meta/stats.json");
    f << "{}\n";
  }
}

std::string RobotContext::toRos2ConfigYaml(const std::string &topicPrefix,
    const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs) const {
  std::ostringstream oss;
  oss << "protocol: ros2\n";
  oss << "topics:\n";

  std::string prefix = topicPrefix.empty() ? "/robot_driver" : topicPrefix;

  // Joint states
  if (!jointNames_.empty()) {
    oss << "  - topic: " << prefix << "/joint_states\n";
    oss << "    type: sensor_msgs.msg.JointState\n";
    oss << "    adapter: GenericAdapter\n";
    oss << "    mappings:\n";
    oss << "      - field: position\n";
    oss << "        out_key: observation.state\n";

    oss << "  - topic: " << prefix << "/joint_cmd\n";
    oss << "    type: sensor_msgs.msg.JointState\n";
    oss << "    adapter: GenericAdapter\n";
    oss << "    mappings:\n";
    oss << "      - field: position\n";
    oss << "        out_key: action\n";
  }

  // Gripper
  if (!eeNames_.empty()) {
    oss << "  - topic: " << prefix << "/gripper_states\n";
    oss << "    type: sensor_msgs.msg.JointState\n";
    oss << "    adapter: GripperStateAdapter\n";
    oss << "    mappings:\n";
    oss << "      - field: position\n";
    oss << "        out_key: observation.state.gripper\n";

    oss << "  - topic: " << prefix << "/gripper_cmd\n";
    oss << "    type: sensor_msgs.msg.JointState\n";
    oss << "    adapter: GenericAdapter\n";
    oss << "    mappings:\n";
    oss << "      - field: position\n";
    oss << "        out_key: action.gripper\n";
  }

  // EE pose
  if (numEE_ > 0) {
    oss << "  - topic: " << prefix << "/ee_pose\n";
    oss << "    type: geometry_msgs.msg.PoseStamped\n";
    oss << "    adapter: GenericAdapter\n";
    oss << "    mappings:\n";
    for (const auto &field : {"pose.position.x", "pose.position.y", "pose.position.z",
                               "pose.orientation.x", "pose.orientation.y",
                               "pose.orientation.z", "pose.orientation.w"}) {
      oss << "      - field: " << field << "\n";
      oss << "        out_key: observation.end_effector_pose\n";
    }
  }

  // Force/torque sensors
  for (const auto &[category, sensorList] : sensors_) {
    for (const auto &sensor : sensorList) {
      if (category == "force" || category == "torque") {
        oss << "  - topic: " << prefix << "/" << sensor.name << "\n";
        oss << "    type: geometry_msgs.msg.WrenchStamped\n";
        oss << "    adapter: GenericAdapter\n";
        oss << "    mappings:\n";
        if (category == "force") {
          for (const auto &f : {"wrench.force.x", "wrench.force.y", "wrench.force.z"}) {
            oss << "      - field: " << f << "\n";
            oss << "        out_key: observation.sensor.force." << sensor.name << "\n";
          }
        } else {
          for (const auto &f : {"wrench.torque.x", "wrench.torque.y", "wrench.torque.z"}) {
            oss << "      - field: " << f << "\n";
            oss << "        out_key: observation.sensor.torque." << sensor.name << "\n";
          }
        }
      } else if (category == "gyro" || category == "accel") {
        oss << "  - topic: " << prefix << "/" << sensor.name << "\n";
        oss << "    type: sensor_msgs.msg.Imu\n";
        oss << "    adapter: GenericAdapter\n";
        oss << "    mappings:\n";
        if (category == "gyro") {
          for (const auto &f : {"angular_velocity.x", "angular_velocity.y", "angular_velocity.z"}) {
            oss << "      - field: " << f << "\n";
            oss << "        out_key: observation.sensor.gyro." << sensor.name << "\n";
          }
        } else {
          for (const auto &f : {"linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}) {
            oss << "      - field: " << f << "\n";
            oss << "        out_key: observation.sensor.accel." << sensor.name << "\n";
          }
        }
      }
    }
  }

  // Cameras
  auto effectiveConfigs = cameraConfigs;
  if (effectiveConfigs.empty() && !cameraNames_.empty()) {
    for (const auto &cam : cameraNames_) {
      effectiveConfigs[cam] = {480, 640, 3};
    }
  }
  for (const auto &[camName, _] : effectiveConfigs) {
    oss << "  - topic: /camera/" << camName << "/color/image_raw\n";
    oss << "    type: sensor_msgs.msg.Image\n";
    oss << "    adapter: CameraAdapter\n";
    oss << "    mappings:\n";
    oss << "      - field: data\n";
    oss << "        out_key: observation.images." << camName << "\n";
  }

  return oss.str();
}

} // namespace rynn
