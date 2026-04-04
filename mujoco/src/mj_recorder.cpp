#include "mj_recorder.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include "mj_interface.hpp"
#include "robot_context.hpp"

namespace mujoco {

MujocoRecorder::MujocoRecorder(MujocoInterface &mj) : mj_(mj) {
}

MujocoRecorder::~MujocoRecorder() {
  if (isRecording_) {
    endEpisode();
  }
  finalize();
}

void MujocoRecorder::loadRecorderConfig() {
  std::filesystem::path projectRoot = std::filesystem::path(MODEL_DIR).parent_path();
  std::filesystem::path configPath = projectRoot / "config" / "mujoco.yaml";

  config_.dataFormat = getDefaultDataFormat();

  if (std::filesystem::exists(configPath)) {
    try {
      YAML::Node yamlConfig = YAML::LoadFile(configPath.string());

      if (yamlConfig["recorder"]) {
        auto recorderNode = yamlConfig["recorder"];

        if (recorderNode["data_format"]) {
          config_.dataFormat = parseDataFormat(recorderNode["data_format"].as<std::string>());
        }
        if (recorderNode["record_video"]) {
          config_.recordVideo = recorderNode["record_video"].as<bool>();
        }
        if (recorderNode["video_codec"]) {
          std::string codec = recorderNode["video_codec"].as<std::string>();
          if (codec == "h264") {
            config_.videoCodec = VideoCodec::H264;
          } else if (codec == "av1") {
            config_.videoCodec = VideoCodec::AV1;
          }
        }
        if (recorderNode["crf"]) {
          config_.crf = recorderNode["crf"].as<int>();
        }
        if (recorderNode["chunks_size"]) {
          config_.chunksSize = recorderNode["chunks_size"].as<int>();
        }
      }
    } catch (const std::exception &e) {
      std::cerr << "[Recorder] Failed to load config: " << e.what() << std::endl;
    }
  }

  std::cout << "[Recorder] Data format: " << dataFormatToString(config_.dataFormat) << std::endl;
}

std::map<std::string, data::CameraConfig> MujocoRecorder::buildCameraConfigMap() const {
  std::map<std::string, data::CameraConfig> map;
  for (const auto &cam : cameraConfigs_) {
    map[cam.name] = cam;
  }
  return map;
}

void MujocoRecorder::initRecorder() {
  loadRecorderConfig();

  std::filesystem::path projectRoot = std::filesystem::path(MODEL_DIR).parent_path();
  config_.datasetRoot = projectRoot / "record";
  config_.repoId = generateTimestampRepoId();
  config_.fps = static_cast<int>(1.0 / mj_.timingManager->getPeriod("camera"));

  dataWriter_ = createDataWriter(config_.dataFormat);

  cameraConfigs_ = mj_.mjSensor->getCameraConfigs();

  auto *robotManager = mj_.robotManager.get();
  if (robotManager) {
    config_.robotType = "robot";
    mdof_ = robotManager->getMotionDOF();
    adof_ = robotManager->getActionDOF();
    numEE_ = robotManager->getNumEndEffectors();
    eeNames_ = robotManager->getEENames();

    rynn::RobotContext ctx(*robotManager);

    std::map<std::string, std::tuple<int, int, int>> camConfigs;
    for (const auto &cam : cameraConfigs_) {
      camConfigs[cam.name] = {cam.height, cam.width, 3};
    }

    features_ = ctx.getRecordingFeatures(camConfigs, config_.recordVideo);
  } else {
    for (const auto &cam : cameraConfigs_) {
      std::string key = "observation.images." + cam.name;
      features_[key] = {"video", {cam.height, cam.width, 3}, {"height", "width", "channels"}};
    }
  }

  for (const auto &cam : cameraConfigs_) {
    imageBuffers_[cam.name] = {};
  }

  initDirectories();
  writeInfoJson();
}

void MujocoRecorder::startRecording(const std::string &task) {
  if (!isRecording_) {
    startEpisode(task);
    std::cout << "[Recorder] Started recording episode " << currentEpisodeIndex_ << std::endl;
  }
}

void MujocoRecorder::stopRecording() {
  if (isRecording_) {
    endEpisode();
    std::cout << "[Recorder] Stopped recording" << std::endl;
  }
}

void MujocoRecorder::newEpisode() {
  std::string task = currentTask_;
  stopRecording();
  startRecording(task);
}

void MujocoRecorder::initDirectories() {
  auto root = config_.datasetRoot / config_.repoId;
  std::filesystem::create_directories(root / "meta");
  std::filesystem::create_directories(root / "data" / "chunk-000");
  for (const auto &cam : cameraConfigs_) {
    std::filesystem::create_directories(root / "videos" / "chunk-000" / ("observation.images." + cam.name));
  }
}

void MujocoRecorder::writeInfoJson() {
  auto infoPath = config_.datasetRoot / config_.repoId / "meta" / "info.json";
  std::ofstream ofs(infoPath);
  if (!ofs) return;

  std::string dataExt = dataWriter_ ? dataWriter_->getExtension() : "";
  std::string dataPathTemplate;
  if (dataExt.empty()) {
    dataPathTemplate = "";
  } else {
    dataPathTemplate = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}" + dataExt;
  }

  ofs << "{\n";
  ofs << "  \"codebase_version\": \"v2.1\",\n";
  ofs << "  \"robot_type\": \"" << config_.robotType << "\",\n";
  ofs << "  \"total_episodes\": " << totalEpisodes_ << ",\n";
  ofs << "  \"total_frames\": " << totalFrames_ << ",\n";
  ofs << "  \"total_tasks\": " << totalTasks_ << ",\n";
  ofs << "  \"total_videos\": " << (totalEpisodes_ * cameraConfigs_.size()) << ",\n";
  ofs << "  \"total_chunks\": " << (getEpisodeChunk(totalEpisodes_) + 1) << ",\n";
  ofs << "  \"chunks_size\": " << config_.chunksSize << ",\n";
  ofs << "  \"fps\": " << config_.fps << ",\n";
  ofs << "  \"data_format\": \"" << dataFormatToString(config_.dataFormat) << "\",\n";
  if (!dataPathTemplate.empty()) {
    ofs << "  \"data_path\": \"" << dataPathTemplate << "\",\n";
  }
  ofs << "  \"video_path\": \"videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4\",\n";
  ofs << "  \"features\": {\n";

  bool first = true;
  for (const auto &[name, info] : features_) {
    if (!first) ofs << ",\n";
    first = false;
    ofs << "    \"" << name << "\": {\n";
    ofs << "      \"dtype\": \"" << info.dtype << "\",\n";
    ofs << "      \"shape\": [";
    for (size_t i = 0; i < info.shape.size(); ++i) {
      if (i > 0) ofs << ", ";
      ofs << info.shape[i];
    }
    ofs << "]";

    if (!info.names.empty()) {
      ofs << ",\n      \"names\": [";
      for (size_t i = 0; i < info.names.size(); ++i) {
        if (i > 0) ofs << ", ";
        ofs << "\"" << info.names[i] << "\"";
      }
      ofs << "]";
    }

    ofs << "\n    }";
  }

  ofs << "\n  }\n";
  ofs << "}\n";
}

void MujocoRecorder::startEpisode(const std::string &task) {
  if (isRecording_) {
    endEpisode();
  }

  currentTask_ = task;
  frameBuffer_.clear();
  for (auto &[name, buffer] : imageBuffers_) {
    buffer.clear();
  }

  if (taskToIndex_.find(task) == taskToIndex_.end()) {
    taskToIndex_[task] = totalTasks_++;
    appendTaskMetadata(task);
  }

  if (config_.recordVideo) {
    for (const auto &cam : cameraConfigs_) {
      auto encoder = std::make_unique<VideoEncoder>(config_.videoCodec, config_.fps, config_.crf);
      auto videoPath = config_.datasetRoot / config_.repoId / getVideoFilePath(currentEpisodeIndex_, cam.name);
      encoder->open(videoPath, cam.width, cam.height);
      videoEncoders_[cam.name] = std::move(encoder);
    }
  }

  isRecording_ = true;
}

void MujocoRecorder::recordFrame(const data::RuntimeData &runtimeData,
                                 const std::map<std::string, data::ImageFrame> &images) {
  if (!isRecording_) return;

  EpisodeFrame frame;
  frame.timestamp = runtimeData.simTime;
  frame.frameIndex = static_cast<int>(frameBuffer_.size());
  frame.qFb = runtimeData.qFb;
  frame.qdFb = runtimeData.qdFb;
  frame.qtauFb = runtimeData.qtauFb;
  frame.qCmd = runtimeData.qCmd;
  frame.qdCmd = runtimeData.qdCmd;
  frame.qtauCmd = runtimeData.qtauCmd;

  // Build 7D EE poses from bodyStates (pos, qx, qy, qz, qw)
  for (const auto &state : runtimeData.bodyStates) {
    Eigen::Matrix<double, 7, 1> pose;
    pose << state.pos.x(), state.pos.y(), state.pos.z(),
            state.quat.x(), state.quat.y(), state.quat.z(), state.quat.w();
    frame.eePose7d.push_back(pose);
  }

  for (const auto &plan : runtimeData.bodyPlans) {
    Eigen::Matrix<double, 7, 1> pose;
    pose << plan.pos.x(), plan.pos.y(), plan.pos.z(),
            plan.quat.x(), plan.quat.y(), plan.quat.z(), plan.quat.w();
    frame.eePoseCmd7d.push_back(pose);
  }

  for (const auto &gripper : runtimeData.gripperFeedbacks) {
    frame.gripperPositions.push_back(gripper.posFb);
  }

  for (const auto &gripper : runtimeData.gripperCommands) {
    frame.gripperCommands.push_back(gripper.posCmd);
  }

  // Extract sensor data from RuntimeData
  for (size_t i = 0; i < runtimeData.ftSensors.size(); ++i) {
    const auto &ft = runtimeData.ftSensors[i];
    if (!ft.name.empty()) {
      std::string forceKey = "observation.sensor.force." + ft.name;
      frame.sensorData[forceKey] = ft.force;

      std::string torqueKey = "observation.sensor.torque." + ft.name;
      frame.sensorData[torqueKey] = ft.torque;
    }
  }

  for (size_t i = 0; i < runtimeData.imuSensors.size(); ++i) {
    const auto &imu = runtimeData.imuSensors[i];
    if (!imu.name.empty()) {
      std::string gyroKey = "observation.sensor.gyro." + imu.name;
      frame.sensorData[gyroKey] = imu.angVel;

      std::string accelKey = "observation.sensor.accel." + imu.name;
      frame.sensorData[accelKey] = imu.linAcc;
    }
  }

  frame.taskIndex = taskToIndex_.count(currentTask_) ? taskToIndex_[currentTask_] : 0;

  frameBuffer_.push_back(frame);

  for (const auto &[camName, imgFrame] : images) {
    if (imageBuffers_.find(camName) != imageBuffers_.end()) {
      imageBuffers_[camName].push_back(imgFrame);

      if (videoEncoders_.find(camName) != videoEncoders_.end() && videoEncoders_[camName]->isOpen()) {
        videoEncoders_[camName]->writeFrame(imgFrame.ptr(), imgFrame.width, imgFrame.height);
      }
    }
  }
}

void MujocoRecorder::endEpisode() {
  if (!isRecording_) return;

  isRecording_ = false;

  for (auto &[name, encoder] : videoEncoders_) {
    if (encoder && encoder->isOpen()) {
      encoder->close();
    }
  }
  videoEncoders_.clear();

  if (frameBuffer_.empty()) return;

  writeEpisodeData();
  appendEpisodeMetadata(static_cast<int>(frameBuffer_.size()));

  totalFrames_ += static_cast<int>(frameBuffer_.size());
  totalEpisodes_++;
  currentEpisodeIndex_++;

  writeInfoJson();

  frameBuffer_.clear();
  for (auto &[name, buffer] : imageBuffers_) {
    buffer.clear();
  }
}

void MujocoRecorder::writeEpisodeData() {
  if (config_.dataFormat == DataFormat::None || !dataWriter_) {
    return;
  }

  auto dataPath = config_.datasetRoot / config_.repoId / getDataFilePath(currentEpisodeIndex_);

  dataWriter_->create(dataPath);

  size_t numFrames = frameBuffer_.size();
  if (numFrames == 0) {
    dataWriter_->close();
    return;
  }

  int mdof = frameBuffer_[0].qFb.size();

  if (mdof == 0) {
    dataWriter_->close();
    return;
  }

  // Joint state vectors
  std::vector<Eigen::VectorXd> qFbVecs, qdFbVecs, qtauFbVecs;
  std::vector<Eigen::VectorXd> qCmdVecs, qdCmdVecs, qtauCmdVecs;

  // Action = joints + grippers concatenated
  std::vector<Eigen::VectorXd> actionVecs;

  // Gripper
  std::vector<double> gripperObs;

  // EE poses: per-EE 7D vectors
  std::map<std::string, std::vector<double>> eePoseObsMap;
  std::map<std::string, std::vector<double>> eePoseCmdMap;

  // Sensor data accumulation
  std::map<std::string, std::vector<double>> sensorDataMap;

  // Metadata
  std::vector<double> timestamps;
  std::vector<int64_t> frameIndices, episodeIndices, globalIndices, taskIndices;

  for (const auto &frame : frameBuffer_) {
    qFbVecs.push_back(frame.qFb);
    qdFbVecs.push_back(frame.qdFb);
    qtauFbVecs.push_back(frame.qtauFb);
    qdCmdVecs.push_back(frame.qdCmd);
    qtauCmdVecs.push_back(frame.qtauCmd);

    // Build action vector: qCmd + gripperCommands
    int actionDim = mdof + static_cast<int>(frame.gripperCommands.size());
    Eigen::VectorXd actionVec(actionDim);
    actionVec.head(mdof) = frame.qCmd;
    for (size_t g = 0; g < frame.gripperCommands.size(); ++g) {
      actionVec[mdof + g] = frame.gripperCommands[g];
    }
    actionVecs.push_back(actionVec);

    // Gripper feedback
    for (double g : frame.gripperPositions) {
      gripperObs.push_back(g);
    }

    // EE poses (observation) - 7D per EE
    for (size_t i = 0; i < frame.eePose7d.size(); ++i) {
      std::string key;
      if (numEE_ == 1) {
        key = "observation.end_effector_pose";
      } else {
        std::string eeName = (i < eeNames_.size()) ? eeNames_[i] : "ee" + std::to_string(i);
        key = "observation." + eeName + "_pose";
      }
      for (int j = 0; j < 7; ++j) {
        eePoseObsMap[key].push_back(frame.eePose7d[i][j]);
      }
    }

    // EE poses (action) - 7D per EE
    for (size_t i = 0; i < frame.eePoseCmd7d.size(); ++i) {
      std::string key;
      if (numEE_ == 1) {
        key = "action.end_effector_pose";
      } else {
        std::string eeName = (i < eeNames_.size()) ? eeNames_[i] : "ee" + std::to_string(i);
        key = "action." + eeName + "_pose";
      }
      for (int j = 0; j < 7; ++j) {
        eePoseCmdMap[key].push_back(frame.eePoseCmd7d[i][j]);
      }
    }

    // Sensor data
    for (const auto &[sensorKey, sensorVec] : frame.sensorData) {
      for (int j = 0; j < sensorVec.size(); ++j) {
        sensorDataMap[sensorKey].push_back(sensorVec[j]);
      }
    }

    timestamps.push_back(frame.timestamp);
    frameIndices.push_back(frame.frameIndex);
    episodeIndices.push_back(currentEpisodeIndex_);
    globalIndices.push_back(globalIndex_++);
    taskIndices.push_back(frame.taskIndex);
  }

  // Write joint state columns
  dataWriter_->writeEigenVectors("observation.state", qFbVecs);
  dataWriter_->writeEigenVectors("observation.velocity", qdFbVecs);
  dataWriter_->writeEigenVectors("observation.torque", qtauFbVecs);
  dataWriter_->writeEigenVectors("action", actionVecs);
  dataWriter_->writeEigenVectors("action.velocity", qdCmdVecs);
  dataWriter_->writeEigenVectors("action.torque", qtauCmdVecs);

  // Write gripper state
  if (!gripperObs.empty() && numEE_ > 0) {
    dataWriter_->writeDataset("observation.state.gripper", gripperObs,
                              {numFrames, static_cast<size_t>(numEE_)});
  }

  // Write EE pose columns (7D per EE)
  for (const auto &[key, data] : eePoseObsMap) {
    dataWriter_->writeDataset(key, data, {numFrames, 7});
  }
  for (const auto &[key, data] : eePoseCmdMap) {
    dataWriter_->writeDataset(key, data, {numFrames, 7});
  }

  // Write sensor columns
  for (const auto &[key, data] : sensorDataMap) {
    size_t dim = data.size() / numFrames;
    if (dim > 0) {
      dataWriter_->writeDataset(key, data, {numFrames, dim});
    }
  }

  // Write metadata columns
  dataWriter_->writeDataset("timestamp", timestamps, {numFrames});
  dataWriter_->writeDataset("frame_index", frameIndices, {numFrames});
  dataWriter_->writeDataset("episode_index", episodeIndices, {numFrames});
  dataWriter_->writeDataset("index", globalIndices, {numFrames});
  dataWriter_->writeDataset("task_index", taskIndices, {numFrames});

  dataWriter_->close();
}

void MujocoRecorder::appendEpisodeMetadata(int episodeLength) {
  auto episodesPath = config_.datasetRoot / config_.repoId / "meta" / "episodes.jsonl";
  std::ofstream ofs(episodesPath, std::ios::app);
  if (!ofs) return;

  ofs << "{\"episode_index\": " << currentEpisodeIndex_
      << ", \"tasks\": [\"" << currentTask_ << "\"]"
      << ", \"length\": " << episodeLength << "}\n";
}

void MujocoRecorder::appendTaskMetadata(const std::string &task) {
  auto tasksPath = config_.datasetRoot / config_.repoId / "meta" / "tasks.jsonl";
  std::ofstream ofs(tasksPath, std::ios::app);
  if (!ofs) return;

  ofs << "{\"task_index\": " << taskToIndex_[task]
      << ", \"task\": \"" << task << "\"}\n";
}

void MujocoRecorder::finalize() {
  writeInfoJson();
}

bool MujocoRecorder::isRecording() const {
  return isRecording_;
}

int MujocoRecorder::currentEpisodeIndex() const {
  return currentEpisodeIndex_;
}

int MujocoRecorder::totalFrames() const {
  return totalFrames_;
}

int MujocoRecorder::totalEpisodes() const {
  return totalEpisodes_;
}

int MujocoRecorder::getEpisodeChunk(int episodeIndex) const {
  return episodeIndex / config_.chunksSize;
}

std::string MujocoRecorder::getDataFilePath(int episodeIndex) const {
  if (config_.dataFormat == DataFormat::None || !dataWriter_) {
    return "";
  }

  int chunk = getEpisodeChunk(episodeIndex);
  std::ostringstream ss;
  ss << "data/chunk-" << std::setfill('0') << std::setw(3) << chunk
     << "/episode_" << std::setfill('0') << std::setw(6) << episodeIndex
     << dataWriter_->getExtension();
  return ss.str();
}

std::string MujocoRecorder::getVideoFilePath(int episodeIndex, const std::string &cameraKey) const {
  int chunk = getEpisodeChunk(episodeIndex);
  std::ostringstream ss;
  ss << "videos/chunk-" << std::setfill('0') << std::setw(3) << chunk
     << "/observation.images." << cameraKey
     << "/episode_" << std::setfill('0') << std::setw(6) << episodeIndex << ".mp4";
  return ss.str();
}

} // namespace mujoco
