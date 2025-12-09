#include "mj_recorder.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include "mj_interface.hpp"

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
    int mdof = robotManager->getMotionDOF();
    int numEE = robotManager->getNumEndEffectors();

    features_["observation.state"] = {"float32", {mdof}, {}};
    features_["observation.velocity"] = {"float32", {mdof}, {}};
    features_["observation.torque"] = {"float32", {mdof}, {}};
    features_["action"] = {"float32", {mdof}, {}};
    features_["action.velocity"] = {"float32", {mdof}, {}};
    features_["action.torque"] = {"float32", {mdof}, {}};

    if (numEE > 0) {
      features_["observation.ee_pos"] = {"float32", {numEE * 3}, {}};
      features_["observation.ee_quat"] = {"float32", {numEE * 4}, {}};
      features_["observation.gripper"] = {"float32", {numEE}, {}};
      features_["action.ee_pos"] = {"float32", {numEE * 3}, {}};
      features_["action.ee_quat"] = {"float32", {numEE * 4}, {}};
      features_["action.gripper"] = {"float32", {numEE}, {}};
    }

    features_["timestamp"] = {"float64", {1}, {}};
    features_["frame_index"] = {"int64", {1}, {}};
    features_["episode_index"] = {"int64", {1}, {}};
  }

  for (const auto &cam : cameraConfigs_) {
    std::string key = "observation.images." + cam.name;
    features_[key] = {"video", {cam.height, cam.width, 3}, {}};
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
    ofs << "]\n";
    ofs << "    }";
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

  for (const auto &state : runtimeData.bodyStates) {
    frame.eePoses.push_back(state.pos);
    frame.eeQuats.push_back(state.quat);
  }

  for (const auto &plan : runtimeData.bodyPlans) {
    frame.eePosCmd.push_back(plan.pos);
    frame.eeQuatCmd.push_back(plan.quat);
  }

  for (const auto &gripper : runtimeData.gripperFeedbacks) {
    frame.gripperPositions.push_back(gripper.posFb);
  }

  for (const auto &gripper : runtimeData.gripperCommands) {
    frame.gripperCommands.push_back(gripper.posCmd);
  }

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
  size_t numEE = frameBuffer_[0].eePoses.size();

  if (mdof == 0) {
    dataWriter_->close();
    return;
  }

  std::vector<Eigen::VectorXd> qFbVecs, qdFbVecs, qtauFbVecs;
  std::vector<Eigen::VectorXd> qCmdVecs, qdCmdVecs, qtauCmdVecs;
  std::vector<double> eePosObs, eeQuatObs;
  std::vector<double> eePosCmd, eeQuatCmd;
  std::vector<double> gripperObs, gripperCmd;
  std::vector<double> timestamps;
  std::vector<int64_t> frameIndices, episodeIndices;

  for (const auto &frame : frameBuffer_) {
    qFbVecs.push_back(frame.qFb);
    qdFbVecs.push_back(frame.qdFb);
    qtauFbVecs.push_back(frame.qtauFb);
    qCmdVecs.push_back(frame.qCmd);
    qdCmdVecs.push_back(frame.qdCmd);
    qtauCmdVecs.push_back(frame.qtauCmd);

    for (const auto &pos : frame.eePoses) {
      eePosObs.push_back(pos.x());
      eePosObs.push_back(pos.y());
      eePosObs.push_back(pos.z());
    }
    for (const auto &quat : frame.eeQuats) {
      eeQuatObs.push_back(quat.x());
      eeQuatObs.push_back(quat.y());
      eeQuatObs.push_back(quat.z());
      eeQuatObs.push_back(quat.w());
    }

    for (const auto &pos : frame.eePosCmd) {
      eePosCmd.push_back(pos.x());
      eePosCmd.push_back(pos.y());
      eePosCmd.push_back(pos.z());
    }
    for (const auto &quat : frame.eeQuatCmd) {
      eeQuatCmd.push_back(quat.x());
      eeQuatCmd.push_back(quat.y());
      eeQuatCmd.push_back(quat.z());
      eeQuatCmd.push_back(quat.w());
    }

    for (double g : frame.gripperPositions) {
      gripperObs.push_back(g);
    }

    for (double g : frame.gripperCommands) {
      gripperCmd.push_back(g);
    }

    timestamps.push_back(frame.timestamp);
    frameIndices.push_back(frame.frameIndex);
    episodeIndices.push_back(currentEpisodeIndex_);
  }

  dataWriter_->writeEigenVectors("observation.state", qFbVecs);
  dataWriter_->writeEigenVectors("observation.velocity", qdFbVecs);
  dataWriter_->writeEigenVectors("observation.torque", qtauFbVecs);
  dataWriter_->writeEigenVectors("action", qCmdVecs);
  dataWriter_->writeEigenVectors("action.velocity", qdCmdVecs);
  dataWriter_->writeEigenVectors("action.torque", qtauCmdVecs);

  if (numEE > 0) {
    dataWriter_->writeDataset("observation.ee_pos", eePosObs,
                              {numFrames, numEE * 3});
    dataWriter_->writeDataset("observation.ee_quat", eeQuatObs,
                              {numFrames, numEE * 4});
    dataWriter_->writeDataset("observation.gripper", gripperObs,
                              {numFrames, numEE});

    if (!eePosCmd.empty()) {
      size_t numEECmd = frameBuffer_[0].eePosCmd.size();
      dataWriter_->writeDataset("action.ee_pos", eePosCmd,
                                {numFrames, numEECmd * 3});
      dataWriter_->writeDataset("action.ee_quat", eeQuatCmd,
                                {numFrames, numEECmd * 4});
    }

    if (!gripperCmd.empty()) {
      size_t numGripper = frameBuffer_[0].gripperCommands.size();
      dataWriter_->writeDataset("action.gripper", gripperCmd,
                                {numFrames, numGripper});
    }
  }

  dataWriter_->writeDataset("timestamp", timestamps, {numFrames});
  dataWriter_->writeDataset("frame_index", frameIndices, {numFrames});
  dataWriter_->writeDataset("episode_index", episodeIndices, {numFrames});

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
