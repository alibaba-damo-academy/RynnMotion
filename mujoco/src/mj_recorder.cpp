#include "mj_recorder.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "mj_interface.hpp"

namespace mujoco {

MujocoRecorder::MujocoRecorder(MujocoInterface &mj)
    : mj_(mj), hdf5Writer_(std::make_unique<HDF5Writer>()) {}

MujocoRecorder::~MujocoRecorder() {
  if (isRecording_) {
    endEpisode();
  }
  finalize();
}

void MujocoRecorder::initRecorder() {
  std::filesystem::path projectRoot = std::filesystem::path(MODEL_DIR).parent_path();
  config_.datasetRoot = projectRoot / "record";
  config_.repoId = generateTimestampRepoId();
  config_.fps = static_cast<int>(1.0 / mj_.timingManager->getPeriod("camera"));
  config_.videoCodec = VideoCodec::H264;
  config_.crf = 23;
  config_.recordVideo = true;

  cameraConfigs_ = mj_.mjSensor->getCameraConfigs();

  auto *robotManager = mj_.robotManager.get();
  if (robotManager) {
    config_.robotType = "robot";
    int mdof = robotManager->getMotionDOF();

    features_["observation.state"] = {"float32", {mdof}, {}};
    features_["action"] = {"float32", {mdof}, {}};
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
    std::filesystem::create_directories(root / "videos" / "chunk-000" /
                                        ("observation.images." + cam.name));
  }
}

void MujocoRecorder::writeInfoJson() {
  auto infoPath = config_.datasetRoot / config_.repoId / "meta" / "info.json";
  std::ofstream ofs(infoPath);
  if (!ofs) return;

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
  ofs << "  \"data_path\": \"data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.hdf5\",\n";
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
      auto videoPath = config_.datasetRoot / config_.repoId /
                       getVideoFilePath(currentEpisodeIndex_, cam.name);
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

  for (const auto &gripper : runtimeData.gripperFeedbacks) {
    frame.gripperPositions.push_back(gripper.posFb);
  }

  frameBuffer_.push_back(frame);

  for (const auto &[camName, imgFrame] : images) {
    if (imageBuffers_.find(camName) != imageBuffers_.end()) {
      imageBuffers_[camName].push_back(imgFrame);

      if (videoEncoders_.find(camName) != videoEncoders_.end() &&
          videoEncoders_[camName]->isOpen()) {
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
  auto dataPath = config_.datasetRoot / config_.repoId /
                  getDataFilePath(currentEpisodeIndex_);

  hdf5Writer_->create(dataPath);

  size_t numFrames = frameBuffer_.size();
  if (numFrames == 0) {
    hdf5Writer_->close();
    return;
  }

  int mdof = frameBuffer_[0].qFb.size();

  std::vector<Eigen::VectorXd> qFbVecs, qCmdVecs;
  std::vector<double> timestamps;
  std::vector<int64_t> frameIndices, episodeIndices;

  for (const auto &frame : frameBuffer_) {
    qFbVecs.push_back(frame.qFb);
    qCmdVecs.push_back(frame.qCmd);
    timestamps.push_back(frame.timestamp);
    frameIndices.push_back(frame.frameIndex);
    episodeIndices.push_back(currentEpisodeIndex_);
  }

  hdf5Writer_->writeEigenVectors("observation.state", qFbVecs);
  hdf5Writer_->writeEigenVectors("action", qCmdVecs);
  hdf5Writer_->writeDataset("timestamp", timestamps, {numFrames});
  hdf5Writer_->writeDataset("frame_index", frameIndices, {numFrames});
  hdf5Writer_->writeDataset("episode_index", episodeIndices, {numFrames});

  hdf5Writer_->close();
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
  int chunk = getEpisodeChunk(episodeIndex);
  std::ostringstream ss;
  ss << "data/chunk-" << std::setfill('0') << std::setw(3) << chunk
     << "/episode_" << std::setfill('0') << std::setw(6) << episodeIndex << ".hdf5";
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
