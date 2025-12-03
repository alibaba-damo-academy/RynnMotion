#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <Eigen/Dense>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "image_buffer.hpp"
#include "mj_hdf5_writer.hpp"
#include "mj_video_encoder.hpp"
#include "runtime_data.hpp"

namespace rynn {
class RobotManager;
}

namespace mujoco {

inline std::string generateTimestampRepoId() {
  auto now = std::chrono::system_clock::now();
  std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
  std::tm *localTime = std::localtime(&nowTime);

  std::ostringstream oss;
  oss << "mj_" << std::put_time(localTime, "%Y%m%d_%H%M");
  return oss.str();
}

struct RecorderConfig {
  std::filesystem::path datasetRoot{"record"};
  std::string repoId{"local/recording"};
  std::string robotType{"unknown"};
  int fps{30};
  VideoCodec videoCodec{VideoCodec::AV1};
  int crf{30};
  int chunksSize{1000};
  bool recordVideo{true};
};

struct FeatureInfo {
  std::string dtype;
  std::vector<int> shape;
  std::vector<std::string> names;
};

struct EpisodeFrame {
  double timestamp;
  int frameIndex;
  Eigen::VectorXd qFb;
  Eigen::VectorXd qdFb;
  Eigen::VectorXd qtauFb;
  Eigen::VectorXd qCmd;
  Eigen::VectorXd qdCmd;
  Eigen::VectorXd qtauCmd;
  std::vector<Eigen::Vector3d> eePoses;
  std::vector<Eigen::Quaterniond> eeQuats;
  std::vector<double> gripperPositions;
};

class MujocoInterface;

/**
 * @class MujocoRecorder
 * @brief Records robot data and camera frames to HDF5/MP4 datasets
 */
class MujocoRecorder {
public:
  explicit MujocoRecorder(MujocoInterface &mj);
  ~MujocoRecorder();

  MujocoRecorder(const MujocoRecorder &) = delete;
  MujocoRecorder &operator=(const MujocoRecorder &) = delete;

  void initRecorder();
  void startRecording(const std::string &task = "default");
  void stopRecording();
  void newEpisode();

  void startEpisode(const std::string &task);
  void recordFrame(const data::RuntimeData &runtimeData,
                   const std::map<std::string, data::ImageFrame> &images);
  void endEpisode();
  void finalize();

  bool isRecording() const;
  int currentEpisodeIndex() const;
  int totalFrames() const;
  int totalEpisodes() const;

private:
  void initDirectories();
  void writeInfoJson();
  void writeEpisodeData();
  void encodeEpisodeVideos();
  void appendEpisodeMetadata(int episodeLength);
  void appendTaskMetadata(const std::string &task);
  int getEpisodeChunk(int episodeIndex) const;
  std::string getDataFilePath(int episodeIndex) const;
  std::string getVideoFilePath(int episodeIndex, const std::string &cameraKey) const;

  MujocoInterface &mj_;

  RecorderConfig config_;
  std::unique_ptr<HDF5Writer> hdf5Writer_;
  std::map<std::string, std::unique_ptr<VideoEncoder>> videoEncoders_;

  std::vector<EpisodeFrame> frameBuffer_;
  std::map<std::string, std::vector<data::ImageFrame>> imageBuffers_;
  std::map<std::string, FeatureInfo> features_;
  std::map<std::string, int> taskToIndex_;

  bool isRecording_{false};
  int currentEpisodeIndex_{0};
  int totalFrames_{0};
  int totalEpisodes_{0};
  int totalTasks_{0};
  std::string currentTask_;

  std::vector<data::CameraConfig> cameraConfigs_;
};

} // namespace mujoco
