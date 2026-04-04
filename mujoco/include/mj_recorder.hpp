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

#include "feature_info.hpp"
#include "image_buffer.hpp"
#include "mj_data_writer.hpp"
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
  DataFormat dataFormat{DataFormat::None};
};

struct EpisodeFrame {
  double timestamp;
  int frameIndex;

  // Joint feedback
  Eigen::VectorXd qFb;
  Eigen::VectorXd qdFb;
  Eigen::VectorXd qtauFb;

  // Joint commands
  Eigen::VectorXd qCmd;
  Eigen::VectorXd qdCmd;
  Eigen::VectorXd qtauCmd;

  // EE feedback as 7D vectors (x, y, z, qx, qy, qz, qw)
  std::vector<Eigen::Matrix<double, 7, 1>> eePose7d;

  // EE commands as 7D vectors (x, y, z, qx, qy, qz, qw)
  std::vector<Eigen::Matrix<double, 7, 1>> eePoseCmd7d;

  // Gripper
  std::vector<double> gripperPositions;  // feedback
  std::vector<double> gripperCommands;   // commands

  // Sensor data: key -> values
  std::map<std::string, Eigen::VectorXd> sensorData;

  // Metadata
  int64_t taskIndex{0};
};

class MujocoInterface;

/**
 * @class MujocoRecorder
 * @brief Records robot data and camera frames to HDF5/Parquet/MP4 datasets
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
  void loadRecorderConfig();
  void initDirectories();
  void writeInfoJson();
  void writeEpisodeData();
  void encodeEpisodeVideos();
  void appendEpisodeMetadata(int episodeLength);
  void appendTaskMetadata(const std::string &task);
  int getEpisodeChunk(int episodeIndex) const;
  std::string getDataFilePath(int episodeIndex) const;
  std::string getVideoFilePath(int episodeIndex, const std::string &cameraKey) const;

  std::map<std::string, data::CameraConfig> buildCameraConfigMap() const;

  MujocoInterface &mj_;

  RecorderConfig config_;
  std::unique_ptr<IDataWriter> dataWriter_;
  std::map<std::string, std::unique_ptr<VideoEncoder>> videoEncoders_;

  std::vector<EpisodeFrame> frameBuffer_;
  std::map<std::string, std::vector<data::ImageFrame>> imageBuffers_;
  rynn::FeatureMap features_;
  std::map<std::string, int> taskToIndex_;

  // EE names cache for per-EE feature key generation
  std::vector<std::string> eeNames_;
  int numEE_{0};
  int mdof_{0};
  int adof_{0};

  bool isRecording_{false};
  int currentEpisodeIndex_{0};
  int totalFrames_{0};
  int totalEpisodes_{0};
  int totalTasks_{0};
  int64_t globalIndex_{0};
  std::string currentTask_;

  std::vector<data::CameraConfig> cameraConfigs_;
};

} // namespace mujoco
