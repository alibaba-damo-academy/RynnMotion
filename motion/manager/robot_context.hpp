#pragma once

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "feature_info.hpp"

namespace rynn {

class RobotManager;

/**
 * @class RobotContext
 * @brief Auto-generates dataset feature schemas from RobotManager
 *
 * Mirrors Python RobotContext: produces identical feature keys, shapes,
 * dtypes, and names for any given MJCF model. Used by MujocoRecorder
 * and mjcfDatasetExe --schema / --empty-dataset / --ros2-config.
 */
class RobotContext {
public:
  explicit RobotContext(const RobotManager &rm);

  FeatureMap getHwFeatures(
      const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs = {},
      bool useVideos = true) const;

  FeatureMap getRecordingFeatures(
      const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs = {},
      bool useVideos = true) const;

  std::string toSchemaJson(const FeatureMap &features) const;
  std::string toInfoJson(const FeatureMap &features, int fps,
                         const std::string &robotType = "",
                         bool useVideos = true) const;

  void writeEmptyDataset(const std::string &outputDir, int fps,
                         const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs = {},
                         bool useVideos = true) const;

  std::string toRos2ConfigYaml(const std::string &topicPrefix = "",
                               const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs = {}) const;

private:
  void addEEPoseFeatures(FeatureMap &features, const std::string &prefix) const;
  void addCameraPoseFeatures(FeatureMap &features,
                             const std::map<std::string, std::tuple<int, int, int>> &cameraConfigs) const;
  void addSensorFeatures(FeatureMap &features) const;
  void addMetadataFeatures(FeatureMap &features) const;

  int mdof_;
  int adof_;
  int numEE_;
  bool isDexHand_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> eeNames_;
  std::vector<std::string> cameraNames_;

  struct SensorGroup {
    std::string name;
    int dim;
  };
  std::map<std::string, std::vector<SensorGroup>> sensors_;
};

} // namespace rynn
