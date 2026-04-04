#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#ifdef __APPLE__
#include "mujoco.h"
#else
#include "mujoco/mujoco.h"
#endif

#include "debug_config.hpp"
#include "discovery.hpp"
#include "robot_context.hpp"
#include "robot_manager.hpp"

namespace {

std::string actuatorModeToString(rynn::ActuatorMode mode) {
  switch (mode) {
  case rynn::ActuatorMode::kGeneral: return "general";
  case rynn::ActuatorMode::kPosition: return "position";
  case rynn::ActuatorMode::kVelocity: return "velocity";
  case rynn::ActuatorMode::kTorque: return "torque";
  case rynn::ActuatorMode::kIntVelocity: return "intvelocity";
  case rynn::ActuatorMode::kDamper: return "damper";
  case rynn::ActuatorMode::kCylinder: return "cylinder";
  case rynn::ActuatorMode::kMuscle: return "muscle";
  case rynn::ActuatorMode::kAdhesion: return "adhesion";
  case rynn::ActuatorMode::kPlugin: return "plugin";
  }
  return "unknown";
}

template <typename T>
std::string formatVec(const std::vector<T> &v) {
  std::string result = "[";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0) result += ", ";
    if constexpr (std::is_same_v<T, std::string>) {
      result += v[i];
    } else {
      result += std::to_string(v[i]);
    }
  }
  result += "]";
  return result;
}

std::string formatEigenVec(const Eigen::VectorXd &v, int precision = 4) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << "[";
  for (int i = 0; i < v.size(); ++i) {
    if (i > 0) oss << ", ";
    oss << v[i];
  }
  oss << "]";
  return oss.str();
}

void printRobotInfo(const rynn::DiscoveredRobotInfo &robotInfo, bool verbose) {
  rynn::RobotManager rm(robotInfo.number);

  int mdof = rm.getMotionDOF();
  int adof = rm.getActionDOF();
  int numEE = rm.getNumEndEffectors();
  int totalNu = mdof + numEE * adof;

  std::cout << "\n=== Robot: " << robotInfo.canonicalName << " (#" << robotInfo.number << ") ===" << std::endl;
  std::cout << "  Robot MJCF: " << robotInfo.robotMjcfPath << std::endl;
  std::cout << "  Pino MJCF:  " << robotInfo.pinocchioPath << std::endl;

  // Actuators
  std::cout << "\n  Actuators:" << std::endl;
  std::cout << "    mdof: " << mdof << ", adof: " << adof
            << ", numEE: " << numEE << ", total nu: " << totalNu << std::endl;

  auto jointIndices = rm.getJointIndices();
  auto eeIndices = rm.getEEIndices();
  auto jointNames = rm.getJointNames();
  auto eeNames = rm.getEENames();
  auto eeJointIndices = rm.getEEJointIndices();
  auto actuatorModes = rm.getActuatorModes();

  std::cout << "    Joint Indices:    " << formatVec(jointIndices) << std::endl;
  std::cout << "    Joint Names:      " << formatVec(jointNames) << std::endl;

  if (!eeIndices.empty()) {
    std::cout << "    EE Indices:       " << formatVec(eeIndices) << std::endl;
    std::cout << "    EE Names:         " << formatVec(eeNames) << std::endl;
    std::cout << "    EE Joint Indices: " << formatVec(eeJointIndices) << std::endl;
  }

  // Summarize actuator modes
  std::map<std::string, int> modeCounts;
  for (const auto &mode : actuatorModes) {
    modeCounts[actuatorModeToString(mode)]++;
  }
  std::string modesStr;
  for (const auto &[name, count] : modeCounts) {
    if (!modesStr.empty()) modesStr += ", ";
    modesStr += name + " x" + std::to_string(count);
  }
  std::cout << "    Actuator Modes:   [" << modesStr << "]" << std::endl;

  // Sensors
  auto forceSensors = rm.getForceSensors();
  auto torqueSensors = rm.getTorqueSensors();
  auto gyroSensors = rm.getGyroSensors();
  auto accelSensors = rm.getAccelerometerSensors();
  auto rangefinderSensors = rm.getRangefinderSensors();
  auto framePosSensors = rm.getFramePosSensors();
  auto frameQuatSensors = rm.getFrameQuatSensors();
  auto cameraNames = rm.getCameraNames();

  bool hasSensors = !forceSensors.empty() || !torqueSensors.empty() ||
                    !gyroSensors.empty() || !accelSensors.empty() ||
                    !rangefinderSensors.empty() || !framePosSensors.empty() ||
                    !frameQuatSensors.empty() || !cameraNames.empty();

  if (hasSensors) {
    std::cout << "\n  Sensors:" << std::endl;

    if (!cameraNames.empty()) {
      std::cout << "    Cameras: " << cameraNames.size() << " (" << formatVec(cameraNames).substr(1, formatVec(cameraNames).size() - 2) << ")" << std::endl;
    } else {
      std::cout << "    Cameras: 0" << std::endl;
    }

    if (!forceSensors.empty() || !torqueSensors.empty()) {
      std::cout << "    Force/Torque: force=" << forceSensors.size()
                << ", torque=" << torqueSensors.size() << std::endl;
    }
    if (!gyroSensors.empty() || !accelSensors.empty()) {
      std::cout << "    IMU: gyro=" << gyroSensors.size()
                << ", accel=" << accelSensors.size() << std::endl;
    }
    if (!framePosSensors.empty() || !frameQuatSensors.empty()) {
      std::cout << "    Frame: pos=" << framePosSensors.size()
                << ", quat=" << frameQuatSensors.size() << std::endl;
    }
    if (!rangefinderSensors.empty()) {
      std::cout << "    Rangefinder: " << rangefinderSensors.size() << std::endl;
    }
  }

  // Keyframes
  auto keyframeNames = rm.getKeyframeNames();
  int numKeyframes = rm.getNumKeyframes();
  if (numKeyframes > 0) {
    std::cout << "\n  Keyframes: " << numKeyframes << " (" << formatVec(keyframeNames).substr(1, formatVec(keyframeNames).size() - 2) << ")" << std::endl;
  }

  // Sites
  int numSites = rm.getNumSites();
  if (numSites > 0) {
    std::cout << "  Sites: " << numSites << std::endl;
  }

  // Scenes
  if (!robotInfo.scenes.empty()) {
    std::cout << "  Scenes: " << robotInfo.scenes.size();
    std::string sceneList;
    for (const auto &scene : robotInfo.scenes) {
      if (!sceneList.empty()) sceneList += ", ";
      sceneList += scene.name;
    }
    std::cout << " (" << sceneList << ")" << std::endl;
  }

  // Verbose details
  if (verbose) {
    std::cout << "\n  [Verbose Details]" << std::endl;

    // Joint limits
    auto qPosMin = rm.getJointPosMin();
    auto qPosMax = rm.getJointPosMax();
    auto qVelMax = rm.getJointVelMax();
    auto qTorqueMax = rm.getJointTorqueMax();

    std::cout << "    Joint Limits (pos min):    " << formatEigenVec(qPosMin) << std::endl;
    std::cout << "    Joint Limits (pos max):    " << formatEigenVec(qPosMax) << std::endl;
    std::cout << "    Joint Limits (vel max):    " << formatEigenVec(qVelMax) << std::endl;
    std::cout << "    Joint Limits (torque max): " << formatEigenVec(qTorqueMax) << std::endl;

    // Gains
    auto simKp = rm.getSimKp();
    auto simKd = rm.getSimKd();
    std::cout << "    Sim Kp: " << formatEigenVec(simKp, 2) << std::endl;
    std::cout << "    Sim Kd: " << formatEigenVec(simKd, 2) << std::endl;

    // Keyframe values
    for (int i = 0; i < numKeyframes; ++i) {
      auto kf = rm.getKeyframe(i);
      std::string name = (i < static_cast<int>(keyframeNames.size())) ? keyframeNames[i] : "?";
      std::cout << "    Keyframe[" << i << "] " << name << ": " << formatEigenVec(kf) << std::endl;
    }

    // EE ranges
    auto eeRanges = rm.getEERanges();
    for (size_t i = 0; i < eeRanges.size(); ++i) {
      std::cout << "    EE[" << i << "] range: [" << eeRanges[i].first << ", " << eeRanges[i].second << "]" << std::endl;
    }

    // Sensor details
    auto printSensorDetails = [](const std::string &label, const std::vector<rynn::SensorInfo> &sensors) {
      for (const auto &s : sensors) {
        std::cout << "    " << label << ": " << s.name << " (adr=" << s.index << ", dim=" << s.dim << ")" << std::endl;
      }
    };

    printSensorDetails("Force sensor", forceSensors);
    printSensorDetails("Torque sensor", torqueSensors);
    printSensorDetails("Gyro sensor", gyroSensors);
    printSensorDetails("Accel sensor", accelSensors);
    printSensorDetails("Rangefinder", rangefinderSensors);
    printSensorDetails("Frame pos", framePosSensors);
    printSensorDetails("Frame quat", frameQuatSensors);

    // Site names
    if (rm.hasSites()) {
      auto siteNames = rm.getSiteNames();
      std::cout << "    Site names: " << formatVec(siteNames) << std::endl;
    }
  }

  std::cout << std::endl;
}

void printSchema(const std::string &mjcfPath) {
  rynn::RobotManager rm(mjcfPath);
  rynn::RobotContext ctx(rm);
  auto features = ctx.getRecordingFeatures();
  std::cout << ctx.toSchemaJson(features);
}

void createEmptyDataset(const std::string &mjcfPath, const std::string &robotName,
                        const std::string &outputDir, int fps) {
  rynn::RobotManager rm(mjcfPath);
  rynn::RobotContext ctx(rm);
  ctx.writeEmptyDataset(outputDir, fps);
  std::cout << "Created empty LeRobot v3.0 dataset at: " << outputDir << std::endl;
}

void generateRos2Config(const std::string &mjcfPath, const std::string &robotName,
                        const std::string &outputPath, const std::string &topicPrefix) {
  rynn::RobotManager rm(mjcfPath);
  rynn::RobotContext ctx(rm);
  std::string prefix = topicPrefix.empty() ? "/" + robotName + "_driver" : topicPrefix;
  std::string yaml = ctx.toRos2ConfigYaml(prefix);

  std::ofstream f(outputPath);
  f << yaml;
  std::cout << "Generated ROS2 config at: " << outputPath << std::endl;
}

std::string generateTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&time, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M");
  return oss.str();
}

std::string resolveScenePath(const rynn::DiscoveredRobotInfo &robotInfo, const std::string &sceneName) {
  if (sceneName.empty()) {
    if (robotInfo.scenes.empty()) {
      throw std::runtime_error("No scenes found for robot " + robotInfo.canonicalName);
    }
    return robotInfo.scenes[0].fullPath;
  }
  auto scene = rynn::SceneDiscovery::getScene(robotInfo.scenes, sceneName);
  return scene.fullPath;
}

void printUsage(const std::vector<rynn::DiscoveredRobotInfo> &robots) {
  std::cout << "Usage: mjcfDatasetExe [options] <robot>" << std::endl;
  std::cout << "       mjcfDatasetExe --all" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  -v, --verbose              Show detailed limits, gains, keyframe values, sensor addresses" << std::endl;
  std::cout << "  -s, --schema               Print recording feature schema as JSON" << std::endl;
  std::cout << "  --scene <name>             Scene to use for dataset/config generation (has actuators, sensors, cameras)" << std::endl;
  std::cout << "  --empty-dataset <dir>      Create empty LeRobot v3.0 dataset at <dir> (requires --scene)" << std::endl;
  std::cout << "  --ros2-config <file>       Generate ROS2 recording config YAML to <file> (requires --scene)" << std::endl;
  std::cout << "  --fps <int>                FPS for dataset (default: 30)" << std::endl;
  std::cout << "  --topic-prefix <str>       ROS2 topic prefix (default: /<robot>_driver)" << std::endl;
  std::cout << "  --no-output                With --schema: print only, don't create files in record/" << std::endl;
  std::cout << "  -h, --help                 Show this help" << std::endl;
  std::cout << "  --all                      Inspect all discovered robots" << std::endl;
  std::cout << std::endl;
  std::cout << "Available robots:" << std::endl;

  for (const auto &robot : robots) {
    std::cout << "  " << std::setw(3) << robot.number << ". " << robot.canonicalName;
    if (!robot.aliases.empty()) {
      std::cout << " (aliases: ";
      for (size_t i = 0; i < std::min(robot.aliases.size(), size_t(3)); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << robot.aliases[i];
      }
      if (robot.aliases.size() > 3) std::cout << ", ...";
      std::cout << ")";
    }
    std::cout << std::endl;
  }
}

} // namespace

int main(int argc, char *argv[]) {
  auto &discovery = rynn::RobotDiscovery::getInstance();
  discovery.scanRobots(MODEL_DIR);

  auto allRobots = discovery.getAllRobots();

  bool verbose = false;
  bool inspectAll = false;
  bool showSchema = false;
  bool noOutput = false;
  std::string robotQuery;
  std::string sceneName;
  std::string emptyDatasetDir;
  std::string ros2ConfigPath;
  std::string topicPrefix;
  int fps = 30;

  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "-v" || arg == "--verbose") {
      verbose = true;
    } else if (arg == "-s" || arg == "--schema") {
      showSchema = true;
    } else if (arg == "-h" || arg == "--help") {
      printUsage(allRobots);
      return 0;
    } else if (arg == "--all") {
      inspectAll = true;
    } else if (arg == "--scene" && i + 1 < argc) {
      sceneName = argv[++i];
    } else if (arg == "--empty-dataset" && i + 1 < argc) {
      emptyDatasetDir = argv[++i];
    } else if (arg == "--ros2-config" && i + 1 < argc) {
      ros2ConfigPath = argv[++i];
    } else if (arg == "--fps" && i + 1 < argc) {
      fps = std::stoi(argv[++i]);
    } else if (arg == "--topic-prefix" && i + 1 < argc) {
      topicPrefix = argv[++i];
    } else if (arg == "--no-output") {
      noOutput = true;
    } else if (arg[0] != '-') {
      robotQuery = arg;
    }
  }

  if (!inspectAll && robotQuery.empty()) {
    printUsage(allRobots);
    return 1;
  }

  if (inspectAll) {
    std::cout << "Inspecting all " << allRobots.size() << " discovered robots..." << std::endl;
    int success = 0, failed = 0;

    for (const auto &robotInfo : allRobots) {
      if (robotInfo.robotMjcfPath.empty()) {
        std::cerr << "  Skipping " << robotInfo.canonicalName << " (#" << robotInfo.number << "): no robot MJCF found" << std::endl;
        failed++;
        continue;
      }

      try {
        printRobotInfo(robotInfo, verbose);
        success++;
      } catch (const std::exception &e) {
        std::cerr << "  FAILED " << robotInfo.canonicalName << " (#" << robotInfo.number << "): " << e.what() << std::endl;
        failed++;
      }
    }

    std::cout << "\nSummary: " << success << " parsed, " << failed << " failed, " << allRobots.size() << " total" << std::endl;
    return 0;
  }

  // Single robot query
  try {
    auto robotInfo = discovery.getRobotInfo(robotQuery);

    if (robotInfo.robotMjcfPath.empty()) {
      std::cerr << "Error: Robot " << robotQuery << " has no robot MJCF path" << std::endl;
      return 1;
    }

    if (!emptyDatasetDir.empty() || !ros2ConfigPath.empty() || (showSchema && !sceneName.empty())) {
      // Dataset/config/schema-with-scene operations use scene XML (has actuators, sensors, cameras)
      std::string scenePath = resolveScenePath(robotInfo, sceneName);
      std::cerr << "Using scene: " << scenePath << std::endl;

      if (!emptyDatasetDir.empty()) {
        createEmptyDataset(scenePath, robotInfo.canonicalName, emptyDatasetDir, fps);
      } else if (!ros2ConfigPath.empty()) {
        generateRos2Config(scenePath, robotInfo.canonicalName, ros2ConfigPath, topicPrefix);
      } else {
        // --schema --scene: print to stdout AND auto-create record/ output
        printSchema(scenePath);

        if (!noOutput) {
          std::string ts = generateTimestamp();
          std::string dirName = robotInfo.canonicalName + "_" + sceneName + "_" + ts;
          std::string recordDir = std::string(MODEL_DIR) + "/../record/" + dirName;
          createEmptyDataset(scenePath, robotInfo.canonicalName, recordDir, fps);
          generateRos2Config(scenePath, robotInfo.canonicalName, recordDir + "/ros2_config.yaml", topicPrefix);
        }
      }
    } else if (showSchema) {
      // Schema without --scene uses robot MJCF (backward compat)
      printSchema(robotInfo.robotMjcfPath);
    } else {
      printRobotInfo(robotInfo, verbose);
    }
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cout << std::endl;
    printUsage(allRobots);
    return 1;
  }

  return 0;
}
