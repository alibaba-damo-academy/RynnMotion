#include "robot_context.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "discovery.hpp"
#include "robot_manager.hpp"

using namespace rynn;

static int testsFailed = 0;
static int testsPassed = 0;

#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::cerr << "  FAIL: " << (msg) << " [" << __FILE__ << ":"             \
                << __LINE__ << "]" << std::endl;                               \
      ++testsFailed;                                                           \
    } else {                                                                   \
      ++testsPassed;                                                           \
    }                                                                          \
  } while (0)

#define CHECK_EQ(a, b, msg)                                                    \
  do {                                                                         \
    if ((a) != (b)) {                                                          \
      std::cerr << "  FAIL: " << (msg) << " (got " << (a) << ", expected "    \
                << (b) << ") [" << __FILE__ << ":" << __LINE__ << "]"          \
                << std::endl;                                                  \
      ++testsFailed;                                                           \
    } else {                                                                   \
      ++testsPassed;                                                           \
    }                                                                          \
  } while (0)

// =========================================================================
// 1. SO101 HW Features
// =========================================================================

void testSo101HwFeatures() {
  std::cout << "\n=== SO101 HW Features ===" << std::endl;

  auto &discovery = RobotDiscovery::getInstance();
  discovery.scanRobots(MODEL_DIR);

  auto info = discovery.getRobotInfo("so101");
  RobotManager rm(info.number);
  RobotContext ctx(rm);

  auto features = ctx.getHwFeatures();

  // observation.state
  CHECK(features.count("observation.state") > 0, "observation.state exists");
  CHECK_EQ(features["observation.state"].shape[0], 5, "observation.state shape=5");
  CHECK_EQ(features["observation.state"].dtype, std::string("float32"),
           "observation.state dtype=float32");
  CHECK_EQ(static_cast<int>(features["observation.state"].names.size()), 5,
           "observation.state has 5 names");

  // action
  CHECK(features.count("action") > 0, "action exists");
  CHECK_EQ(features["action"].shape[0], 6, "action shape=6");
  CHECK_EQ(static_cast<int>(features["action"].names.size()), 6,
           "action has 6 names (joints + gripper)");

  // observation.state.gripper
  CHECK(features.count("observation.state.gripper") > 0,
        "observation.state.gripper exists");
  CHECK_EQ(features["observation.state.gripper"].shape[0], 1,
           "gripper shape=1");

  std::cout << "  HW features keys:";
  for (const auto &[key, info] : features) {
    std::cout << " " << key;
  }
  std::cout << std::endl;
}

// =========================================================================
// 2. SO101 Recording Features
// =========================================================================

void testSo101RecordingFeatures() {
  std::cout << "\n=== SO101 Recording Features ===" << std::endl;

  auto &discovery = RobotDiscovery::getInstance();
  auto info = discovery.getRobotInfo("so101");
  RobotManager rm(info.number);
  RobotContext ctx(rm);

  auto hwFeatures = ctx.getHwFeatures();
  auto recFeatures = ctx.getRecordingFeatures();

  // Recording features should be a superset of hw features
  for (const auto &[key, fi] : hwFeatures) {
    CHECK(recFeatures.count(key) > 0,
          "recording features contain hw key: " + key);
  }

  // Verify extra recording features
  CHECK(recFeatures.count("observation.velocity") > 0,
        "recording has observation.velocity");
  CHECK(recFeatures.count("observation.torque") > 0,
        "recording has observation.torque");
  CHECK(recFeatures.count("action.velocity") > 0,
        "recording has action.velocity");
  CHECK(recFeatures.count("action.torque") > 0,
        "recording has action.torque");
  CHECK(recFeatures.count("timestamp") > 0, "recording has timestamp");
  CHECK(recFeatures.count("frame_index") > 0, "recording has frame_index");
  CHECK(recFeatures.count("episode_index") > 0,
        "recording has episode_index");
  CHECK(recFeatures.count("observation.end_effector_pose") > 0,
        "recording has observation.end_effector_pose");
  CHECK(recFeatures.count("action.end_effector_pose") > 0,
        "recording has action.end_effector_pose");

  // EE pose shape = 7
  CHECK_EQ(recFeatures["observation.end_effector_pose"].shape[0], 7,
           "ee pose shape=7");

  std::cout << "  Recording features: " << recFeatures.size() << " keys"
            << std::endl;
}

// =========================================================================
// 3. Multi-Robot Iteration
// =========================================================================

void testMultiRobot() {
  std::cout << "\n=== Multi-Robot Iteration ===" << std::endl;

  auto &discovery = RobotDiscovery::getInstance();

  struct RobotExpected {
    std::string name;
    int mdof;
    int adof;
  };

  std::vector<RobotExpected> robots = {
      {"fr3", 7, 1},    {"ur5e", 6, 1},    {"piper", 6, 1},
      {"rm75", 7, 1},   {"so101", 5, 1},   {"rizon4s", 7, 0},
      {"dm6dof", 6, 0}, {"openarm", 7, 1},  {"so101_6dof", 6, 1},
  };

  for (const auto &expected : robots) {
    try {
      auto info = discovery.getRobotInfo(expected.name);
      RobotManager rm(info.number);
      RobotContext ctx(rm);

      auto features = ctx.getHwFeatures();

      CHECK(rm.getMotionDOF() > 0,
            expected.name + ": mdof > 0");
      CHECK_EQ(rm.getMotionDOF(), expected.mdof,
               expected.name + ": mdof");
      CHECK_EQ(rm.getActionDOF(), expected.adof,
               expected.name + ": adof");

      int actionDim = rm.getMotionDOF() + rm.getActionDOF();
      CHECK_EQ(features["action"].shape[0], actionDim,
               expected.name + ": action dim = mdof + adof");
      CHECK(!features.empty(), expected.name + ": features non-empty");

      std::cout << "  " << expected.name << ": mdof=" << rm.getMotionDOF()
                << ", adof=" << rm.getActionDOF()
                << ", features=" << features.size() << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "  FAIL: " << expected.name << " threw: " << e.what()
                << std::endl;
      ++testsFailed;
    }
  }
}

// =========================================================================
// 4. Camera Config
// =========================================================================

void testCameraConfig() {
  std::cout << "\n=== Camera Config ===" << std::endl;

  auto &discovery = RobotDiscovery::getInstance();
  auto info = discovery.getRobotInfo("so101");
  RobotManager rm(info.number);
  RobotContext ctx(rm);

  std::map<std::string, std::tuple<int, int, int>> cameraConfigs = {
      {"front", {480, 640, 3}},
      {"wrist", {240, 320, 3}},
  };

  auto features = ctx.getHwFeatures(cameraConfigs);

  CHECK(features.count("observation.images.front") > 0,
        "has observation.images.front");
  CHECK(features.count("observation.images.wrist") > 0,
        "has observation.images.wrist");

  auto frontShape = features["observation.images.front"].shape;
  CHECK_EQ(frontShape[0], 480, "front height=480");
  CHECK_EQ(frontShape[1], 640, "front width=640");
  CHECK_EQ(frontShape[2], 3, "front channels=3");

  auto wristShape = features["observation.images.wrist"].shape;
  CHECK_EQ(wristShape[0], 240, "wrist height=240");
  CHECK_EQ(wristShape[1], 320, "wrist width=320");

  CHECK_EQ(features["observation.images.front"].dtype, std::string("video"),
           "front dtype=video (default)");

  // Test with useVideos=false
  auto featImg = ctx.getHwFeatures(cameraConfigs, false);
  CHECK_EQ(featImg["observation.images.front"].dtype, std::string("image"),
           "front dtype=image when useVideos=false");

  std::cout << "  Camera features verified" << std::endl;
}

// =========================================================================
// 5. JSON Schema Output
// =========================================================================

void testJsonSchemaOutput() {
  std::cout << "\n=== JSON Schema Output ===" << std::endl;

  auto &discovery = RobotDiscovery::getInstance();
  auto info = discovery.getRobotInfo("so101");
  RobotManager rm(info.number);
  RobotContext ctx(rm);

  auto features = ctx.getRecordingFeatures();
  std::string json = ctx.toSchemaJson(features);

  CHECK(!json.empty(), "JSON output is non-empty");
  CHECK(json.find("\"features\"") != std::string::npos,
        "JSON contains 'features' key");
  CHECK(json.find("\"observation.state\"") != std::string::npos,
        "JSON contains observation.state");
  CHECK(json.find("\"action\"") != std::string::npos,
        "JSON contains action");
  CHECK(json.find("\"float32\"") != std::string::npos,
        "JSON contains float32 dtype");
  CHECK(json.find("\"timestamp\"") != std::string::npos,
        "JSON contains timestamp");

  // Verify it looks like valid JSON (starts with { and ends with })
  size_t first = json.find_first_not_of(" \t\n\r");
  size_t last = json.find_last_not_of(" \t\n\r");
  CHECK(first != std::string::npos && json[first] == '{', "JSON starts with {");
  CHECK(last != std::string::npos && json[last] == '}', "JSON ends with }");

  std::cout << "  JSON schema: " << json.size() << " bytes" << std::endl;
}

// =========================================================================
// Main
// =========================================================================

int main() {
  std::cout << "=== RobotContext Test Suite ===" << std::endl;

  try {
    testSo101HwFeatures();
    testSo101RecordingFeatures();
    testMultiRobot();
    testCameraConfig();
    testJsonSchemaOutput();
  } catch (const std::exception &e) {
    std::cerr << "\nFATAL: Unhandled exception: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "\n=== Results ===" << std::endl;
  std::cout << "  Passed: " << testsPassed << std::endl;
  std::cout << "  Failed: " << testsFailed << std::endl;

  if (testsFailed > 0) {
    std::cout << "\n  *** SOME TESTS FAILED ***" << std::endl;
    return 1;
  }

  std::cout << "\n  All tests passed!" << std::endl;
  return 0;
}
