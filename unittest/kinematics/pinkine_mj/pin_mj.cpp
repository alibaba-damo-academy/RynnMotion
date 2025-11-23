#include "pin_mj.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <vector>

#ifndef MODEL_DIR
#define MODEL_DIR "../../../models"
#endif

namespace test {

HybridKinematicsTest::HybridKinematicsTest(int robotNumber) :
    mjModel_(nullptr),
    mjData_(nullptr),
    glfwWindow_(nullptr),
    enableRendering_(false),
    renderingInitialized_(false),
    robotNumber_(robotNumber),
    posTolerance_(1e-3),
    quatTolerance_(1e-2),
    jacoTolerance_(1e-3),
    numJoints_(0),
    testsPassed_(0),
    testsFailed_(0),
    maxPosError_(0.0),
    maxQuatError_(0.0),
    eeSiteId_(-1),
    simTimestep_(0.001),
    multiSiteMode_(false) {
}

HybridKinematicsTest::HybridKinematicsTest(int robotNumber, const std::vector<std::string> &site_names) :
    mjModel_(nullptr),
    mjData_(nullptr),
    glfwWindow_(nullptr),
    enableRendering_(false),
    renderingInitialized_(false),
    robotNumber_(robotNumber),
    posTolerance_(1e-3),
    quatTolerance_(1e-2),
    jacoTolerance_(1e-3),
    numJoints_(0),
    testsPassed_(0),
    testsFailed_(0),
    maxPosError_(0.0),
    maxQuatError_(0.0),
    eeSiteId_(-1),
    simTimestep_(0.001),
    multiSiteMode_(true),
    siteNames_(site_names) {
}

HybridKinematicsTest::~HybridKinematicsTest() {
  closeRendering();
  if (mjData_) {
    mj_deleteData(mjData_);
  }
  if (mjModel_) {
    mj_deleteModel(mjModel_);
  }
}

bool HybridKinematicsTest::initialize() {
  if (!initializePinocchio()) {
    std::cerr << "Failed to initialize Pinocchio" << std::endl;
    return false;
  }

  if (!loadMujocoModel()) {
    std::cerr << "Failed to load MuJoCo model" << std::endl;
    return false;
  }

  if (enableRendering_) {
    if (!initRendering()) {
      std::cerr << "Failed to initialize rendering" << std::endl;
      return false;
    }
  }

  return true;
}

bool HybridKinematicsTest::initializePinocchio() {
  try {
    if (!robotManager_) {
      robotManager_ = std::make_unique<rynn::RobotManager>(robotNumber_);
      numJoints_ = robotManager_->getMotionDOF();
    }

    std::string mjcfPath = robotManager_->getPinoMJCF();
    if (!std::filesystem::exists(mjcfPath)) {
      std::cerr << "Pinocchio MJCF file not found: " << mjcfPath << std::endl;
      return false;
    }

    if (multiSiteMode_) {
      pinKine_ = std::make_unique<utils::PinKine>(mjcfPath, siteNames_);
      std::cout << "Initialized Pinocchio in multi-site mode with " << siteNames_.size() << " sites" << std::endl;
    } else {
      pinKine_ = std::make_unique<utils::PinKine>(mjcfPath, "EE");
    }

    // Get numJoints from Pinocchio model (excludes gripper)
    numJoints_ = pinKine_->getPinModel().nq;

    return true;
  } catch (const std::exception &e) {
    std::cerr << "Pinocchio initialization error: " << e.what() << std::endl;
    return false;
  }
}

bool HybridKinematicsTest::loadMujocoModel() {
  try {
    mjcfPath_ = robotManager_->getRobotMJCF();
    if (!std::filesystem::exists(mjcfPath_)) {
      std::cerr << "MJCF file not found: " << mjcfPath_ << std::endl;
      return false;
    }
    char error[1000] = "Could not load XML model";
    mjModel_ = mj_loadXML(mjcfPath_.c_str(), 0, error, 1000);
    if (!mjModel_) {
      std::cerr << "MuJoCo model loading failed: " << error << std::endl;
      return false;
    }

    mjData_ = mj_makeData(mjModel_);
    if (!mjData_) {
      std::cerr << "Failed to create MuJoCo data" << std::endl;
      return false;
    }

    mj_resetData(mjModel_, mjData_);
    simTimestep_ = mjModel_->opt.timestep;

    if (multiSiteMode_) {
      // Find all site IDs
      siteIds_.clear();
      for (const auto &siteName : siteNames_) {
        int siteId = mj_name2id(mjModel_, mjOBJ_SITE, siteName.c_str());
        if (siteId == -1) {
          std::cerr << "Could not find site: " << siteName << std::endl;
          return false;
        }
        siteIds_.push_back(siteId);
      }
      std::cout << "Found " << siteIds_.size() << " sites in MuJoCo model" << std::endl;
    } else {
      eeSiteId_ = mj_name2id(mjModel_, mjOBJ_SITE, "EE");
      if (eeSiteId_ == -1) {
        std::cerr << "Could not find EE site" << std::endl;
        return false;
      }
    }

    return true;
  } catch (const std::exception &e) {
    std::cerr << "MuJoCo loading error: " << e.what() << std::endl;
    return false;
  }
}

Eigen::VectorXd HybridKinematicsTest::genJointRef(double time, int scenario, double durationSeconds) {
  Eigen::VectorXd qRef(numJoints_);

  switch (scenario) {
  case 0: {
    qRef.setZero();
    std::cout << "qRef: " << qRef.transpose() << std::endl;
    break;
  }
  case 1: {
    double t = std::min(time / durationSeconds, 1.0);
    double theta = -t * M_PI / 4;
    qRef.setConstant(theta);
    break;
  }
  default:
    std::cerr << "Unknown scenario: " << scenario << ", using scenario 0 (zero angles)" << std::endl;
    qRef.setZero();
    break;
  }

  return qRef;
}

bool HybridKinematicsTest::getMujocoEEPose(const Eigen::VectorXd &qRef, Eigen::Vector3d &pos, Eigen::Vector4d &quat) {
  for (int i = 0; i < numJoints_; ++i) {
    mjData_->ctrl[i] = qRef[i];
  }
  mj_step(mjModel_, mjData_);
  mj_forward(mjModel_, mjData_);
  const double *eeSitePos = mjData_->site_xpos + 3 * eeSiteId_;
  const double *eeSiteRotm = mjData_->site_xmat + 9 * eeSiteId_;
  double eeSiteQuat[4];
  mju_mat2Quat(eeSiteQuat, eeSiteRotm);

  pos = Eigen::Vector3d(eeSitePos[0], eeSitePos[1], eeSitePos[2]);
  quat = Eigen::Vector4d(eeSiteQuat[0], eeSiteQuat[1], eeSiteQuat[2], eeSiteQuat[3]);

  return true;
}

bool HybridKinematicsTest::getPinEEPose(const Eigen::VectorXd &qRef, Eigen::Vector3d &pos, Eigen::Vector4d &quat) {
  try {
    Eigen::VectorXd qFb(numJoints_);
    for (int i = 0; i < numJoints_; ++i) {
      qFb[i] = mjData_->qpos[i];
    }
    pinKine_->update(qFb);
    pos = pinKine_->getEEPos();
    quat = pinKine_->getEEQuat();
    return true;
  } catch (const std::exception &e) {
    std::cerr << "Pinocchio computation error: " << e.what() << std::endl;
    return false;
  }
}

bool HybridKinematicsTest::comparePoses(const Eigen::Vector3d &pos1, const Eigen::Vector4d &quat1,
                                        const Eigen::Vector3d &pos2, const Eigen::Vector4d &quat2) {
  double posError = (pos1 - pos2).norm();
  double dot = std::abs(quat1.dot(quat2));
  dot = std::min(1.0, dot);
  double quatError = 2.0 * std::acos(dot);

  maxPosError_ = std::max(maxPosError_, posError);
  maxQuatError_ = std::max(maxQuatError_, quatError);

  return (posError < posTolerance_) && (quatError < quatTolerance_);
}

bool HybridKinematicsTest::getMujocoJaco(const Eigen::VectorXd &qRef, Eigen::MatrixXd &jaco) {
  // Set joint positions
  for (int i = 0; i < numJoints_; ++i) {
    mjData_->ctrl[i] = qRef[i];
  }
  mj_step(mjModel_, mjData_);
  mj_forward(mjModel_, mjData_);

  // Allocate Jacobian matrix for arm joints only (6 rows x numJoints_ columns)
  jaco.resize(6, numJoints_);

  // MuJoCo outputs row-major 3xnv matrices, allocate temporary buffers for full Jacobian
  std::vector<mjtNum> jacp(3 * mjModel_->nv); // translational Jacobian (3 x nv)
  std::vector<mjtNum> jacr(3 * mjModel_->nv); // rotational Jacobian (3 x nv)

  // Get full Jacobians from MuJoCo (outputs in row-major format)
  mj_jacSite(mjModel_, mjData_, jacp.data(), jacr.data(), eeSiteId_);

  // Copy from row-major MuJoCo format to column-major Eigen format
  // Only extract the first numJoints_ columns (arm DOF, excluding gripper)
  // MuJoCo outputs: jacp[row * nv + col] for translational (rows 0-2)
  //                 jacr[row * nv + col] for rotational (rows 0-2)
  // Eigen needs: jaco(row, col) for linear (rows 0-2) and angular (rows 3-5)
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < numJoints_; ++col) {
      jaco(row, col) = jacp[row * mjModel_->nv + col];     // Linear (rows 0-2)
      jaco(row + 3, col) = jacr[row * mjModel_->nv + col]; // Angular (rows 3-5)
    }
  }

  return true;
}

bool HybridKinematicsTest::getPinJaco(const Eigen::VectorXd &qRef, Eigen::MatrixXd &jaco) {
  try {
    Eigen::VectorXd qFb(numJoints_);
    for (int i = 0; i < numJoints_; ++i) {
      qFb[i] = mjData_->qpos[i];
    }
    pinKine_->update(qFb);
    jaco = pinKine_->getEEJaco();
    return true;
  } catch (const std::exception &e) {
    std::cerr << "Pinocchio Jacobian computation error: " << e.what() << std::endl;
    return false;
  }
}

bool HybridKinematicsTest::compareJacobians(const Eigen::MatrixXd &jaco1, const Eigen::MatrixXd &jaco2) {
  if (jaco1.rows() != jaco2.rows() || jaco1.cols() != jaco2.cols()) {
    std::cerr << "Jacobian dimension mismatch: (" << jaco1.rows() << "x" << jaco1.cols()
              << ") vs (" << jaco2.rows() << "x" << jaco2.cols() << ")" << std::endl;
    return false;
  }

  double jacoError = (jaco1 - jaco2).norm();
  return jacoError < jacoTolerance_;
}

bool HybridKinematicsTest::runKineComparison(int scenario, bool compareJacobian) {
  double durationSeconds = (scenario == 0) ? 5 * simTimestep_ : 10.0;
  int maxSteps = (scenario == 0) ? 5 : INT_MAX;

  testsPassed_ = 0;
  testsFailed_ = 0;
  maxPosError_ = 0.0;
  maxQuatError_ = 0.0;
  targetJointAngles_.resize(0);

  int stepCount = 0;
  double currentTime = 0.0;

  while (currentTime < durationSeconds && stepCount < maxSteps) {
    Eigen::VectorXd qRef = genJointRef(currentTime, scenario, durationSeconds);
    bool mjSuccess, pinSuccess;
    bool mjPinMatch;

    if (compareJacobian) {
      // Jacobian comparison
      Eigen::MatrixXd jacoMJ, jacoPin;
      mjSuccess = getMujocoJaco(qRef, jacoMJ);
      pinSuccess = getPinJaco(qRef, jacoPin);

      if (!mjSuccess || !pinSuccess) {
        testsFailed_++;
        std::cerr << "Step " << stepCount << " failed: Jacobian computation error" << std::endl;
      } else {
        mjPinMatch = compareJacobians(jacoMJ, jacoPin);

        bool shouldPrint = (stepCount == 0) || (scenario == 0) || (std::fmod(currentTime, 1.0) < simTimestep_) || (currentTime + simTimestep_ >= durationSeconds);

        if (shouldPrint) {
          Eigen::VectorXd qFb(mjModel_->nq);
          for (int j = 0; j < mjModel_->nq; ++j) {
            qFb[j] = mjData_->qpos[j];
          }
          std::cout << "--- t=" << std::fixed << std::setprecision(2) << currentTime << "s ---" << std::endl;
          std::cout << "Joint angles (rad): " << std::setprecision(6) << qFb.transpose() << std::endl;

          std::cout << "[MuJoCo   ] Jacobian (6x" << jacoMJ.cols() << "):" << std::endl;
          std::cout << jacoMJ << std::endl;

          std::cout << "[Pinocchio] Jacobian (6x" << jacoPin.cols() << "):" << std::endl;
          std::cout << jacoPin << std::endl;

          double mjPinJacoError = (jacoMJ - jacoPin).norm();
          std::cout << "Pin-MJ Jacobian diff: " << std::scientific << std::setprecision(3) << mjPinJacoError << std::endl;
          std::cout << std::endl;
        }

        if (mjPinMatch) {
          testsPassed_++;
        } else {
          testsFailed_++;
        }
      }
    } else {
      // Pose comparison (default)
      Eigen::Vector3d eePosMJ, eePosPin;
      Eigen::Vector4d eeQuatMJ, eeQuatPin;

      mjSuccess = getMujocoEEPose(qRef, eePosMJ, eeQuatMJ);
      pinSuccess = getPinEEPose(qRef, eePosPin, eeQuatPin);

      if (!mjSuccess || !pinSuccess) {
        testsFailed_++;
        std::cerr << "Step " << stepCount << " failed: computation error" << std::endl;
      } else {
        mjPinMatch = comparePoses(eePosMJ, eeQuatMJ, eePosPin, eeQuatPin);

        bool shouldPrint = (stepCount == 0) || (scenario == 0) || (std::fmod(currentTime, 1.0) < simTimestep_) || (currentTime + simTimestep_ >= durationSeconds);

        if (shouldPrint) {
          Eigen::VectorXd qFb(mjModel_->nq);
          for (int j = 0; j < mjModel_->nq; ++j) {
            qFb[j] = mjData_->qpos[j];
          }
          std::cout << "--- t=" << std::fixed << std::setprecision(2) << currentTime << "s ---" << std::endl;
          std::cout << "Joint angles (rad): " << std::setprecision(6) << qFb.transpose() << std::endl;
          std::cout << "[MuJoCo   ] Pos: [ " << eePosMJ[0] << "  " << eePosMJ[1] << "  " << eePosMJ[2] << " ]" << std::endl;
          std::cout << "[MuJoCo   ] Quat: [ " << eeQuatMJ.w() << "  " << eeQuatMJ.x() << "  " << eeQuatMJ.y() << "  " << eeQuatMJ.z() << " ]" << std::endl;
          std::cout << "[Pinocchio] Pos: [ " << eePosPin[0] << "  " << eePosPin[1] << "  " << eePosPin[2] << " ]" << std::endl;
          std::cout << "[Pinocchio] Quat: [ " << eeQuatPin.w() << "  " << eeQuatPin.x() << "  " << eeQuatPin.y() << "  " << eeQuatPin.z() << " ]" << std::endl;

          double mjPinPosError = (eePosMJ - eePosPin).norm();
          double mjPinQuatError = 2.0 * std::acos(std::min(1.0, std::abs(eeQuatMJ.dot(eeQuatPin))));
          std::cout << "Pin-MJ diff: " << std::scientific << std::setprecision(3) << mjPinPosError << " m, " << mjPinQuatError << " rad" << std::endl;
          std::cout << std::endl;
        }

        if (mjPinMatch) {
          testsPassed_++;
        } else {
          testsFailed_++;
        }
      }
    }

    if (enableRendering_ && renderingInitialized_ && (stepCount % 50 == 0)) {
      callRendering();
    }

    stepCount++;
    currentTime += simTimestep_;
  }

  printResults();
  return testsFailed_ == 0;
}

bool HybridKinematicsTest::getMujocoSitePose(const Eigen::VectorXd &qRef, int siteIndex, Eigen::Vector3d &pos, Eigen::Vector4d &quat) {
  if (!multiSiteMode_ || siteIndex >= static_cast<int>(siteIds_.size())) {
    std::cerr << "Invalid site index: " << siteIndex << std::endl;
    return false;
  }

  // Set joint positions and update MuJoCo
  for (int i = 0; i < numJoints_; ++i) {
    mjData_->ctrl[i] = qRef[i];
  }
  mj_step(mjModel_, mjData_);
  mj_forward(mjModel_, mjData_);

  // Get site pose
  int siteId = siteIds_[siteIndex];
  const double *sitePos = mjData_->site_xpos + 3 * siteId;
  const double *siteRotm = mjData_->site_xmat + 9 * siteId;
  double siteQuat[4];
  mju_mat2Quat(siteQuat, siteRotm);

  pos = Eigen::Vector3d(sitePos[0], sitePos[1], sitePos[2]);
  quat = Eigen::Vector4d(siteQuat[0], siteQuat[1], siteQuat[2], siteQuat[3]);

  return true;
}

bool HybridKinematicsTest::getPinSitePose(const Eigen::VectorXd &qRef, int siteIndex, Eigen::Vector3d &pos, Eigen::Vector4d &quat) {
  if (!multiSiteMode_ || siteIndex >= static_cast<int>(siteNames_.size())) {
    std::cerr << "Invalid site index: " << siteIndex << std::endl;
    return false;
  }

  try {
    // Get feedback from MuJoCo
    Eigen::VectorXd qFb(numJoints_);
    for (int i = 0; i < numJoints_; ++i) {
      qFb[i] = mjData_->qpos[i];
    }

    // Update Pinocchio
    pinKine_->update(qFb);

    // Get site pose
    pos = pinKine_->getSitePos(siteIndex);
    quat = pinKine_->getSiteQuat(siteIndex);

    return true;
  } catch (const std::exception &e) {
    std::cerr << "Pinocchio site pose computation error: " << e.what() << std::endl;
    return false;
  }
}

bool HybridKinematicsTest::runMultiSiteComparison(int scenario) {
  if (!multiSiteMode_) {
    std::cerr << "Multi-site comparison requires multi-site mode" << std::endl;
    return false;
  }

  double durationSeconds = (scenario == 0) ? 5 * simTimestep_ : 10.0;
  int maxSteps = (scenario == 0) ? 5 : INT_MAX;

  testsPassed_ = 0;
  testsFailed_ = 0;
  maxPosError_ = 0.0;
  maxQuatError_ = 0.0;
  targetJointAngles_.resize(0);

  int stepCount = 0;
  double currentTime = 0.0;

  std::cout << "\n=== Multi-Site Kinematics Comparison ===" << std::endl;
  std::cout << "Testing " << siteNames_.size() << " sites: ";
  for (size_t i = 0; i < siteNames_.size(); ++i) {
    std::cout << siteNames_[i];
    if (i < siteNames_.size() - 1) std::cout << ", ";
  }
  std::cout << "\n"
            << std::endl;

  while (currentTime < durationSeconds && stepCount < maxSteps) {
    Eigen::VectorXd qRef = genJointRef(currentTime, scenario, durationSeconds);

    bool shouldPrint = (stepCount == 0) || (scenario == 0) || (std::fmod(currentTime, 1.0) < simTimestep_) || (currentTime + simTimestep_ >= durationSeconds);

    if (shouldPrint) {
      Eigen::VectorXd qFb(mjModel_->nq);
      for (int j = 0; j < mjModel_->nq; ++j) {
        qFb[j] = mjData_->qpos[j];
      }
      std::cout << "--- t=" << std::fixed << std::setprecision(2) << currentTime << "s ---" << std::endl;
      std::cout << "Joint angles (rad): " << std::setprecision(6) << qFb.transpose() << std::endl;
    }

    // Compare each site
    for (size_t siteIdx = 0; siteIdx < siteNames_.size(); ++siteIdx) {
      Eigen::Vector3d sitePosPin, sitePosMJ;
      Eigen::Vector4d siteQuatPin, siteQuatMJ;

      bool mjSuccess = getMujocoSitePose(qRef, siteIdx, sitePosMJ, siteQuatMJ);
      bool pinSuccess = getPinSitePose(qRef, siteIdx, sitePosPin, siteQuatPin);

      if (!mjSuccess || !pinSuccess) {
        testsFailed_++;
        std::cerr << "Step " << stepCount << " failed for site " << siteNames_[siteIdx] << std::endl;
        continue;
      }

      bool match = comparePoses(sitePosMJ, siteQuatMJ, sitePosPin, siteQuatPin);

      if (shouldPrint) {
        std::cout << "\n[" << siteNames_[siteIdx] << "]" << std::endl;
        std::cout << "  [MuJoCo   ] Pos: [ " << std::setprecision(6) << sitePosMJ[0] << "  "
                  << sitePosMJ[1] << "  " << sitePosMJ[2] << " ]" << std::endl;
        std::cout << "  [MuJoCo   ] Quat: [ " << siteQuatMJ.w() << "  " << siteQuatMJ.x() << "  "
                  << siteQuatMJ.y() << "  " << siteQuatMJ.z() << " ]" << std::endl;
        std::cout << "  [Pinocchio] Pos: [ " << sitePosPin[0] << "  " << sitePosPin[1] << "  "
                  << sitePosPin[2] << " ]" << std::endl;
        std::cout << "  [Pinocchio] Quat: [ " << siteQuatPin.w() << "  " << siteQuatPin.x() << "  "
                  << siteQuatPin.y() << "  " << siteQuatPin.z() << " ]" << std::endl;

        double posError = (sitePosMJ - sitePosPin).norm();
        double quatError = 2.0 * std::acos(std::min(1.0, std::abs(siteQuatMJ.dot(siteQuatPin))));
        std::cout << "  Pin-MJ diff: " << std::scientific << std::setprecision(6)
                  << posError << " m, " << quatError << " rad ";
        if (match) {
          std::cout << "✓ PASS" << std::endl;
        } else {
          std::cout << "✗ FAIL" << std::endl;
        }
      }

      if (match) {
        testsPassed_++;
      } else {
        testsFailed_++;
      }
    }

    if (shouldPrint) {
      std::cout << std::endl;
    }

    if (enableRendering_ && renderingInitialized_ && (stepCount % 50 == 0)) {
      callRendering();
    }

    stepCount++;
    currentTime += simTimestep_;
  }

  std::cout << "=== Summary ===" << std::endl;
  std::cout << "Total site comparisons: " << (testsPassed_ + testsFailed_) << std::endl;
  std::cout << "Passed: " << testsPassed_ << std::endl;
  std::cout << "Failed: " << testsFailed_ << std::endl;
  std::cout << "Max position error: " << std::scientific << std::setprecision(6) << maxPosError_ << " m" << std::endl;
  std::cout << "Max quaternion error: " << maxQuatError_ << " rad" << std::endl;

  printResults();
  return testsFailed_ == 0;
}

void HybridKinematicsTest::printResults() {
  if (testsFailed_ == 0) {
    std::cout << "✓ All tests passed" << std::endl;
  } else {
    std::cout << "✗ " << testsFailed_ << " test(s) failed" << std::endl;
  }
}

bool HybridKinematicsTest::initRendering() {
  if (renderingInitialized_) {
    return true;
  }

  try {
    if (!glfwInit()) {
      std::cerr << "Failed to initialize GLFW" << std::endl;
      return false;
    }

    glfwWindow_ = glfwCreateWindow(1200, 900, "Hybrid Kinematics Test", nullptr, nullptr);
    if (!glfwWindow_) {
      std::cerr << "Failed to create GLFW window" << std::endl;
      glfwTerminate();
      return false;
    }

    glfwMakeContextCurrent(glfwWindow_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&mjCamera_);
    mjv_defaultOption(&mjvOption_);
    mjv_defaultScene(&mjScene_);
    mjr_defaultContext(&mjrContext_);

    mjv_makeScene(mjModel_, &mjScene_, 2000);
    mjr_makeContext(mjModel_, &mjrContext_, mjFONTSCALE_150);

    mjCamera_.azimuth = 90.0;
    mjCamera_.elevation = -10.0;
    mjCamera_.distance = 1.5;
    mjCamera_.lookat[0] = 0.0;
    mjCamera_.lookat[1] = 0.0;
    mjCamera_.lookat[2] = 0.3;

    renderingInitialized_ = true;
    callRendering();
    return true;

  } catch (const std::exception &e) {
    std::cerr << "Rendering initialization error: " << e.what() << std::endl;
    closeRendering();
    return false;
  }
}

void HybridKinematicsTest::callRendering() {
  if (!enableRendering_ || !renderingInitialized_ || !glfwWindow_) {
    return;
  }

  if (glfwWindowShouldClose(glfwWindow_)) {
    return;
  }

  try {
    glfwPollEvents();
    mjv_updateScene(mjModel_, mjData_, &mjvOption_, nullptr, &mjCamera_, mjCAT_ALL, &mjScene_);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(glfwWindow_, &viewport.width, &viewport.height);
    mjr_render(viewport, &mjScene_, &mjrContext_);
    glfwSwapBuffers(glfwWindow_);
  } catch (const std::exception &e) {
    std::cerr << "Rendering error: " << e.what() << std::endl;
  }
}

void HybridKinematicsTest::closeRendering() {
  if (!renderingInitialized_) {
    return;
  }

  try {
    mjv_freeScene(&mjScene_);
    mjr_freeContext(&mjrContext_);
    if (glfwWindow_) {
      glfwDestroyWindow(glfwWindow_);
      glfwWindow_ = nullptr;
    }
    glfwTerminate();
    renderingInitialized_ = false;
  } catch (const std::exception &e) {
    std::cerr << "Rendering cleanup error: " << e.what() << std::endl;
  }
}

} // namespace test
