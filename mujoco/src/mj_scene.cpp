#include "mj_scene.hpp"

#include <algorithm>
#include <random>

#include "orient_tools.hpp"

namespace mujoco {

inline void eigenToMjt(const Eigen::Vector3d &v, mjtNum *arr) {
  arr[0] = v[0];
  arr[1] = v[1];
  arr[2] = v[2];
}

inline void eigenToMjt(const Eigen::Vector3d &v, float *arr) {
  arr[0] = static_cast<float>(v[0]);
  arr[1] = static_cast<float>(v[1]);
  arr[2] = static_cast<float>(v[2]);
}

inline void eigenToMjt(const Eigen::Matrix3d &m, mjtNum *arr) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      arr[i * 3 + j] = m(i, j);
    }
  }
}

inline void eigenToMjt(const Eigen::Matrix3d &m, float *arr) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      arr[i * 3 + j] = static_cast<float>(m(i, j));
    }
  }
}

MujocoScene::MujocoScene(MujocoInterface &mj) :
    mj_(mj),
    robotType_(mj.robotManager->getRobotType()),
    sceneNumber_(mj.sceneManager->getSceneNumber()),
    sceneType_(mj.sceneManager->getSceneType()) {
}

void MujocoScene::initScene() {
  numEE_ = mj_.robotManager->getNumEndEffectors();
  isDualArm_ = (numEE_ >= 2);

  if (isDualArm_) {
    robotBaseOffset_ = Eigen::Vector3d(-0.25, 0.0, 0.75);
  }

  initObjects();
  initCameras();
}

void MujocoScene::initObjects() {
  std::vector<std::string> objectNames;

  for (int i = 0; i < mj_.mjModel_->nbody; i++) {
    const char *bodyName = mj_id2name(mj_.mjModel_, mjOBJ_BODY, i);
    if (!bodyName) continue;

    std::string name(bodyName);

    if (name == "world" || name == "floor" || name.find("link") != std::string::npos || name.find("base") != std::string::npos) {
      continue;
    }

    objectNames.push_back(name);
  }

  if (!objectNames.empty()) {
    for (const auto &name : objectNames) {
      mj_.runtimeData_.addObject(name);
    }
  }

  if (sceneType_ == rynn::SceneType::kUI || sceneType_ == rynn::SceneType::kTracking) {
    mj_.runtimeData_.addObject("cube");
    trackGeomCount_ = 4;
  }
}

void MujocoScene::renderTrackingVisuals() {
  if (trackGeomCount_ == 0) return;

  auto *cubeObj = mj_.runtimeData_.getObjectByName("cube");
  if (!cubeObj) return;

  if (mj_.mjScene_.ngeom + 4 > mj_.mjScene_.maxgeom) return;

  trackGeomIdx_ = mj_.mjScene_.ngeom;

  Eigen::Vector3d cubeWorldPos = cubeObj->pos;
  if (isDualArm_ && (sceneType_ == rynn::SceneType::kUI || sceneType_ == rynn::SceneType::kTracking)) {
    cubeWorldPos = cubeObj->pos + robotBaseOffset_;
  }

  const Eigen::Matrix3d R = cubeObj->quat.toRotationMatrix();
  mjtNum mat[9], pos[3];
  eigenToMjt(R, mat);
  eigenToMjt(cubeWorldPos, pos);

  mjvGeom *cube = &mj_.mjScene_.geoms[mj_.mjScene_.ngeom++];
  mjtNum sz[3] = {0.01, 0.01, 0.01};
  float rgba[4] = {0.9f, 0.9f, 0.1f, 0.9f};
  mjv_initGeom(cube, mjGEOM_BOX, sz, pos, mat, rgba);
  cube->objtype = mjOBJ_UNKNOWN;
  cube->objid = -1;
  cube->category = mjCAT_DECOR;

  const float colors[3][4] = {
      {1.0f, 0.0f, 0.0f, 1.0f},
      {0.0f, 1.0f, 0.0f, 1.0f},
      {0.0f, 0.0f, 1.0f, 1.0f}};

  const Eigen::Matrix3d axisRots[3] = {
      utils::rotmy(M_PI / 2),
      utils::rotmx(-M_PI / 2),
      Eigen::Matrix3d::Identity()};

  for (int i = 0; i < 3; i++) {
    mjvGeom *cyl = &mj_.mjScene_.geoms[mj_.mjScene_.ngeom++];

    mjtNum cylSz[3] = {0.002, 0.03, 0.0};

    Eigen::Vector3d cylPos = cubeObj->pos + R.col(i) * 0.03;
    if (isDualArm_ && (sceneType_ == rynn::SceneType::kUI || sceneType_ == rynn::SceneType::kTracking)) {
      cylPos = cylPos + robotBaseOffset_;
    }
    const Eigen::Matrix3d cylR = R * axisRots[i];

    mjtNum cylPosMjt[3], cylMat[9];
    eigenToMjt(cylPos, cylPosMjt);
    eigenToMjt(cylR, cylMat);

    mjv_initGeom(cyl, mjGEOM_CYLINDER, cylSz, cylPosMjt, cylMat, colors[i]);
    cyl->objtype = mjOBJ_UNKNOWN;
    cyl->objid = -1;
    cyl->category = mjCAT_DECOR;
  }
}

void MujocoScene::initCameras() {
  mjv_defaultCamera(&mj_.mjCamera_);
  mj_.mjCamera_.lookat[0] = mj_.mjModel_->stat.center[0];
  mj_.mjCamera_.lookat[1] = mj_.mjModel_->stat.center[1];
  mj_.mjCamera_.lookat[2] = mj_.mjModel_->stat.center[2];
  mj_.mjCamera_.azimuth = mj_.mjModel_->vis.global.azimuth;
  mj_.mjCamera_.elevation = mj_.mjModel_->vis.global.elevation;
  mj_.mjCamera_.distance = 2.0 * mj_.mjModel_->stat.extent;

  std::vector<std::string> cameraNames;

  for (int i = 0; i < mj_.mjModel_->ncam; i++) {
    const char *camName = mj_id2name(mj_.mjModel_, mjOBJ_CAMERA, i);
    if (camName) {
      cameraNames.push_back(std::string(camName));
    }
  }

  if (!cameraNames.empty()) {
    for (const auto &name : cameraNames) {
      mj_.runtimeData_.addCamera(name);
    }
  }
}

void MujocoScene::update() {
  updateObjects();
  updateCameras();
}

void MujocoScene::updateObjects() {
  if (mj_.runtimeData_.objectStates.size() == 0) {
    return;
  }

  for (size_t objIdx = 0; objIdx < mj_.runtimeData_.getObjectCount(); ++objIdx) {
    auto &obj = mj_.runtimeData_.objectStates[objIdx];

    if (obj.name == "cube" && trackGeomIdx_ >= 0) {
      Eigen::Vector3d cubeWorldPos = obj.pos;
      if (isDualArm_ && (sceneType_ == rynn::SceneType::kUI || sceneType_ == rynn::SceneType::kTracking)) {
        cubeWorldPos = obj.pos + robotBaseOffset_;
      }

      const Eigen::Matrix3d R = obj.quat.toRotationMatrix();
      mjtNum mat[9];
      eigenToMjt(R, mat);

      mjvGeom *cube = &mj_.mjScene_.geoms[trackGeomIdx_];
      eigenToMjt(cubeWorldPos, cube->pos);
      std::copy(mat, mat + 9, cube->mat);

      const Eigen::Matrix3d axisRots[3] = {
          utils::rotmy(M_PI / 2), utils::rotmx(-M_PI / 2), Eigen::Matrix3d::Identity()};

      for (int i = 0; i < 3; i++) {
        mjvGeom *cyl = &mj_.mjScene_.geoms[trackGeomIdx_ + 1 + i];

        Eigen::Vector3d cylPos = obj.pos + R.col(i) * 0.03;
        if (isDualArm_ && (sceneType_ == rynn::SceneType::kUI || sceneType_ == rynn::SceneType::kTracking)) {
          cylPos = cylPos + robotBaseOffset_;
        }
        const Eigen::Matrix3d cylR = R * axisRots[i];

        eigenToMjt(cylPos, cyl->pos);
        eigenToMjt(cylR, cyl->mat);
      }
      continue;
    }

    int body_id = mj_name2id(mj_.mjModel_, mjOBJ_BODY, obj.name.c_str());
    if (body_id == -1) {
      continue;
    }

    obj.pos = Eigen::Vector3d(
        mj_.mjData_->xpos[3 * body_id],
        mj_.mjData_->xpos[3 * body_id + 1],
        mj_.mjData_->xpos[3 * body_id + 2]);

    // Convert pickplace objects from world frame to robot base frame
    if (isDualArm_ && sceneType_ == rynn::SceneType::kPickPlace) {
      obj.pos -= robotBaseOffset_;
    }

    obj.quat = Eigen::Quaterniond(
        mj_.mjData_->xquat[4 * body_id],     // w
        mj_.mjData_->xquat[4 * body_id + 1], // x
        mj_.mjData_->xquat[4 * body_id + 2], // y
        mj_.mjData_->xquat[4 * body_id + 3]  // z
    );

    int jnt_adr = mj_.mjModel_->body_jntadr[body_id];
    if (jnt_adr != -1 && mj_.mjModel_->jnt_type[jnt_adr] == mjJNT_FREE) {
      int qvel_adr = mj_.mjModel_->jnt_dofadr[jnt_adr];
      obj.velocity << mj_.mjData_->qvel[qvel_adr],
          mj_.mjData_->qvel[qvel_adr + 1],
          mj_.mjData_->qvel[qvel_adr + 2],
          mj_.mjData_->qvel[qvel_adr + 3],
          mj_.mjData_->qvel[qvel_adr + 4],
          mj_.mjData_->qvel[qvel_adr + 5];
    } else {
      obj.velocity.setZero();
    }
  }
}

void MujocoScene::updateCameras() {
  if (mj_.runtimeData_.cameras.size() == 0) {
    return;
  }

  size_t cameraIndex = 0;
  for (int i = 0; i < mj_.mjModel_->ncam; i++) {
    const char *camName = mj_id2name(mj_.mjModel_, mjOBJ_CAMERA, i);
    if (!camName) continue;

    if (cameraIndex < mj_.runtimeData_.getCameraCount()) {
      data::Camera &cam = mj_.runtimeData_.cameras[cameraIndex];

      cam.pos = Eigen::Vector3d(
          mj_.mjData_->cam_xpos[3 * i],
          mj_.mjData_->cam_xpos[3 * i + 1],
          mj_.mjData_->cam_xpos[3 * i + 2]);

      mjtNum quat[4];
      mju_mat2Quat(quat, mj_.mjData_->cam_xmat + 9 * i);
      cam.quat = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);

      cameraIndex++;
    }
  }
}

} // namespace mujoco
