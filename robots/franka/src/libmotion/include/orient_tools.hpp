/*! @file orientation_tools.h
 *  @brief Utility functions for 3D rotations
 *
 *  This file contains rotation utilities.  We generally use "coordinate
 * transformations" as opposed to the displacement transformations that are
 * commonly found in graphics.  To describe the orientation of a body, we use a
 * rotation matrix which transforms from world to body coordinates. This is the
 * transpose of the matrix which would rotate the body itself into the correct
 * orientation.
 *
 *  This follows the convention of Roy Featherstone's excellent book, Rigid Body
 * Dynamics Algorithms and the spatial_v2 MATLAB library that comes with it.
 * Note that we don't use the spatial_v2 convention for quaternions!
 */

#pragma once

#include <cmath>
#include <iostream>
#include <type_traits>

#include <Eigen/Dense>
#include "math_tools.hpp"

namespace utils {

static constexpr double quaternionDerivativeStabilization = 0.1;

enum class CoordinateAxis { X,
                            Y,
                            Z };

/*!
 * Convert radians to degrees
 */
template <typename T>
T rad2deg(T rad) {
  static_assert(std::is_floating_point<T>::value,
                "must use floating point value");
  return rad * T(180) / T(M_PI);
}

/*!
 * Convert degrees to radians
 */
template <typename T>
T deg2rad(T deg) {
  static_assert(std::is_floating_point<T>::value,
                "must use floating point value");
  return deg * T(M_PI) / T(180);
}

/*!
 * Compute rotation matrix for coordinate transformation. Note that
 * coordinateRotation(CoordinateAxis:X, .1) * v will rotate v by -.1 radians -
 * this transforms into a frame rotated by .1 radians!.
 */
template <typename T>
Eigen::Matrix<T, 3, 3> coordinateRotation(CoordinateAxis axis, T theta) {
  static_assert(std::is_floating_point<T>::value,
                "must use floating point value");
  T s = std::sin(theta);
  T c = std::cos(theta);

  Eigen::Matrix<T, 3, 3> R;

  if (axis == CoordinateAxis::X) {
    R << 1, 0, 0, 0, c, s, 0, -s, c;
  } else if (axis == CoordinateAxis::Y) {
    R << c, 0, -s, 0, 1, 0, s, 0, c;
  } else if (axis == CoordinateAxis::Z) {
    R << c, s, 0, -s, c, 0, 0, 0, 1;
  }

  return R;
}

template <typename T>
Eigen::Matrix<typename T::Scalar, 3, 3> crossMatrix(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "must have 3x1 vector");
  Eigen::Matrix<typename T::Scalar, 3, 3> m;
  m << 0, -v(2), v(1),
      v(2), 0, -v(0),
      -v(1), v(0), 0;
  return m;
}

/*!
 * Go from rpy to rotation matrix.
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 3, 3> rpyToRotMat(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "must have 3x1 vector");
  Eigen::Matrix<typename T::Scalar, 3, 3> m = coordinateRotation(CoordinateAxis::X, v[0]) * coordinateRotation(CoordinateAxis::Y, v[1]) * coordinateRotation(CoordinateAxis::Z, v[2]);
  return m;
}

inline Eigen::Matrix3d rpyToRotm(double roll, double pitch, double yaw) {
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return R;
}

/*!
 * Convert a 3x1 vector to a skew-symmetric 3x3 matrix
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 3, 3> vectorToSkewMat(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  Eigen::Matrix<typename T::Scalar, 3, 3> m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

/*!
 * Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 3, 1> matToSkewVec(const Eigen::MatrixBase<T> &m) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                "Must have 3x3 matrix");
  return 0.5 * Eigen::Matrix<typename T::Scalar, 3, 1>(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), (m(1, 0) - m(0, 1)));
}

/*!
 * Convert a coordinate transformation matrix to an orientation quaternion.
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 4, 1> rotm2quat(
    const Eigen::MatrixBase<T> &r1) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                "Must have 3x3 matrix");
  Eigen::Matrix<typename T::Scalar, 4, 1> q;
  Eigen::Matrix<typename T::Scalar, 3, 3> r = r1.transpose();
  typename T::Scalar tr = r.trace();
  if (tr > 0.0) {
    typename T::Scalar S = sqrt(tr + 1.0) * 2.0;
    q(0) = 0.25 * S;
    q(1) = (r(2, 1) - r(1, 2)) / S;
    q(2) = (r(0, 2) - r(2, 0)) / S;
    q(3) = (r(1, 0) - r(0, 1)) / S;
  } else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
    typename T::Scalar S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    q(0) = (r(2, 1) - r(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (r(0, 1) + r(1, 0)) / S;
    q(3) = (r(0, 2) + r(2, 0)) / S;
  } else if (r(1, 1) > r(2, 2)) {
    typename T::Scalar S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    q(0) = (r(0, 2) - r(2, 0)) / S;
    q(1) = (r(0, 1) + r(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (r(1, 2) + r(2, 1)) / S;
  } else {
    typename T::Scalar S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    q(0) = (r(1, 0) - r(0, 1)) / S;
    q(1) = (r(0, 2) + r(2, 0)) / S;
    q(2) = (r(1, 2) + r(2, 1)) / S;
    q(3) = 0.25 * S;
  }
  return q;
}

/*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 3, 3> quaternionToRotationMatrix(
    const Eigen::MatrixBase<T> &q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  typename T::Scalar e0 = q(0);
  typename T::Scalar e1 = q(1);
  typename T::Scalar e2 = q(2);
  typename T::Scalar e3 = q(3);

  Eigen::Matrix<typename T::Scalar, 3, 3> R;
  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
      2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
      1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
      2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
      1 - 2 * (e1 * e1 + e2 * e2);
  R.transposeInPlace();
  return R;
}

/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 3, 1> quatToRPY(const Eigen::MatrixBase<T> &q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  Eigen::Matrix<typename T::Scalar, 3, 1> rpy;
  typename T::Scalar as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
  rpy(2) =
      std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                 (q[0] * q[0]) + (q[1] * q[1]) - (q[2] * q[2]) - (q[3] * q[3]));
  rpy(1) = std::asin(as);
  rpy(0) =
      std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                 (q[0] * q[0]) - (q[1] * q[1]) - (q[2] * q[2]) + (q[3] * q[3]));
  return rpy;
}

template <typename T>
Eigen::Matrix<typename T::Scalar, 4, 1> rpyToQuat(const Eigen::MatrixBase<T> &rpy) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 vec");
  Eigen::Matrix<typename T::Scalar, 3, 3> R = rpyToRotMat(rpy);
  Eigen::Matrix<typename T::Scalar, 4, 1> q = rotm2quat(R);
  return q;
}

/*!
 * Convert a quaternion to so3.
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 3, 1> quatToso3(const Eigen::MatrixBase<T> &q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  Eigen::Matrix<typename T::Scalar, 3, 1> so3;
  typename T::Scalar theta = 2. * std::acos(q[0]);
  so3[0] = theta * q[1] / std::sin(theta / 2.);
  so3[1] = theta * q[2] / std::sin(theta / 2.);
  so3[2] = theta * q[3] / std::sin(theta / 2.);
  return so3;
}
/*!
 * Quaternion derivative calculation, like rqd(q, omega) in MATLAB.
 * the omega is expressed in body frame
 * @param q
 * @param omega
 * @return
 */
template <typename T, typename T2>
Eigen::Matrix<typename T::Scalar, 4, 1> quatDerivative(const Eigen::MatrixBase<T> &q,
                                        const Eigen::MatrixBase<T2> &omega) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 omega");
  // first case in rqd
  Eigen::Matrix<typename T::Scalar, 4, 4> Q;
  Q << q[0], -q[1], -q[2], -q[3], q[1], q[0], -q[3], q[2], q[2], q[3], q[0],
      -q[1], q[3], -q[2], q[1], q[0];

  Eigen::Matrix<typename T::Scalar, 4, 1> qq(
      quaternionDerivativeStabilization * omega.norm() * (1 - q.norm()),
      omega[0], omega[1], omega[2]);
  Eigen::Matrix<typename T::Scalar, 4, 1> dq = 0.5 * Q * qq;
  return dq;
}

/*!
 * Take the product of two quaternions
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 4, 1> quatProduct(const Eigen::MatrixBase<T> &q1,
                                     const Eigen::MatrixBase<T> &q2) {
  typename T::Scalar r1 = q1[0];
  typename T::Scalar r2 = q2[0];
  Eigen::Matrix<typename T::Scalar, 3, 1> v1(q1[1], q1[2], q1[3]);
  Eigen::Matrix<typename T::Scalar, 3, 1> v2(q2[1], q2[2], q2[3]);

  typename T::Scalar r = r1 * r2 - v1.dot(v2);
  Eigen::Matrix<typename T::Scalar, 3, 1> v = r1 * v2 + r2 * v1 + v1.cross(v2);
  Eigen::Matrix<typename T::Scalar, 4, 1> q(r, v[0], v[1], v[2]);
  return q;
}

/*!
 * Compute new quaternion given:
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
template <typename T, typename T2, typename T3>
Eigen::Matrix<typename T::Scalar, 4, 1> integrateQuat(const Eigen::MatrixBase<T> &quat,
                                       const Eigen::MatrixBase<T2> &omega,
                                       T3 dt) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 omega");
  Eigen::Matrix<typename T::Scalar, 3, 1> axis;
  typename T::Scalar ang = omega.norm();
  if (ang > 0) {
    axis = omega / ang;
  } else {
    axis = Eigen::Matrix<typename T::Scalar, 3, 1>(1, 0, 0);
  }

  ang *= dt;
  Eigen::Matrix<typename T::Scalar, 3, 1> ee = std::sin(ang / 2) * axis;
  Eigen::Matrix<typename T::Scalar, 4, 1> quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

  Eigen::Matrix<typename T::Scalar, 4, 1> quatNew = quatProduct(quatD, quat);
  quatNew = quatNew / quatNew.norm();
  return quatNew;
}

/*!
 * Compute new quaternion given:
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
template <typename T, typename T2, typename T3>
Eigen::Matrix<typename T::Scalar, 4, 1> integrateQuatImplicit(
    const Eigen::MatrixBase<T> &quat, const Eigen::MatrixBase<T2> &omega,
    T3 dt) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 omega");
  Eigen::Matrix<typename T::Scalar, 3, 1> axis;
  typename T::Scalar ang = omega.norm();
  if (ang > 0) {
    axis = omega / ang;
  } else {
    axis = Eigen::Matrix<typename T::Scalar, 3, 1>(1, 0, 0);
  }

  ang *= dt;
  Eigen::Matrix<typename T::Scalar, 3, 1> ee = std::sin(ang / 2) * axis;
  Eigen::Matrix<typename T::Scalar, 4, 1> quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

  Eigen::Matrix<typename T::Scalar, 4, 1> quatNew = quatProduct(quat, quatD);
  quatNew = quatNew / quatNew.norm();
  return quatNew;
}

template <typename T>
void quaternionToso3(const Eigen::Matrix<T, 4, 1> quat, Eigen::Matrix<T, 3, 1> &so3) {
  so3[0] = quat[1];
  so3[1] = quat[2];
  so3[2] = quat[3];

  T theta =
      2.0 * asin(sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]));

  if (fabs(theta) < 0.0000001) {
    so3.setZero();
    return;
  }
  so3 /= sin(theta / 2.0);
  so3 *= theta;
}
template <typename T>
Eigen::Matrix<T, 4, 1> so3ToQuat(Eigen::Matrix<T, 3, 1> &so3) {
  Eigen::Matrix<T, 4, 1> quat;

  T theta = sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]);

  if (fabs(theta) < 1.e-6) {
    quat.setZero();
    quat[0] = 1.;
    return quat;
  }
  quat[0] = cos(theta / 2.);
  quat[1] = so3[0] / theta * sin(theta / 2.);
  quat[2] = so3[1] / theta * sin(theta / 2.);
  quat[3] = so3[2] / theta * sin(theta / 2.);
  return quat;
}

inline Eigen::Vector3d R2rpy(const Eigen::Matrix3d &R) {
  Eigen::Vector3d rpy;
  rpy[0] = atan2(R(2, 1), R(2, 2));
  rpy[1] = asin(-R(2, 0));
  rpy[2] = atan2(R(1, 0), R(0, 0));
  return rpy;
}

inline Eigen::Vector3d R2zyz(const Eigen::Matrix3d &R) {
  Eigen::Vector3d zyz;
  zyz[1] = acos(R(2, 2));

  if (fabs(R(2, 2)) < 1.0) {
    zyz[0] = atan2(R(1, 2), R(0, 2));
    zyz[2] = atan2(R(2, 1), -R(2, 0));
  } else {
    zyz[0] = 0;
    zyz[2] = atan2(R(1, 0), R(0, 0));
  }

  return zyz;
}

inline Eigen::Vector3d quat2RPY(const Eigen::Vector4d &q) {
  Eigen::Vector3d rpy;

  // q is in [x,y,z,w] order: q(0)=x, q(1)=y, q(2)=z, q(3)=w
  const double sinr_cosp = 2 * (q(3) * q(0) + q(1) * q(2));
  const double cosr_cosp = 1 - 2 * (square(q(0)) + square(q(1)));
  rpy(0) = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2 * (q(3) * q(1) - q(2) * q(0));
  if (std::abs(sinp) >= 1)
    rpy(1) = std::copysign(M_PI_2, sinp);
  else
    rpy(1) = std::asin(sinp);

  const double siny_cosp = 2 * (q(3) * q(2) + q(0) * q(1));
  const double cosy_cosp = 1 - 2 * (square(q(1)) + square(q(2)));
  rpy(2) = std::atan2(siny_cosp, cosy_cosp);

  return rpy;
}

/**
 * @brief convert Z-Y-X euler angles to quaternion
 *
 * @param rpy Z-Y-X euler angles, in the order of roll/X, pitch/Y, yaw/Z
 * @return Eigen::Quaterniond
 */
inline Eigen::Quaterniond EulerZYX2Quaternion(const Eigen::Vector3d &rpy) {
  Eigen::AngleAxisd r_angle(rpy(0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd p_angle(rpy(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd y_angle(rpy(2), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat = y_angle * p_angle * r_angle;
  return quat;
}

/**
 * @brief
 *
 * @param pos, 3*1 position vector in Eigen::Vector3d
 * @param quat, 4*1 quaternion in Eigen::Quaterniond
 * @return Eigen::Matrix4d
 */
inline Eigen::Matrix4d posQuatToHomo(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

/**
 * @brief
 *
 * @param T, 4*4 homogeneous transformation matrix
 * @return std::pair<Eigen::Vector3d, Eigen::Quaterniond>
 *
 * Eigen::Matrix4d transformMatrix = ...;
 * auto [position, orientation] = homogeneousToPosQuat(transformMatrix);
 */
inline std::pair<Eigen::Vector3d, Eigen::Quaterniond> homoToPosQuat(const Eigen::Matrix4d &T) {
  Eigen::Matrix3d rotm = T.block<3, 3>(0, 0);
  Eigen::Vector3d pos = T.block<3, 1>(0, 3);
  Eigen::Quaterniond quat(rotm);
  return std::make_pair(pos, quat);
}

inline Eigen::Matrix3d rotmx(double theta) {
  Eigen::Matrix3d R;
  R << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
  return R;
}

inline Eigen::Matrix3d rotmy(double theta) {
  Eigen::Matrix3d R;
  R << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
  return R;
}

inline Eigen::Matrix3d rotmz(double theta) {
  Eigen::Matrix3d R;
  R << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
  return R;
}

template <typename T>
Eigen::Matrix4d spatialVelocityToTransform(const Eigen::MatrixBase<T> &spatialVel, double dt) {
  assert(spatialVel.size() == 6 && "Input must be a 6x1 vector.");

  Eigen::Vector3d linearVel = spatialVel.template head<3>();
  Eigen::Vector3d angularVel = spatialVel.template tail<3>();

  Eigen::Matrix3d skewAngularVel = vectorToSkewMat(angularVel);

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() + dt * skewAngularVel;
  transform.block<3, 1>(0, 3) = dt * linearVel;

  return transform;
}

} // namespace utils
