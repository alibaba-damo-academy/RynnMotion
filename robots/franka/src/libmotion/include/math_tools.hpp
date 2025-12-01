#pragma once

#include <Eigen/Dense>

namespace utils {

template <typename T>
T square(T a) {
  return a * a;
}

template <typename T>
void limitBufferSize(std::vector<T> &buffer, size_t maxSize) {
  if (buffer.size() > maxSize) {
    buffer.erase(buffer.begin(), buffer.begin() + (buffer.size() - maxSize));
  }
}

template <typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T> &a, const Eigen::MatrixBase<T> &b, T2 tol) {
  long x = T::RowsAtCompileTime;
  long y = T::ColsAtCompileTime;

  if (T::RowsAtCompileTime == Eigen::Dynamic || T::ColsAtCompileTime == Eigen::Dynamic) {
    assert(a.rows() == b.rows());
    assert(a.cols() == b.cols());
    x = a.rows();
    y = a.cols();
  }

  for (long i = 0; i < x; i++) {
    for (long j = 0; j < y; j++) {
      T2 error = std::abs(a(i, j) - b(i, j));
      if (error >= tol)
        return false;
    }
  }
  return true;
}

template <typename T>
Eigen::Matrix<T, 3, 3> VectorCrossMatrix(Eigen::Matrix<T, 3, 1> v) {
  Eigen::Matrix<T, 3, 3> rs;
  rs << 0, -v(2), v(1),
      v(2), 0, -v(0),
      -v(1), v(0), 0;
  return rs;
}

template <typename T>
inline int sign(const T &a) {
  return a < 0 ? -1 : 1;
}

template <typename T>
inline double sech(T x) {
  return 2 / (exp(x) + exp(-x));
}

template <typename T>
inline double gause(T x) {
  return exp(-x * x);
}

template <typename T1, typename T2, typename T3>
bool clip(T1 &x, const T2 &a, const T3 &b) {
  if (a <= b) {
    if (x <= a) {
      x = (T1)a;
      return 1;
    } else if (x > b) {
      x = (T1)b;
      return 1;
    }
  } else {
    if (x <= b) {
      x = (T1)b;
      return 1;
    } else if (x > a) {
      x = (T1)a;
      return 1;
    }
  }
  return 0;
}

template <typename T1, typename T2>
bool clip(T1 &x, T2 lim) {
  if (lim < 0) {
    lim = -lim;
  }
  if (x < -lim) {
    x = (T1)(-lim);
    return 1;
  } else if (x > lim) {
    x = (T1)(lim);
    return 1;
  }
  return 0;
}

Eigen::MatrixXd computeNullspace(const Eigen::MatrixXd &jacobian);
double getSignedAngle(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
double getSignedAngle2D(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2);
} // namespace utils
