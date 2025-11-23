#include "math_tools.hpp"

namespace utils {

Eigen::MatrixXd computeNullspace(const Eigen::MatrixXd &jacobian) {
  int n = jacobian.cols();
  Eigen::MatrixXd jacobian_pseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
  Eigen::MatrixXd nullspace = I - jacobian_pseudoInverse * jacobian;
  return nullspace;
}

double getSignedAngle(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
  double dot_product = v1.dot(v2);

  double magnitude_v1 = v1.norm();
  double magnitude_v2 = v2.norm();

  double cos_theta = dot_product / (magnitude_v1 * magnitude_v2);

  cos_theta = std::fmax(-1.0, std::fmin(1.0, cos_theta));

  double theta = std::acos(cos_theta);

  Eigen::Vector3d cross_product = v1.cross(v2);

  if (cross_product.z() < 0) {
    return -theta;
  } else {
    return theta;
  }
}

double getSignedAngle2D(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2) {
  double q1 = atan2(v1.y(), v1.x());
  double q2 = atan2(v2.y(), v2.x());
  return q2 - q1;
}

} // namespace utils
