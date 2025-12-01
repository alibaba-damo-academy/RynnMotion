#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>

#include "fifthCurve.hpp"
#include "math_tools.hpp"

namespace sketch {

using Point3D = Eigen::Vector3d;
using Path = std::vector<Point3D>;

struct OneTrajectoryFifthSpline {
  std::vector<double> t;
  std::vector<Point3D> pos;
  std::vector<Point3D> vel;

  std::vector<Eigen::VectorXd> x_coeffs;
  std::vector<Eigen::VectorXd> y_coeffs;
  std::vector<Eigen::VectorXd> z_coeffs;

  std::vector<Eigen::VectorXd> vx_coeffs;
  std::vector<Eigen::VectorXd> vy_coeffs;
  std::vector<Eigen::VectorXd> vz_coeffs;

  std::vector<Eigen::VectorXd> ax_coeffs;
  std::vector<Eigen::VectorXd> ay_coeffs;
  std::vector<Eigen::VectorXd> az_coeffs;

  std::vector<double> segment_times;
  double time_total;
};

struct TrajectoryFifthSpline {
  std::vector<OneTrajectoryFifthSpline> trajs;
  Point3D last_point;
  double time_total_all_trajs;
};

class FifthSpline {
public:
  FifthSpline();
  virtual ~FifthSpline() = default;

  FifthSpline(const FifthSpline &) = delete;
  FifthSpline &operator=(const FifthSpline &) = delete;

  /**
   * @brief Generates a spline through given waypoints using fifth-order polynomials
   * @param waypoints Series of points to traverse
   * @param max_vel Maximum allowed velocity
   * @param trajectory Reference to trajectory to be filled
   * @return True if generation was successful
   */
  bool genSpline(const Path &waypoints, double max_vel, OneTrajectoryFifthSpline &trajectory);

  /**
   * @brief Evaluates a spline at a specific time
   * @param trajectory The trajectory to evaluate
   * @param t Time at which to evaluate
   * @param position Output position at time t
   * @param velocity Optional output velocity at time t
   * @return True if evaluation was successful, false otherwise
   */
  bool evalSplineAtTime(const OneTrajectoryFifthSpline &trajectory, double t,
                        Point3D &position, Point3D *velocity = nullptr,
                        Point3D *accelaration = nullptr);

private:
  std::vector<double> computeSegmentTimeAndVel(const Path &waypoints, std::vector<Point3D> &vel_list,
                                               double max_vel, double max_acc);

  std::vector<Point3D> genSplineAccProfile(const Path &waypoints,
                                           const std::vector<Point3D> &vel_list,
                                           const std::vector<double> &t_list,
                                           double max_acc);

  double computeSegmentTimeWithinMaxAcc(double distance, double v0,
                                        double v1, double max_vel,
                                        double max_acc, double &updated_v_norm,
                                        int &update_vel_type);

  double computeTimeWithSufficientDis(double distance, double v0, double v1,
                                      double max_vel, double max_acc);

  void computeAccAndDecProcess(double distance, double v0, double v1,
                               double max_acc, double &updated_v_norm,
                               double &t_seg, int &update_vel_type);

  double computeOneSegmentVelocityAndTime(const Path &waypoints, size_t i,
                                          std::vector<Point3D> &vel_list,
                                          const double max_vel, const double max_acc,
                                          double &updated_v_norm, int &update_vel_type,
                                          bool xyz_dir_flag);

  void reduceNextVel(Point3D &v1, const Point3D &updated_vel_xyz,
                     const Eigen::Vector3i &update_vel_type_xyz,
                     bool &update_vel_flag_xyz);

  void reduceCurrentVel(const Point3D &v0, const Point3D &updated_vel_xyz,
                        const Eigen::Vector3i &update_vel_type_xyz,
                        double v0_all, double &updated_v_norm,
                        int &update_vel_type);

  bool reduceBothVels(std::vector<Point3D> &vel_list, size_t i,
                      double txyz_max, const Point3D &time_vec,
                      const Point3D &vec1, double max_acc, double v0_all,
                      double &updated_v_norm, int &update_vel_type);

  double computeFastAxisVels(double v0, double v1, double distance,
                             double t_xyz, double txyz_max,
                             double max_acc, double v0_all,
                             double &updated_v_norm, int &update_vel_type);

  double computeMaxVelByRadius(const Eigen::Vector3d &triangle_side_length);

  double computeSegmentLengthRatio(const Eigen::Vector3d &triangle_side_length);

  void updateXYZVelByTangentDir(Point3D &v1, double max_vel,
                                const Eigen::Vector3d &dir_vec,
                                const Eigen::Vector3d &triangle_side_length);

  double velAllByXYZ(double &v1_xyz, double dir_xyz, double max_vel);

  std::vector<Point3D> genSplineVelProfile(const Path &waypoints, double max_vel, double acc_max);

  void addCoefficients(double p0, double p1, double v0, double v1, double a0, double a1, double t,
                       std::vector<Eigen::VectorXd> &pos_coeffs, std::vector<Eigen::VectorXd> &vel_coeffs,
                       std::vector<Eigen::VectorXd> &acc_coeffs);

  // Fifth order polynomial curve for trajectory generation
  std::shared_ptr<utils::FifthCurve> curvePtr_;
};

} // namespace sketch