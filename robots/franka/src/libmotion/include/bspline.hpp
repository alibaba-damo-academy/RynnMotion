#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

namespace sketch {

using Point3D = Eigen::Vector3d;
using Path = std::vector<Point3D>;
using PathList = std::vector<Path>;

struct TrajectoryBSpline {
  std::vector<std::vector<double>> t_seg_list;
  PathList path_list;
  PathList vel_points_list;
  PathList acc_points_list;

  PathList key_paths;
  PathList key_vel_lists;
  PathList key_acc_lists;

  std::vector<std::vector<float>> node_vectors;
  std::vector<int> b_spline_orders;
  std::vector<double> segment_times;

  double time_total_all_trajs;
  Point3D last_point;
};

struct BSplineKeyPointList {
  Path key_path_point_list;
  Path key_vel_point_list;
  Path key_acc_point_list;
};

class BSpline {
public:
  BSpline() = default;

  ~BSpline() = default;

  void genSpline(const Path &origin_control_points, TrajectoryBSpline &traj, double max_vel = 0.1,
                 int spline_order = 3, int num_path_points_required = -1, bool rearrange_key_points_flag = true) const;

  bool evalSplineAtTime(const TrajectoryBSpline &trajectory, double t,
                        Point3D &position, Point3D *velocity, Point3D *acceleration) const;

private:
  float bSplineBasis(const std::vector<float> &node_vector, int i, int spline_order, float u) const;

  double computeLineDistance(const Path &path) const;

  std::vector<float> quasiUniformMode(int index_end, int k) const;

  void computePathPointVal(const std::vector<float> &u_list,
                           const BSplineKeyPointList &key_point_list,
                           const std::vector<float> &node_vector,
                           int spline_order, TrajectoryBSpline &traj,
                           double time_seg) const;

  Point3D computePointValWithGivenUVal(const Path &control_points,
                                       const std::vector<float> &node_vector,
                                       int control_point_offset, int spline_order, float u) const;

  void computeBSplinePoints(const BSplineKeyPointList &key_point_list, const std::vector<float> &node_vector,
                            int spline_order, int num_path_points_required, TrajectoryBSpline &traj, double max_vel) const;

  Path deriBSplineBasis(const Path &key_point_list, const std::vector<float> &node_vector,
                        int control_point_offset, int spline_order) const;

  Point3D computeDerivative(const Point3D &point2, const Point3D &point1, double du) const;

  Path rearangeKeyPointList(const Path &path) const;

  Point3D evaluatePointValueWithGivenUVal(float u, Point3D *velocity, Point3D *acceleration,
                                          const std::vector<float> &node_vector,
                                          const Path &key_path, const Path &key_vel_list,
                                          const Path &key_acc_list, int spline_order,
                                          double time_ratio) const;

  Point3D lerp(const Point3D &a, const Point3D &b, double t) const;
};

} // namespace sketch