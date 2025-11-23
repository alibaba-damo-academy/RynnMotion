#include "bspline.hpp"

namespace sketch {

void BSpline::genSpline(const Path &origin_control_points, TrajectoryBSpline &traj,
                        double max_vel, int spline_order, int num_path_points_required,
                        bool rearrange_key_points_flag) const {
  if (origin_control_points.size() < 2) {
    std::cout << "The number of control points is less than 2, please check!" << std::endl;
    return;
  }
  Path rearranged_control_points;
  if (rearrange_key_points_flag) {
    rearranged_control_points = rearangeKeyPointList(origin_control_points);
  } else {
    rearranged_control_points = origin_control_points;
  }

  BSplineKeyPointList key_point_list;
  key_point_list.key_path_point_list.insert(key_point_list.key_path_point_list.end(), rearranged_control_points.begin(), rearranged_control_points.end());

  int num_key_points = rearranged_control_points.size();

  if (num_key_points <= spline_order) {
    std::cerr << "Number of control points must be greater than the B-spline order." << std::endl;
    return;
  }

  std::vector<float> node_vector = quasiUniformMode(num_key_points - 1, spline_order);

  key_point_list.key_vel_point_list = deriBSplineBasis(key_point_list.key_path_point_list, node_vector, 0, spline_order);
  key_point_list.key_acc_point_list = deriBSplineBasis(key_point_list.key_vel_point_list, node_vector, 1, spline_order - 1);

  traj.key_paths.emplace_back(key_point_list.key_path_point_list);
  traj.key_vel_lists.emplace_back(key_point_list.key_vel_point_list);
  traj.key_acc_lists.emplace_back(key_point_list.key_acc_point_list);
  traj.node_vectors.emplace_back(node_vector);
  traj.b_spline_orders.emplace_back(spline_order);
  traj.last_point = origin_control_points.back();

  computeBSplinePoints(key_point_list, node_vector, spline_order, num_path_points_required, traj, max_vel);
}

void BSpline::computeBSplinePoints(const BSplineKeyPointList &key_point_list, const std::vector<float> &node_vector,
                                   int spline_order, int num_path_points_required, TrajectoryBSpline &traj, double max_vel) const {
  int num_key_points = key_point_list.key_path_point_list.size();
  if (num_key_points <= spline_order) {
    float dis_line = computeLineDistance(key_point_list.key_path_point_list);
    double time_seg = dis_line / (0.7 * max_vel);
    traj.segment_times.emplace_back(time_seg);
    Path path_points, vel_points, acc_points;
    path_points.reserve(num_key_points);
    vel_points.reserve(num_key_points);
    acc_points.reserve(num_key_points);
    std::vector<double> t_list;
    t_list.reserve(num_key_points);
    for (int i = 0; i < num_key_points; i++) {
      path_points.emplace_back(key_point_list.key_path_point_list[i]);
      vel_points.emplace_back(Point3D(0, 0, 0));
      acc_points.emplace_back(Point3D(0, 0, 0));
      t_list.emplace_back(i * 1.0 / (num_key_points - 1) * time_seg);
    }
    traj.path_list.emplace_back(path_points);
    traj.vel_points_list.emplace_back(vel_points);
    traj.acc_points_list.emplace_back(acc_points);
    traj.t_seg_list.emplace_back(t_list);
    traj.time_total_all_trajs += time_seg;
    return;
  }

  int index_end = num_key_points - 1;

  float dis_line = computeLineDistance(key_point_list.key_path_point_list);
  int path_point_num = 10;
  if (num_path_points_required > 0) {
    path_point_num = num_path_points_required;
  } else {
    int num_path_dis = int(dis_line * 500);
    if (path_point_num < num_path_dis) {
      path_point_num = num_path_dis;
    }
  }

  double time_seg = dis_line / (0.5 * max_vel);
  traj.segment_times.emplace_back(time_seg);
  traj.time_total_all_trajs += time_seg;

  std::vector<float> u_list(path_point_num, 0);
  for (int i = 0; i < path_point_num; i++) {
    u_list[i] = node_vector[0] + i * 1.0 / (path_point_num - 1) * (node_vector[index_end + spline_order + 1] - node_vector[0]);
  }

  computePathPointVal(u_list, key_point_list, node_vector, spline_order, traj, time_seg);
}

void BSpline::computePathPointVal(const std::vector<float> &u_list,
                                  const BSplineKeyPointList &key_point_list,
                                  const std::vector<float> &node_vector,
                                  int spline_order, TrajectoryBSpline &traj,
                                  double time_seg) const {
  int num_u_list = u_list.size();
  Path path_points, vel_points, acc_points;
  path_points.reserve(num_u_list);
  vel_points.reserve(num_u_list);
  acc_points.reserve(num_u_list);
  std::vector<double> t_list;
  t_list.reserve(num_u_list);
  double time_ratio = time_seg / u_list[num_u_list - 1];
  for (int j = 0; j < num_u_list; j++) {
    float u = u_list[j];
    t_list.emplace_back(u / u_list[num_u_list - 1] * time_seg);
    Point3D point = computePointValWithGivenUVal(key_point_list.key_path_point_list, node_vector, 0, spline_order, u);
    path_points.emplace_back(point);
    if (j > 0) {
      Point3D &point2 = path_points[j];
      Point3D &point1 = path_points[j - 1];
      double du = u_list[j] - u_list[j - 1];
      Point3D vel = computeDerivative(point2, point1, du) / time_ratio;
      if (j == 1) {
        vel_points.emplace_back(vel);
      }
      vel_points.emplace_back(vel);

      Point3D &vel2 = vel_points[j];
      Point3D &vel1 = vel_points[j - 1];
      Point3D acc = computeDerivative(vel2, vel1, du) / time_ratio;
      if (j == 1) {
        acc_points.emplace_back(acc);
      }
      acc_points.emplace_back(acc);
    }

    // Point3D vel = computePointValWithGivenUVal(key_point_list.key_vel_point_list, node_vector, 1, spline_order - 1, u);
    // vel_points.emplace_back(vel);

    // Point3D acc = computePointValWithGivenUVal(key_point_list.key_acc_point_list, node_vector, 2, spline_order - 2, u);
    // acc_points.emplace_back(acc);
  }

  if (!path_points.empty()) {
    int num_key_points = key_point_list.key_path_point_list.size();
    int index_end = num_key_points - 1;
    path_points.back() = key_point_list.key_path_point_list[index_end];
  }
  traj.path_list.emplace_back(path_points);
  traj.vel_points_list.emplace_back(vel_points);
  traj.acc_points_list.emplace_back(acc_points);
  traj.t_seg_list.emplace_back(t_list);
}

Point3D BSpline::computeDerivative(const Point3D &point2, const Point3D &point1, double du) const {
  if (std::abs(du) < 1e-8) {
    return Point3D(0.0, 0.0, 0.0);
  }
  return (point2 - point1) / du;
}

Point3D BSpline::computePointValWithGivenUVal(const Path &control_points, const std::vector<float> &node_vector,
                                              int control_point_offset, int spline_order, float u) const {
  Point3D result(0.0f, 0.0f, 0.0f);
  int num_control_points = control_points.size();
  int node_size = node_vector.size();
  for (int i = 0; i < num_control_points; ++i) {
    int basis_index = i + control_point_offset;

    if (basis_index < 0 || basis_index >= node_size) {
      continue;
    }

    float basis_value = bSplineBasis(node_vector, basis_index, spline_order, u);

    result += control_points[i] * basis_value;
  }

  return result;
}

float BSpline::bSplineBasis(const std::vector<float> &node_vector,
                            int i, int spline_order, float u) const {
  int node_size = node_vector.size();
  if (i < 0 || i >= node_size - spline_order - 1 || spline_order < 0) {
    return 0.0f;
  }

  if (spline_order == 0) {
    if (u >= node_vector[i] && u < node_vector[i + 1]) {
      return 1.0f;
    }

    if (i == node_size - 2 && u == node_vector[i + 1]) {
      return 1.0f;
    }
    return 0.0f;
  }

  float length1 = node_vector[i + spline_order] - node_vector[i];
  float length2 = node_vector[i + spline_order + 1] - node_vector[i + 1];

  float alpha = length1 != 0 ? (u - node_vector[i]) / length1 : 0;
  float beta = length2 != 0 ? (node_vector[i + spline_order + 1] - u) / length2 : 0;

  return alpha * bSplineBasis(node_vector, i, spline_order - 1, u) + beta * bSplineBasis(node_vector, i + 1, spline_order - 1, u);
}

bool BSpline::evalSplineAtTime(const TrajectoryBSpline &trajectory, double t,
                               Point3D &position, Point3D *velocity, Point3D *acceleration) const {
  if (trajectory.segment_times.empty() || t < 0 || t > trajectory.time_total_all_trajs) {
    return false;
  }

  double accum_time = 0.0;
  size_t seg_idx = 0;

  // Find the segment that contains time t
  while (seg_idx < trajectory.segment_times.size() && t > accum_time + trajectory.segment_times[seg_idx]) {
    accum_time += trajectory.segment_times[seg_idx];
    seg_idx++;
  }

  if (seg_idx >= trajectory.segment_times.size()) {
    return false;
  }

  // Calculate the time within the segment
  double t_seg = t - accum_time;

  double u = t_seg / trajectory.segment_times[seg_idx];

  position = evaluatePointValueWithGivenUVal(u, velocity, acceleration, trajectory.node_vectors[seg_idx],
                                             trajectory.key_paths[seg_idx], trajectory.key_vel_lists[seg_idx],
                                             trajectory.key_acc_lists[seg_idx], trajectory.b_spline_orders[seg_idx],
                                             trajectory.segment_times[seg_idx]);
  return true;
}

Point3D BSpline::evaluatePointValueWithGivenUVal(float u, Point3D *velocity, Point3D *acceleration,
                                                 const std::vector<float> &node_vector,
                                                 const Path &key_path, const Path &key_vel_list,
                                                 const Path &key_acc_list, int spline_order,
                                                 double time_ratio) const {
  int num_key_points = key_path.size();
  if (num_key_points <= spline_order) {
    return Point3D(0.0f, 0.0f, 0.0f);
  }

  Point3D point = computePointValWithGivenUVal(key_path, node_vector, 0, spline_order, u);

  if (velocity) {
    Point3D vel = computePointValWithGivenUVal(key_vel_list, node_vector, 1, spline_order - 1, u);
    vel = vel / time_ratio;
    (*velocity)(0) = vel.x();
    (*velocity)(1) = vel.y();
    (*velocity)(2) = vel.z();
  }

  if (acceleration && spline_order >= 2) {
    Point3D acc = computePointValWithGivenUVal(key_acc_list, node_vector, 2, spline_order - 2, u);
    acc = acc / time_ratio;
    (*acceleration)(0) = acc.x();
    (*acceleration)(1) = acc.y();
    (*acceleration)(2) = acc.z();
  }
  return point;
}

Path BSpline::deriBSplineBasis(const Path &key_point_list, const std::vector<float> &node_vector,
                               int control_point_offset, int spline_order) const {
  int num_points = key_point_list.size();
  Path key_point_list_deri;
  key_point_list_deri.reserve(num_points - 1);
  for (int i = 0; i < num_points - 1; i++) {
    Point3D point;
    if (abs(node_vector[i + spline_order + 1] - node_vector[i + 1]) > 1e-6) {
      point = spline_order / (node_vector[i + control_point_offset + spline_order + 1] - node_vector[i + control_point_offset + 1]) * (key_point_list[i + 1] - key_point_list[i]);
    }
    key_point_list_deri.emplace_back(point);
  }
  return key_point_list_deri;
}

std::vector<float> BSpline::quasiUniformMode(int index_end, int k) const {
  std::vector<float> node_vector(index_end + k + 2, 0.f);
  int piece_wise = index_end - k + 1;
  if (piece_wise == 1) {
    for (int i = index_end + 1; i < index_end + k + 2; i++) {
      node_vector[i] = 1;
    }
  } else {
    int flag = 1;
    while (flag != piece_wise) {
      node_vector[k + flag] = node_vector[k + flag] + flag * 1.0 / (piece_wise * 1.0);
      flag = flag + 1;
    }

    for (int i = index_end + 1; i < index_end + k + 2; i++) {
      node_vector[i] = 1;
    }
  }
  return node_vector;
}

Path BSpline::rearangeKeyPointList(const Path &path) const {
  int num_points = path.size();
  if (num_points <= 1) return path;

  double total_length = computeLineDistance(path);
  if (total_length < 1e-6) return path;

  double avg_dis = total_length / (num_points - 1);
  Path result;
  result.reserve(2 * num_points);

  result.emplace_back(path[0]);
  for (int i = 0; i < num_points - 1; ++i) {
    const auto &p0 = path[i];
    const auto &p1 = path[i + 1];

    double segment_length = (p1 - p0).norm();
    if (segment_length > 2 * avg_dis) {
      int num_add = static_cast<int>(segment_length / avg_dis) - 1;
      for (int j = 1; j <= num_add; ++j) {
        double t = static_cast<double>(j) / (num_add + 1);
        result.emplace_back(lerp(p0, p1, t));
      }
    }
    result.emplace_back(p1);
  }

  if (result.size() >= 2) {
    const auto &first = result[0];
    const auto &second = result[1];
    double seg_len = (second - first).norm();
    if (seg_len > 0.1 * avg_dis) {
      Point3D new_point = first + 0.05 * avg_dis / seg_len * (second - first);
      result.insert(result.begin() + 1, new_point);
      if (seg_len > 0.3 * avg_dis) {
        new_point = first + 0.15 * avg_dis / seg_len * (second - first);
        result.insert(result.begin() + 2, new_point);
      }
    }
  }

  int n = result.size();
  if (n >= 2) {
    const auto &last = result[n - 1];
    const auto &prev = result[n - 2];
    double seg_len = (last - prev).norm();
    if (seg_len > 0.1 * avg_dis) {
      Point3D new_point = last + 0.05 * avg_dis / seg_len * (prev - last);
      result.insert(result.begin() + n - 1, new_point);
      if (seg_len > 0.3 * avg_dis) {
        new_point = last + 0.15 * avg_dis / seg_len * (prev - last);
        result.insert(result.begin() + n - 1, new_point);
      }
    }
  }
  return result;
}

Point3D BSpline::lerp(const Point3D &a, const Point3D &b, double t) const {
  return a + t * (b - a);
}

double BSpline::computeLineDistance(const Path &path) const {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

} // namespace sketch
