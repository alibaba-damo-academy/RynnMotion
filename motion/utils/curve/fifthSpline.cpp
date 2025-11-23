#include "fifthSpline.hpp"

namespace sketch {

FifthSpline::FifthSpline() {
  curvePtr_ = std::make_shared<utils::FifthCurve>();
}

bool FifthSpline::genSpline(const Path &waypoints, double max_vel, OneTrajectoryFifthSpline &traj) {
  if (waypoints.size() < 2) return false;

  // Clear the trajectory to ensure we start fresh
  traj = OneTrajectoryFifthSpline();

  // Pre-allocate memory for trajectory data
  const size_t num_segments = waypoints.size() - 1;
  traj.segment_times.reserve(num_segments);
  traj.x_coeffs.reserve(num_segments);
  traj.y_coeffs.reserve(num_segments);
  traj.z_coeffs.reserve(num_segments);
  traj.vx_coeffs.reserve(num_segments);
  traj.vy_coeffs.reserve(num_segments);
  traj.vz_coeffs.reserve(num_segments);
  traj.ax_coeffs.reserve(num_segments);
  traj.ay_coeffs.reserve(num_segments);
  traj.az_coeffs.reserve(num_segments);

  const double max_acc = 1.0;

  std::vector<Point3D> vel_list = genSplineVelProfile(waypoints, max_vel, max_acc);
  std::vector<double> t_list = computeSegmentTimeAndVel(waypoints, vel_list, max_vel, max_acc);
  std::vector<Point3D> acc_list = genSplineAccProfile(waypoints, vel_list, t_list, max_acc);

  // Calculate polynomial coefficients
  for (size_t i = 0; i < num_segments; ++i) {
    const auto &p0 = waypoints[i];
    const auto &p1 = waypoints[i + 1];
    const auto &v0 = vel_list[i];
    const auto &v1 = vel_list[i + 1];
    const auto &a0 = acc_list[i];
    const auto &a1 = acc_list[i + 1];
    double t1 = t_list[i];
    if (t1 < 1e-6) {
      std::cout << "i: " << i << ", t1 is too small: " << t1 << std::endl;
      continue;
    }

    traj.segment_times.emplace_back(t1);
    addCoefficients(p0.x(), p1.x(), v0.x(), v1.x(), a0.x(), a1.x(), t1, traj.x_coeffs, traj.vx_coeffs, traj.ax_coeffs);
    addCoefficients(p0.y(), p1.y(), v0.y(), v1.y(), a0.y(), a1.y(), t1, traj.y_coeffs, traj.vy_coeffs, traj.ay_coeffs);
    addCoefficients(p0.z(), p1.z(), v0.z(), v1.z(), a0.z(), a1.z(), t1, traj.z_coeffs, traj.vz_coeffs, traj.az_coeffs);
  }

  traj.time_total = std::accumulate(traj.segment_times.begin(), traj.segment_times.end(), 0.0);

  return true;
}

void FifthSpline::addCoefficients(double p0, double p1, double v0, double v1, double a0, double a1, double t,
                                  std::vector<Eigen::VectorXd> &pos_coeffs, std::vector<Eigen::VectorXd> &vel_coeffs,
                                  std::vector<Eigen::VectorXd> &acc_coeffs) {
  curvePtr_->updateBoundary(0, t, p0, p1, v0, v1, a0, a1);
  pos_coeffs.emplace_back(curvePtr_->getPosCoeffs());
  vel_coeffs.emplace_back(curvePtr_->getVelCoeffs());
  acc_coeffs.emplace_back(curvePtr_->getAccCoeffs());
}

std::vector<Point3D> FifthSpline::genSplineAccProfile(const Path &waypoints, const std::vector<Point3D> &vel_list,
                                                      const std::vector<double> &t_list, double max_acc) {
  std::vector<Point3D> acc_list(waypoints.size(), Eigen::Vector3d::Zero());
  size_t num_points = waypoints.size();
  size_t n_step = 1;
  if (num_points <= 2 * n_step) {
    return acc_list;
  }

  size_t idx_end = num_points - n_step;
  for (size_t i = n_step; i < idx_end; ++i) {
    auto &v_pre = vel_list[i - n_step];
    auto &v_next = vel_list[i + n_step];
    double t_seg_sum = std::accumulate(t_list.begin() + i - n_step, t_list.begin() + i + n_step + 1, 0.0);

    if (t_seg_sum > 1e-6) {
      acc_list[i] = (v_next - v_pre) / t_seg_sum;
    }
  }

  return acc_list;
}

std::vector<double> FifthSpline::computeSegmentTimeAndVel(const Path &waypoints, std::vector<Point3D> &vel_list,
                                                          double max_vel, double max_acc) {
  std::vector<double> t_list(waypoints.size() - 1);
  std::vector<bool> xyz_dir_flag_list(waypoints.size() - 1, false);

  size_t i = 0;
  while (i < waypoints.size() - 1) {
    double v0_all = vel_list[i].norm();
    double updated_v_norm = 0.0;
    int update_vel_type = 0; // 0: no update, 1: update v1, 2: update v0
    t_list[i] = computeOneSegmentVelocityAndTime(waypoints, i, vel_list, max_vel, max_acc, updated_v_norm,
                                                 update_vel_type, xyz_dir_flag_list[i]);

    xyz_dir_flag_list[i] = true;
    if (update_vel_type == 2 && v0_all >= 1e-6) {
      vel_list[i] *= updated_v_norm / v0_all;
      i -= 1;
      continue;
    }
    i += 1;
  }

  return t_list;
}

bool FifthSpline::evalSplineAtTime(const OneTrajectoryFifthSpline &trajectory, double t,
                                   Point3D &position, Point3D *velocity, Point3D *acceleration) {
  if (trajectory.segment_times.empty() || t < 0 || t > trajectory.time_total) {
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

  double t_seg = t - accum_time;

  // Evaluate position using FifthCurve's evaluatePolynomial
  position(0) = curvePtr_->evaluatePolynomial(trajectory.x_coeffs[seg_idx], t_seg);
  position(1) = curvePtr_->evaluatePolynomial(trajectory.y_coeffs[seg_idx], t_seg);
  position(2) = curvePtr_->evaluatePolynomial(trajectory.z_coeffs[seg_idx], t_seg);

  if (velocity) {
    (*velocity)(0) = curvePtr_->evaluatePolynomial(trajectory.vx_coeffs[seg_idx], t_seg);
    (*velocity)(1) = curvePtr_->evaluatePolynomial(trajectory.vy_coeffs[seg_idx], t_seg);
    (*velocity)(2) = curvePtr_->evaluatePolynomial(trajectory.vz_coeffs[seg_idx], t_seg);
  }

  if (acceleration) {
    (*acceleration)(0) = curvePtr_->evaluatePolynomial(trajectory.ax_coeffs[seg_idx], t_seg);
    (*acceleration)(1) = curvePtr_->evaluatePolynomial(trajectory.ay_coeffs[seg_idx], t_seg);
    (*acceleration)(2) = curvePtr_->evaluatePolynomial(trajectory.az_coeffs[seg_idx], t_seg);
  }

  return true;
}

double FifthSpline::computeSegmentTimeWithinMaxAcc(double distance, double v0, double v1,
                                                   double max_vel, double max_acc,
                                                   double &updated_v_norm, int &update_vel_type) {
  double t_seg = 0.0;
  updated_v_norm = 0.0;
  update_vel_type = 0; // 0: no update, 1: update v1, 2: update v0

  constexpr double min_distance = 1e-6;
  if (abs(distance) <= min_distance) {
    return t_seg;
  }
  double tf_smooth_vel = std::numeric_limits<double>::max();
  if (abs(v1) + abs(v0) > 1e-6) {
    tf_smooth_vel = abs(distance) / (0.5 * (abs(v1) + abs(v0)));
  }

  double distance_acc = abs(v1 * v1 - v0 * v0) / (2.0 * max_acc);
  if (abs(distance) >= distance_acc - 1e-8) {
    t_seg = computeTimeWithSufficientDis(distance, v0, v1, max_vel, max_acc);
  } else if (v0 * v1 >= 0 && abs(distance) > 0) {
    computeAccAndDecProcess(distance, v0, v1, max_acc, updated_v_norm, t_seg, update_vel_type);
  } else {
    std::cout << "[FifthSpline] v1 >= v0 && v0 >= 0 && distance > 0: v0: " << v0 << ", v1: " << v1 << ", distance: " << distance << std::endl;
    return t_seg;
  }

  return tf_smooth_vel > 1.2 * t_seg ? t_seg : tf_smooth_vel;
}

double FifthSpline::computeTimeWithSufficientDis(double distance, double v0, double v1, double max_vel, double max_acc) {
  const double v0_squared = v0 * v0;
  const double v1_squared = v1 * v1;
  const double max_vel_squared = max_vel * max_vel;
  double max_speed_squared = std::max(max_vel_squared, std::max(v1_squared, v0_squared));
  if (max_speed_squared <= max_vel_squared) {
    const double s_acc = (max_vel_squared - v0_squared) / (2.0 * max_acc);
    const double s_dec = (max_vel_squared - v1_squared) / (2.0 * max_acc);
    if (abs(distance) >= s_acc + s_dec) {
      const double t_acc = (max_vel - abs(v0)) / max_acc;               // Acceleration time
      const double t_dec = (max_vel - abs(v1)) / max_acc;               // Deceleration time
      const double t_const = (abs(distance) - s_acc - s_dec) / max_vel; // Constant velocity time
      return t_acc + t_dec + t_const;
    } else {
      double vm = std::sqrt((max_acc * abs(distance) + 0.5 * (v0_squared + v1_squared)));
      return (2 * vm - abs(v0) - abs(v1)) / max_acc;
    }
  } else {
    double v_bigger = std::max(abs(v0), abs(v1));
    double t1 = abs(abs(v1) - abs(v0)) / max_acc;
    double s1 = abs(v1_squared - v0_squared) / (2.0 * max_acc);
    double t2 = (abs(distance) - s1) / v_bigger;
    return t1 + t2;
  }
}

void FifthSpline::computeAccAndDecProcess(double distance, double v0, double v1,
                                          double max_acc, double &updated_v_norm,
                                          double &t_seg, int &update_vel_type) {
  double v_bigger = abs(v0) >= abs(v1) ? v0 : v1;
  double v_smaller = abs(v0) < abs(v1) ? v0 : v1;
  double required_acc = (v_bigger * v_bigger - v_smaller * v_smaller) / (2 * abs(distance));
  if (required_acc > 1.1 * max_acc) {
    int vel_sign = utils::sign(v_bigger);
    updated_v_norm = vel_sign * std::sqrt(2 * abs(distance) * max_acc + v_smaller * v_smaller);
    t_seg = (abs(updated_v_norm) - abs(v_smaller)) / max_acc;
    update_vel_type = abs(v1) >= abs(v0) ? 1 : 2;
  } else {
    t_seg = (abs(v_bigger) - abs(v_smaller)) / required_acc;
  }
}

double FifthSpline::computeOneSegmentVelocityAndTime(const Path &waypoints, size_t i, std::vector<Point3D> &vel_list,
                                                     const double max_vel, const double max_acc, double &updated_v_norm,
                                                     int &update_vel_type, bool xyz_dir_flag) {
  const auto &p0 = waypoints[i];
  const auto &p1 = waypoints[i + 1];

  double v0_all = vel_list[i].norm();
  double t_seg = 0.0;

  if (i < waypoints.size() - 2) {
    const auto &p2 = waypoints[i + 2];
    Eigen::Vector3d vec1(p1 - p0);
    Eigen::Vector3d vec2(p2 - p1);
    Eigen::Vector3d vec3(p2 - p0);
    Eigen::Vector3d triangle_side_length(vec1.norm(), vec2.norm(), vec3.norm());
    double ratio = computeSegmentLengthRatio(triangle_side_length);
    Eigen::Vector3d dir_vec(vec1 + ratio * vec2);
    if (dir_vec.norm() > 1e-6) {
      dir_vec.normalize();
      if (!xyz_dir_flag) {
        updateXYZVelByTangentDir(vel_list[i + 1], max_vel, dir_vec, triangle_side_length);
      }

      bool update_vel_flag_xyz = true;
      while (update_vel_flag_xyz) {
        update_vel_flag_xyz = false;

        Point3D updated_vel_xyz(Eigen::Vector3d::Zero());
        Eigen::Vector3i update_vel_type_xyz(Eigen::Vector3i::Zero());
        Point3D time_vec;
        time_vec.x() = computeSegmentTimeWithinMaxAcc(vec1.x(), vel_list[i].x(), vel_list[i + 1].x(), max_vel, max_acc, updated_vel_xyz.x(), update_vel_type_xyz.x());
        time_vec.y() = computeSegmentTimeWithinMaxAcc(vec1.y(), vel_list[i].y(), vel_list[i + 1].y(), max_vel, max_acc, updated_vel_xyz.y(), update_vel_type_xyz.y());
        time_vec.z() = computeSegmentTimeWithinMaxAcc(vec1.z(), vel_list[i].z(), vel_list[i + 1].z(), max_vel, max_acc, updated_vel_xyz.z(), update_vel_type_xyz.z());

        double txyz_max = time_vec.maxCoeff();
        t_seg = txyz_max;

        if (update_vel_type_xyz.any()) {
          if ((update_vel_type_xyz.array() == 1).any()) {
            // if vel_list[i + 1] is too large, need to reduce, and update vel_list[i + 1]
            reduceNextVel(vel_list[i + 1], updated_vel_xyz, update_vel_type_xyz, update_vel_flag_xyz);
          } else if ((update_vel_type_xyz.array() == 2).any()) {
            // if vel_list[i] is too large, need to reduce, and update vel_list[i]
            reduceCurrentVel(vel_list[i], updated_vel_xyz, update_vel_type_xyz, v0_all, updated_v_norm, update_vel_type);
            break;
          } else {
            // check if the time difference between time_vec.x() time_vec.y() and time_vec.z(), if any one is too small, need to reduce corresponding velocity
            bool break_flag = reduceBothVels(vel_list, i, txyz_max, time_vec, vec1, max_acc, v0_all, updated_v_norm, update_vel_type);
            if (break_flag) {
              break;
            }
          }
        }
      }
    } else {
      std::cout << "i: " << i << ", dir_vec.norm() is 0.0, (" << dir_vec.transpose() << std::endl;
      std::cerr << "p0: (" << p0.transpose() << "), p1: (" << p1.transpose() << "), p2: (" << p2.transpose() << ")" << std::endl;
    }
  } else {
    double distance = (p1 - p0).norm();
    double v1_all = vel_list[i + 1].norm();
    t_seg = computeSegmentTimeWithinMaxAcc(distance, v0_all, v1_all, max_vel, max_acc, updated_v_norm, update_vel_type);
  }
  if (t_seg < 1.0e-6) {
    std::cout << "i: " << i << ", t_seg is 0.0, " << t_seg << std::endl;
  }
  return t_seg;
}

void FifthSpline::reduceNextVel(Point3D &v1, const Point3D &updated_vel_xyz,
                                const Eigen::Vector3i &update_vel_type_xyz,
                                bool &update_vel_flag_xyz) {
  Eigen::Vector3d coe(Eigen::Vector3d::Ones());
  for (int j = 0; j < 3; ++j) {
    if (update_vel_type_xyz[j] == 1 && std::abs(v1[j]) >= 1e-6) {
      coe[j] = updated_vel_xyz[j] / v1[j];
    }
  }
  double coe_min = coe.minCoeff();
  v1 *= coe_min;
  update_vel_flag_xyz = true;
}

void FifthSpline::reduceCurrentVel(const Point3D &v0, const Point3D &updated_vel_xyz,
                                   const Eigen::Vector3i &update_vel_type_xyz,
                                   double v0_all, double &updated_v_norm, int &update_vel_type) {
  Point3D coe(Eigen::Vector3d::Ones());
  for (int j = 0; j < 3; ++j) {
    if (update_vel_type_xyz[j] == 2 && std::abs(v0[j]) >= 1e-6) {
      coe[j] = updated_vel_xyz[j] / v0[j];
    }
  }
  updated_v_norm = v0_all * coe.minCoeff();
  update_vel_type = 2;
}

bool FifthSpline::reduceBothVels(std::vector<Point3D> &vel_list, size_t i,
                                 double txyz_max, const Point3D &time_vec,
                                 const Point3D &vec1, double max_acc, double v0_all,
                                 double &updated_v_norm, int &update_vel_type) {
  if (txyz_max > 1e-6) {
    Eigen::Vector3d coe(Eigen::Vector3d::Ones());
    for (int j = 0; j < 3; ++j) {
      if (time_vec[j] < txyz_max) {
        coe[j] = computeFastAxisVels(vel_list[i][j], vel_list[i + 1][j], vec1[j], time_vec[j], txyz_max, max_acc, v0_all, updated_v_norm, update_vel_type);
        if (update_vel_type == 2) {
          vel_list[i + 1][j] *= coe[j];
          if (std::abs(vel_list[i + 1][j]) < std::abs(vel_list[i][j])) {
            vel_list[i][j] *= coe[j];
          }
          return true;
        }
      }
    }
  }

  return false;
}

double FifthSpline::computeFastAxisVels(double v0, double v1, double distance,
                                        double t_xyz, double txyz_max, double max_acc,
                                        double v0_all, double &updated_v_norm, int &update_vel_type) {
  if (t_xyz < 1.0e-6) {
    return 1.0;
  }
  if (abs(txyz_max - t_xyz) / txyz_max > 0.001) {
    if (abs(v0) + abs(v1) < 1e-6) {
      return 1.0;
    }
    double t_slow_1 = 2 * abs(distance) / (abs(v0) + abs(v1));
    if (t_slow_1 < txyz_max * 0.9) {
      updated_v_norm = v0_all;
      update_vel_type = 2;
      double v1_new = 2 * abs(distance) / txyz_max - abs(v0);
      if (v1_new > abs(v0)) {
        return abs(v1_new / v1);
      }
      return t_slow_1 / txyz_max;
    } else {
      if (max_acc * abs(distance) < 0.5 * (v0 * v0 + v1 * v1)) {
        double vm = std::sqrt(0.5 * (v0 * v0 + v1 * v1) - max_acc * abs(distance));
        double t_slow_2 = (abs(v0) + abs(v1) - 2 * vm) / (max_acc);
        if (t_slow_2 <= 0) {
          std::cout << "t_slow_2 <= 0: " << t_slow_2 << std::endl;
        }
        if (t_slow_2 < txyz_max * 0.995) {
          updated_v_norm = v0_all;
          update_vel_type = 2;
          return t_slow_2 / txyz_max;
        }
      }
    }
  }
  return 1.0;
}

double FifthSpline::computeSegmentLengthRatio(const Eigen::Vector3d &triangle_side_length) {
  double a = triangle_side_length[0];
  double b = triangle_side_length[1];
  double c = triangle_side_length[2];
  double angle_ab = (M_PI - acos((a * a + b * b - c * c) / (2.0 * a * b))) * 180.0 / M_PI;
  double ratio = 1.0;
  if (angle_ab > 90.0) {
    ratio = 0.0;
  } else if (b >= 1e-6) {
    if (angle_ab > 45.0) {
      ratio = (90.0 - angle_ab) / 45.0 * a / b;
    } else {
      ratio = a / b;
    }
  }
  return ratio;
}

void FifthSpline::updateXYZVelByTangentDir(Point3D &v1, double max_vel,
                                           const Eigen::Vector3d &dir_vec,
                                           const Eigen::Vector3d &triangle_side_length) {
  Eigen::Array3d v_all;
  for (int j = 0; j < 3; ++j) {
    v_all[j] = velAllByXYZ(v1[j], dir_vec[j], max_vel);
  }

  double v_max_by_radius = computeMaxVelByRadius(triangle_side_length);

  double v_all_min = std::min(v_all.minCoeff(), v_max_by_radius);
  if (v1.norm() < 1e-6) {
    v_all_min = 0.0;
  }

  v1 = (v1.array() / v_all) * v_all_min;
}

double FifthSpline::velAllByXYZ(double &v1_xyz, double dir_xyz, double max_vel) {
  if (abs(dir_xyz) < 1e-6 || abs(v1_xyz) < 1e-5) {
    v1_xyz = 0.0;
    return 2 * max_vel;
  }

  return abs(v1_xyz / dir_xyz);
}

double FifthSpline::computeMaxVelByRadius(const Eigen::Vector3d &triangle_side_length) {
  Eigen::Array3d tri_arr = triangle_side_length.array();
  double s = tri_arr.sum() / 2.0;
  double area = sqrt(s * (s - tri_arr[0]) * (s - tri_arr[1]) * (s - tri_arr[2]));
  double radius = 10000.0;
  if (s - tri_arr[0] > 1e-6 && s - tri_arr[1] > 1e-6 && s - tri_arr[2] > 1e-6) {
    radius = tri_arr.prod() / (4 * area);
  }
  double acc_central = 0.5;
  return std::sqrt(acc_central * radius);
}

std::vector<Point3D> FifthSpline::genSplineVelProfile(const Path &waypoints, double max_vel,
                                                      double acc_max) {
  std::vector<Point3D> segment_max_vel;
  segment_max_vel.reserve(waypoints.size());
  segment_max_vel.emplace_back(Eigen::Vector3d::Zero());
  std::vector<Point3D> waypoints_dist;
  waypoints_dist.reserve(waypoints.size());
  waypoints_dist.emplace_back(Eigen::Vector3d::Zero());
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    waypoints_dist.emplace_back(waypoints[i + 1] - waypoints[i]);
  }

  for (size_t i = 1; i < waypoints.size(); ++i) {
    auto &p1 = waypoints[i - 1];
    auto &p2 = waypoints[i];

    Point3D vmax(max_vel * Eigen::Vector3d::Ones());
    if (i < waypoints.size() - 1) {
      auto &p3 = waypoints[i + 1];
      for (int j = 0; j < 3; ++j) {
        if ((p2[j] >= p1[j] && p3[j] <= p2[j]) || (p2[j] <= p1[j] && p3[j] >= p2[j])) {
          vmax[j] = 0.0;
        } else if (p2[j] < p1[j] && vmax[j] > 1e-6) {
          vmax[j] = -1 * vmax[j];
        }
      }
    } else {
      vmax = Eigen::Vector3d::Zero();
    }
    segment_max_vel.emplace_back(vmax);

    for (int j = 0; j < 3; ++j) {
      int k = i;
      bool update_xyz_vel_flag = true;
      while (k > 0 && update_xyz_vel_flag) {
        update_xyz_vel_flag = false;
        double prev_vmax = segment_max_vel[k - 1][j];
        double cur_vmax = segment_max_vel[k][j];
        double dis_to_prev_point = waypoints_dist[k][j] - waypoints_dist[k - 1][j];
        if (abs(dis_to_prev_point) < 1e-6) {
          dis_to_prev_point = 1e-6;
        }
        if (abs(cur_vmax) < abs(prev_vmax)) {
          double required_dec = (prev_vmax * prev_vmax - cur_vmax * cur_vmax) / (2 * abs(dis_to_prev_point));
          if (required_dec > acc_max + 0.01) {
            segment_max_vel[k - 1][j] = utils::sign(prev_vmax) * std::sqrt(cur_vmax * cur_vmax + 2 * acc_max * abs(dis_to_prev_point));
            update_xyz_vel_flag = true;
          }
        } else if (abs(cur_vmax) > abs(prev_vmax)) {
          double required_acc = (cur_vmax * cur_vmax - prev_vmax * prev_vmax) / (2 * abs(dis_to_prev_point));
          if (required_acc > acc_max) {
            segment_max_vel[k][j] = utils::sign(cur_vmax) * std::sqrt(prev_vmax * prev_vmax + 2 * acc_max * abs(dis_to_prev_point));
          }
        }
        k--;
      }
    }
  }
  return segment_max_vel;
}

} // namespace sketch