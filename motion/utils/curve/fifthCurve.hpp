#pragma once

#include "curveBase.hpp"

namespace utils {
class FifthCurve : public CurveBase {
public:
  FifthCurve() = default;

  // For a minimal example, let's just store position & velocity conditions, etc.
  void setWaypoints(const Eigen::VectorXd &start,
                    const Eigen::VectorXd &end) override {
    // Assume [ start( pos, vel, acc ), end( pos, vel, acc ) ] in 1D
    if (start.size() != 3 || end.size() != 3) {
      throw std::runtime_error("FifthCurve expects 3D (pos, vel, acc) for start/end.");
    }

    // Store the boundary conditions
    p0_ = start[0];
    v0_ = start[1];
    a0_ = start[2];
    p1_ = end[0];
    v1_ = end[1];
    a1_ = end[2];

    // Default time interval [0, 1]
    t0_ = 0.0;
    t1_ = 1.0;

    // Pre-calculate coefficients for better performance
    calculateCoefficients();
  }

  // Method to set boundary conditions
  void updateBoundary(double t0, double t1,
                      double p0, double p1,
                      double v0 = 0.0, double v1 = 0.0,
                      double a0 = 0.0, double a1 = 0.0) override {
    // Store all boundary conditions
    t0_ = t0;
    t1_ = t1;
    p0_ = p0;
    p1_ = p1;
    v0_ = v0;
    v1_ = v1;
    a0_ = a0;
    a1_ = a1;

    // Pre-calculate coefficients for better performance
    calculateCoefficients();
  }

  // Get position coefficients using cached values
  Eigen::VectorXd getPosCoeffs() const override {
    return posCoeffs_;
  }

  // Get velocity coefficients directly
  Eigen::VectorXd getVelCoeffs() const override {
    return velCoeffs_;
  }

  // Get acceleration coefficients directly
  Eigen::VectorXd getAccCoeffs() const override {
    return accCoeffs_;
  }

  Eigen::VectorXd getDerivateCoeffs(const Eigen::VectorXd &pos_coeffs) const override {
    // For a polynomial p(t) = a_n*t^n + a_{n-1}*t^{n-1} + ... + a_1*t + a_0,
    // its derivative is p'(t) = n*a_n*t^{n-1} + (n-1)*a_{n-1}*t^{n-2} + ... + a_1

    const int n = pos_coeffs.size();
    Eigen::VectorXd vel_coeffs(n - 1);

    for (int i = 0; i < n - 1; ++i) {
      vel_coeffs[i] = (n - 1 - i) * pos_coeffs[i];
    }

    return vel_coeffs;
  }

private:
  // Calculate and cache the coefficients
  void calculateCoefficients() {
    double T = t1_ - t0_;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    double c0 = p0_;
    double c1 = v0_;
    double c2 = 0.5 * a0_;
    double c3 = (20 * (p1_ - p0_) - (8 * v1_ + 12 * v0_) * T - (3 * a0_ - a1_) * T2) / (2 * T3);
    double c4 = (30 * (p0_ - p1_) + (14 * v1_ + 16 * v0_) * T + (3 * a0_ - 2 * a1_) * T2) / (2 * T4);
    double c5 = (12 * (p1_ - p0_) - 6 * (v1_ + v0_) * T - (a0_ - a1_) * T2) / (2 * T5);

    // Store position coefficients
    posCoeffs_.resize(6);
    posCoeffs_ << c5, c4, c3, c2, c1, c0;

    // Calculate and store velocity coefficients
    velCoeffs_ = getDerivateCoeffs(posCoeffs_);

    // Calculate and store acceleration coefficients
    accCoeffs_ = getDerivateCoeffs(velCoeffs_);
  }

  // Boundary conditions
  double t0_{0.0}, t1_{1.0};
  double p0_{0.0}, v0_{0.0}, a0_{0.0};
  double p1_{0.0}, v1_{0.0}, a1_{0.0};

  // Cached coefficients for better performance
  Eigen::VectorXd posCoeffs_;
  Eigen::VectorXd velCoeffs_;
  Eigen::VectorXd accCoeffs_;
};

} // namespace utils