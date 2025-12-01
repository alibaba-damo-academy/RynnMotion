#pragma once

#include "curveBase.hpp"

namespace utils {
class CubicCurve : public CurveBase {
public:
  CubicCurve() = default;

  void setWaypoints(const Eigen::VectorXd &start,
                    const Eigen::VectorXd &end) override {
    // For cubic, we only need position and velocity
    if (start.size() < 1 || end.size() < 1) {
      throw std::runtime_error("CubicCurve expects at least position values for start/end.");
    }

    // Store the boundary conditions
    p0_ = start[0];
    p1_ = end[0];
    v0_ = start.size() > 1 ? start[1] : 0.0;
    v1_ = end.size() > 1 ? end[1] : 0.0;

    // Default time interval [0, 1]
    t0_ = 0.0;
    t1_ = 1.0;

    // Pre-calculate coefficients
    calculateCoefficients();
  }

  // Method to set boundary conditions
  void updateBoundary(double t0, double t1,
                      double p0, double p1,
                      double v0 = 0.0, double v1 = 0.0,
                      double a0 = 0.0, double a1 = 0.0) override {
    // Store boundary conditions (ignoring acceleration for cubic)
    t0_ = t0;
    t1_ = t1;
    p0_ = p0;
    p1_ = p1;
    v0_ = v0;
    v1_ = v1;

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

    // Cubic polynomial: a0 + a1*t + a2*t^2 + a3*t^3
    double c0 = p0_;
    double c1 = v0_;
    double c2 = (3.0 * (p1_ - p0_) - (2.0 * v0_ + v1_) * T) / T2;
    double c3 = (2.0 * (p0_ - p1_) + (v0_ + v1_) * T) / T3;

    // Store position coefficients
    posCoeffs_.resize(4);
    posCoeffs_ << c3, c2, c1, c0;

    // Calculate and store velocity coefficients
    velCoeffs_ = getDerivateCoeffs(posCoeffs_);

    // Calculate and store acceleration coefficients
    accCoeffs_ = getDerivateCoeffs(velCoeffs_);
  }

  // Boundary conditions
  double t0_{0.0}, t1_{1.0};
  double p0_{0.0}, p1_{0.0};
  double v0_{0.0}, v1_{0.0};

  // Cached coefficients for better performance
  Eigen::VectorXd posCoeffs_;
  Eigen::VectorXd velCoeffs_;
  Eigen::VectorXd accCoeffs_;
};

} // namespace utils