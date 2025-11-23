#pragma once

#include "curveBase.hpp"

namespace utils {
class BezierCurve : public CurveBase {
public:
  BezierCurve() = default;

  void setWaypoints(const Eigen::VectorXd &start,
                    const Eigen::VectorXd &end) override {
    // For a cubic Bezier in 2D, you need 4 control points: P0, P1, P2, P3
    // Suppose we interpret "start" as (P0x, P0y, P1x, P1y)
    // and "end" as (P2x, P2y, P3x, P3y).
    // Or pass all 4 points in one vector, etc. This is up to you.
    if (start.size() != 4 || end.size() != 4) {
      throw std::runtime_error("BezierCurve expects 4D start & 4D end => total 8 values for 2D cubic control points.");
    }
    // P0 and P1
    P0_ << start[0], start[1];
    P1_ << start[2], start[3];
    // P2 and P3
    P2_ << end[0], end[1];
    P3_ << end[2], end[3];

    // Default time interval [0, 1]
    t0_ = 0.0;
    t1_ = 1.0;

    // Default 1D values for boundary conditions
    p0_ = start[0];
    p1_ = end[2];
    v0_ = 3.0 * (start[2] - start[0]);
    v1_ = 3.0 * (end[2] - end[0]);

    // Pre-calculate coefficients
    calculateCoefficients();
  }

  // Method to set boundary conditions
  void updateBoundary(double t0, double t1,
                      double p0, double p1,
                      double v0 = 0.0, double v1 = 0.0,
                      double a0 = 0.0, double a1 = 0.0) override {
    // Store boundary conditions
    t0_ = t0;
    t1_ = t1;
    p0_ = p0;
    p1_ = p1;
    v0_ = v0;
    v1_ = v1;

    // Compute control points from boundary conditions
    double T = t1 - t0;

    // Update P0 and P3 (endpoints)
    P0_[0] = p0;
    P3_[0] = p1;

    // Compute P1 and P2 from velocities
    P1_[0] = P0_[0] + v0 * T / 3.0;
    P2_[0] = P3_[0] - v1 * T / 3.0;

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
    // For a polynomial p(t) = a*t^3 + b*t^2 + c*t + d
    // The derivative is p'(t) = 3*a*t^2 + 2*b*t + c

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
    // For a cubic Bezier, we can convert to polynomial form:
    // B(t) = a*t^3 + b*t^2 + c*t + d
    // where:
    // a = -P0 + 3*P1 - 3*P2 + P3
    // b = 3*P0 - 6*P1 + 3*P2
    // c = -3*P0 + 3*P1
    // d = P0

    double P0 = P0_[0];
    double P1 = P1_[0];
    double P2 = P2_[0];
    double P3 = P3_[0];

    // Compute polynomial coefficients
    double a = -P0 + 3 * P1 - 3 * P2 + P3;
    double b = 3 * P0 - 6 * P1 + 3 * P2;
    double c = -3 * P0 + 3 * P1;
    double d = P0;

    // Store position coefficients
    posCoeffs_.resize(4);
    posCoeffs_ << a, b, c, d;

    // Calculate and store velocity coefficients
    velCoeffs_ = getDerivateCoeffs(posCoeffs_);

    // Calculate and store acceleration coefficients
    accCoeffs_ = getDerivateCoeffs(velCoeffs_);
  }

  // Control points
  Eigen::Vector2d P0_, P1_, P2_, P3_;

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
