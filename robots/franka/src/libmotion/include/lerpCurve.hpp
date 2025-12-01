#pragma once

#include "curveBase.hpp"

namespace utils {

class LerpCurve : public CurveBase {
public:
  LerpCurve() = default;

  void setWaypoints(const Eigen::VectorXd &start,
                    const Eigen::VectorXd &end) override {
    if (start.size() != end.size()) {
      throw std::runtime_error("LERP: start and end dimension mismatch.");
    }
    start_ = start;
    end_ = end;

    // Default time interval [0, 1]
    t0_ = 0.0;
    t1_ = 1.0;

    // Default 1D values
    p0_ = start.size() > 0 ? start[0] : 0.0;
    p1_ = end.size() > 0 ? end[0] : 0.0;

    // Pre-calculate coefficients
    calculateCoefficients();
  }

  // Method to set boundary conditions
  void updateBoundary(double t0, double t1,
                      double p0, double p1,
                      double v0 = 0.0, double v1 = 0.0,
                      double a0 = 0.0, double a1 = 0.0) override {
    // Store boundary conditions (ignoring velocity and acceleration for linear)
    t0_ = t0;
    t1_ = t1;
    p0_ = p0;
    p1_ = p1;

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
    // For a linear polynomial p(t) = a*t + b
    // The derivative is p'(t) = a

    Eigen::VectorXd vel_coeffs(1);
    vel_coeffs[0] = pos_coeffs[0]; // Just the coefficient of t

    return vel_coeffs;
  }

private:
  // Calculate and cache the coefficients
  void calculateCoefficients() {
    double T = t1_ - t0_;

    // Linear interpolation: p(t) = a*t + b
    double a = (p1_ - p0_) / T;
    double b = p0_ - a * t0_;

    // Store position coefficients
    posCoeffs_.resize(2);
    posCoeffs_ << a, b;

    // Calculate and store velocity coefficients (constant)
    velCoeffs_.resize(1);
    velCoeffs_[0] = a;

    // Calculate and store acceleration coefficients (zero)
    accCoeffs_.resize(1);
    accCoeffs_[0] = 0.0;
  }

  // Original waypoints
  Eigen::VectorXd start_;
  Eigen::VectorXd end_;

  // Boundary conditions
  double t0_{0.0}, t1_{1.0};
  double p0_{0.0}, p1_{0.0};
  double t_{0.0}; // For derivative evaluation

  // Cached coefficients for better performance
  Eigen::VectorXd posCoeffs_;
  Eigen::VectorXd velCoeffs_;
  Eigen::VectorXd accCoeffs_;
};

} // namespace utils
