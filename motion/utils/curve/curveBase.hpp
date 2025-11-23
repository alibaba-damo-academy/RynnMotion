#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace utils {
// Choose whichever typed container fits your usage—here I use VectorXd to
// hold start/end waypoints. Typically for 1D you might use double, or for
// 6D (robot joint angles) you might use a 6-element vector, etc.
enum class CurveMethod {
  kLerp,
  kCubicPoly,
  kFifthPoly,
  kBezier
};

class CurveBase {
public:
  virtual ~CurveBase() = default;

  // Set start and end points (or multiple control points, depending on the curve).
  // For polynomials, this might just be start & end. For Bezier, you'd pass more points.
  // Deprecated: Use updateBoundary instead
  virtual void setWaypoints(const Eigen::VectorXd &start,
                            const Eigen::VectorXd &end) = 0;

  // Method to set boundary conditions for the curve
  virtual void updateBoundary(double t0, double t1,
                              double p0, double p1,
                              double v0 = 0.0, double v1 = 0.0,
                              double a0 = 0.0, double a1 = 0.0) = 0;

  // Get position polynomial coefficients
  // This version uses the boundary conditions set by updateBoundary
  virtual Eigen::VectorXd getPosCoeffs() const = 0;

  // Get velocity polynomial coefficients directly
  virtual Eigen::VectorXd getVelCoeffs() const = 0;

  // Get acceleration polynomial coefficients directly
  virtual Eigen::VectorXd getAccCoeffs() const = 0;

  // Get velocity polynomial coefficients from position coefficients
  // Deprecated: Use getVelCoeffs() instead
  virtual Eigen::VectorXd getDerivateCoeffs(const Eigen::VectorXd &pos_coeffs) const = 0;

  /**
   * @brief Evaluates a polynomial at time t
   * @param coeffs Vector of polynomial coefficients [a_n, a_{n-1}, ..., a_0]
   * @param t Time at which to evaluate
   * @return Value of polynomial at time t
   */
  virtual double evaluatePolynomial(const Eigen::VectorXd &coeffs, double t) const {
    const int n = coeffs.size();
    double result = coeffs[0];

    for (int i = 1; i < n; ++i) {
      result = result * t + coeffs[i];
    }

    return result;
  }

  // (Optional) get second derivative, etc. You can add more as needed.
protected:
};

} // namespace utils