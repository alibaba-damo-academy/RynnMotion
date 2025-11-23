#include "signal_generator.hpp"

#include <algorithm>

namespace utils {

Eigen::VectorXd CubicTrajectory::getOutput(double t) const {
  double time = std::clamp(t, 0.0, duration_);

  double p0 = 0.0, p1 = amp_;
  double v0 = 0.0, v1 = 0.0;

  double T = duration_;
  double T2 = T * T;
  double T3 = T2 * T;

  double c0 = p0;
  double c1 = v0;
  double c2 = (3.0 * (p1 - p0) - (2.0 * v0 + v1) * T) / T2;
  double c3 = (2.0 * (p0 - p1) + (v0 + v1) * T) / T3;

  double tau = time;
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;

  Eigen::VectorXd out(3);
  out(0) = c0 + c1 * tau + c2 * tau2 + c3 * tau3;
  out(1) = c1 + 2.0 * c2 * tau + 3.0 * c3 * tau2;
  out(2) = (t > duration_) ? 0.0 : (2.0 * c2 + 6.0 * c3 * tau);

  return out;
}

Eigen::VectorXd LerpTrajectory::getOutput(double t) const {
  double alpha = (duration_ <= 0.0) ? 1.0 : t / duration_;
  alpha = std::clamp(alpha, 0.0, 1.0);

  Eigen::VectorXd out(3);
  out(0) = amp_ * alpha;
  out(1) = (t >= 0.0 && t <= duration_ && duration_ > 0.0) ? amp_ / duration_ : 0.0;
  out(2) = 0.0;

  return out;
}

Eigen::VectorXd SineSignal::getOutput(double t) const {
  const double omega = 2.0 * M_PI * freq_;

  Eigen::VectorXd out(3);
  out(0) = amp_ * std::sin(omega * t);
  out(1) = amp_ * omega * std::cos(omega * t);
  out(2) = -amp_ * omega * omega * std::sin(omega * t);

  return out;
}

Eigen::VectorXd CosineSignal::getOutput(double t) const {
  const double omega = 2.0 * M_PI * freq_;

  Eigen::VectorXd out(3);
  out(0) = amp_ * std::cos(omega * t);
  out(1) = -amp_ * omega * std::sin(omega * t);
  out(2) = -amp_ * omega * omega * std::cos(omega * t);

  return out;
}

Eigen::VectorXd ChirpSignal::getOutput(double t) const {
  double time = std::min(t, duration_);

  double halfDuration = duration_ * 0.5;
  double k = (freqEnd_ - freqStart_) / halfDuration;

  double phi = 0.0;
  double phiDot = 0.0;
  double phiDdot = 0.0;

  if (time <= halfDuration) {
    phi = freqStart_ * time + 0.5 * k * time * time;
    phiDot = freqStart_ + k * time;
    phiDdot = k;
  } else {
    phi = (freqEnd_ + k * halfDuration) * time - 0.5 * k * time * time +
          (freqStart_ - freqEnd_) * halfDuration;
    phiDot = freqEnd_ + (-k) * (time - halfDuration);
    phiDdot = -k;
  }

  const double twoPi = 2.0 * M_PI;

  Eigen::VectorXd out(3);
  out(0) = amp_ * std::sin(twoPi * phi);
  out(1) = amp_ * std::cos(twoPi * phi) * (twoPi * phiDot);
  out(2) = amp_ * (-std::sin(twoPi * phi) * std::pow(twoPi * phiDot, 2) +
                    std::cos(twoPi * phi) * (twoPi * phiDdot));

  return out;
}

Eigen::VectorXd SquareSignal::getOutput(double t) const {
  double s = std::sin(2.0 * M_PI * freq_ * t);

  Eigen::VectorXd out(3);
  out(0) = amp_ * ((s >= 0.0) ? 1.0 : -1.0);
  out(1) = 0.0;
  out(2) = 0.0;

  return out;
}

Eigen::VectorXd TriangleSignal::getOutput(double t) const {
  double time = std::min(t, duration_);

  double period = 1.0 / freq_;
  double phase = std::fmod(time, period) / period;

  double timeRatio = time / duration_;
  double fadeOut = 1.0;
  double fadeOutStart = 0.95;

  if (timeRatio > fadeOutStart) {
    fadeOut = 1.0 - (timeRatio - fadeOutStart) / (1.0 - fadeOutStart);
  }

  double q = 0.0;
  double qd = 0.0;

  if (phase < 0.5) {
    q = amp_ * (2.0 * phase);
    qd = amp_ * 2.0 * freq_;
  } else {
    q = amp_ * (2.0 - 2.0 * phase);
    qd = -amp_ * 2.0 * freq_;
  }

  q *= fadeOut;
  qd *= fadeOut;

  if (timeRatio > fadeOutStart) {
    double fadeOutDerivative = -amp_ / (duration_ * (1.0 - fadeOutStart));
    qd += q * fadeOutDerivative;
  }

  Eigen::VectorXd out(3);
  out(0) = q;
  out(1) = qd;
  out(2) = 0.0;

  return out;
}

} // namespace utils
