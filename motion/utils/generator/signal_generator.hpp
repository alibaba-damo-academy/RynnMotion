#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <memory>

namespace utils {

/**
 * @brief Base class for all signal generators
 *
 * All signals return a 3D vector [position, velocity, acceleration]
 * for use in robot trajectory generation and dynamics.
 */
class SignalGenerator {
public:
  virtual ~SignalGenerator() = default;

  /**
   * @brief Get signal output at given time
   * @param t Time in seconds
   * @return 3D vector [position, velocity, acceleration]
   */
  virtual Eigen::VectorXd getOutput(double t) const = 0;
};

// ===== Trajectory Generators (Finite-time motions) =====

/**
 * @brief Cubic polynomial trajectory from 0 to amplitude with zero boundary velocities
 *
 * Smoothly interpolates from 0 to amp over duration with zero start/end velocities.
 * Ideal for point-to-point motions with smooth acceleration profiles.
 */
class CubicTrajectory : public SignalGenerator {
public:
  CubicTrajectory(double duration, double amp)
      : duration_(duration), amp_(amp) {}

  Eigen::VectorXd getOutput(double t) const override;

private:
  double duration_;
  double amp_;
};

/**
 * @brief Linear interpolation trajectory from 0 to amplitude
 *
 * Simple ramp from 0 to amp over duration with constant velocity.
 * Zero acceleration throughout.
 */
class LerpTrajectory : public SignalGenerator {
public:
  LerpTrajectory(double duration, double amp)
      : duration_(duration), amp_(amp) {}

  Eigen::VectorXd getOutput(double t) const override;

private:
  double duration_;
  double amp_;
};

// ===== Periodic Signals (Continuous oscillations) =====

/**
 * @brief Sinusoidal signal for continuous oscillation
 *
 * Generates smooth periodic motion: amp * sin(2π * freq * t)
 * Returns analytical derivatives for velocity and acceleration.
 */
class SineSignal : public SignalGenerator {
public:
  SineSignal(double amp, double freq)
      : amp_(amp), freq_(freq) {}

  Eigen::VectorXd getOutput(double t) const override;

private:
  double amp_;
  double freq_;
};

/**
 * @brief Cosine signal for continuous oscillation
 *
 * Generates smooth periodic motion: amp * cos(2π * freq * t)
 * Phase-shifted version of sine signal.
 */
class CosineSignal : public SignalGenerator {
public:
  CosineSignal(double amp, double freq)
      : amp_(amp), freq_(freq) {}

  Eigen::VectorXd getOutput(double t) const override;

private:
  double amp_;
  double freq_;
};

/**
 * @brief Frequency-swept chirp signal
 *
 * Sweeps from freqStart to freqEnd over duration.
 * Useful for system identification and frequency response testing.
 */
class ChirpSignal : public SignalGenerator {
public:
  ChirpSignal(double amp, double duration, double freqEnd, double freqStart = 0.1)
      : amp_(amp), duration_(duration),
        freqStart_(freqStart), freqEnd_(freqEnd) {}

  Eigen::VectorXd getOutput(double t) const override;

private:
  double amp_;
  double duration_;
  double freqStart_;
  double freqEnd_;
};

/**
 * @brief Square wave signal
 *
 * Alternates between +amp and -amp at given frequency.
 * Derivatives are zero (discontinuous signal).
 */
class SquareSignal : public SignalGenerator {
public:
  SquareSignal(double amp, double freq)
      : amp_(amp), freq_(freq) {}

  Eigen::VectorXd getOutput(double t) const override;

private:
  double amp_;
  double freq_;
};

/**
 * @brief Triangle wave signal with optional fade-out
 *
 * Rises and falls linearly between 0 and amp.
 * Includes fade-out effect near end of duration.
 */
class TriangleSignal : public SignalGenerator {
public:
  TriangleSignal(double amp, double freq, double duration)
      : amp_(amp), freq_(freq), duration_(duration) {}

  Eigen::VectorXd getOutput(double t) const override;

private:
  double amp_;
  double freq_;
  double duration_;
};

} // namespace utils
