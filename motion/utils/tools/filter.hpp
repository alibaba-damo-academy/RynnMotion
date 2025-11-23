#pragma once
#include <Eigen/Dense>
#include <cmath>

#include "math_tools.hpp"


namespace utils {

/**
 * @brief second order low pass filter
 * @param cutOffFreqInHz, cut off frequency (in hz, not rad/s)
 * @param dTime, sampling time
 * @param zeta, damping ratio, usualy set over 0.707 to avoid oscillation
 */
class SecondLowPass {
public:
  SecondLowPass(double cutOffFreqInHz, double dTime, double zeta);
  SecondLowPass(double cutOffFreqInHz, double dTime, double zeta, int vecSize);
  double filtering(double u);
  Eigen::VectorXd filtering(Eigen::VectorXd sigIn);

  void reset(double initSignal);
  void reset(Eigen::VectorXd sigIn);

private:
  double fc, dt, zeta;
  double den;
  double ulast, ullast, y, ylast, yllast;
  double A1, A2, A3, B2, B3;
  Eigen::VectorXd sigIn1, sigIn2, sigOut, sigOut1, sigOut2;
};

/**
 * @brief first order low pass filter
 * @param cutOffFreqInHz, cut off frequency (in hz, not rad/s)
 * @param dTime, sampling time
 */
class FirstLowPass {
public:
  FirstLowPass(double cutOffFreqInHz, double dTime);
  FirstLowPass(double cutOffFreqInHz, double dTime, int vecSize);
  double filtering(double u);
  Eigen::VectorXd filtering(Eigen::VectorXd sigIn);

  void reset(double initSignal);
  void reset(Eigen::VectorXd sigIn);

private:
  double fc, dt;
  double den;
  double ulast, y, ylast;
  double A1, A2, B2;
  Eigen::VectorXd sigIn1, sigOut, sigOut1;
};

/**
 * @brief differential of a signal, either in number or eigen vector
 * @param dTime, sampling time
 */
class Diff {
public:
  Diff(double dTime);
  Diff(double dTime, int vecSize);
  ~Diff();
  void reset();
  double run(double u);
  Eigen::VectorXd run(Eigen::VectorXd sigIn);

private:
  bool initFlag{true};
  double dt;
  double uLast;
  Eigen::VectorXd sigInLast, sigOut;
};

} // namespace utils