#include "filter.hpp"
namespace utils {

SecondLowPass ::SecondLowPass(double cutOffFreqInHz, double dTime, double zeta) {
  this->fc = 2. * M_PI * cutOffFreqInHz;
  this->dt = dTime;
  this->zeta = zeta;
  den = fc * fc * dt * dt + 4 * zeta * fc * dt + 4;
  A1 = fc * fc * dt * dt / den;
  A2 = fc * fc * dt * dt * 2 / den;
  A3 = fc * fc * dt * dt / den;
  B2 = -(2 * fc * fc * dt * dt - 8) / den;
  B3 = -(fc * fc * dt * dt - 4 * zeta * fc * dt + 4) / den;
  ulast = 0.;
  ullast = 0.;
  ylast = 0.;
  yllast = 0.;
}

SecondLowPass ::SecondLowPass(double cutOffFreqInHz, double dTime, double zeta, int vecSize) {
  this->fc = 2. * M_PI * cutOffFreqInHz;
  this->dt = dTime;
  this->zeta = zeta;
  den = fc * fc * dt * dt + 4 * zeta * fc * dt + 4;
  A1 = fc * fc * dt * dt / den;
  A2 = fc * fc * dt * dt * 2 / den;
  A3 = fc * fc * dt * dt / den;
  B2 = -(2 * fc * fc * dt * dt - 8) / den;
  B3 = -(fc * fc * dt * dt - 4 * zeta * fc * dt + 4) / den;
  sigIn1 = Eigen::VectorXd::Zero(vecSize);
  sigIn2 = Eigen::VectorXd::Zero(vecSize);
  sigOut = Eigen::VectorXd::Zero(vecSize);
  sigOut1 = Eigen::VectorXd::Zero(vecSize);
  sigOut2 = Eigen::VectorXd::Zero(vecSize);
}

void SecondLowPass ::reset(double initSignal) {
  ulast = initSignal;
  ullast = initSignal;
  y = 0.;
  ylast = 0.;
  yllast = 0.;
}

void SecondLowPass ::reset(Eigen::VectorXd sigIn) {
  sigIn1 = sigIn;
  sigIn2 = sigIn;
  sigOut.setZero(sigIn.size());
  sigOut1.setZero(sigIn.size());
  sigOut2.setZero(sigIn.size());
}

double SecondLowPass ::filtering(double u) {
  y = B2 * ylast + B3 * yllast + A1 * u + A2 * ulast + A3 * ullast;
  ullast = ulast;
  ulast = u;
  yllast = ylast;
  ylast = y;
  return y;
}

Eigen::VectorXd SecondLowPass ::filtering(Eigen::VectorXd sigIn) {
  sigOut = B2 * sigOut1 + B3 * sigOut2 + A1 * sigIn + A2 * sigIn1 + A3 * sigIn2;
  sigIn2 = sigIn1;
  sigIn1 = sigIn;
  sigOut2 = sigOut1;
  sigOut1 = sigOut;
  return sigOut;
}

FirstLowPass ::FirstLowPass(double cutOffFreqInHz, double dTime) {
  this->fc = 2. * M_PI * cutOffFreqInHz;
  this->dt = dTime;
  den = fc * dt + 2;
  A1 = fc * dt / den;
  A2 = fc * dt / den;
  B2 = -(fc * dt - 2) / den;
  ulast = 0.;
  ylast = 0.;
}

FirstLowPass ::FirstLowPass(double cutOffFreqInHz, double dTime, int vecSize) {
  this->fc = 2. * M_PI * cutOffFreqInHz;
  this->dt = dTime;
  den = fc * dt + 2;
  A1 = fc * dt / den;
  A2 = fc * dt / den;
  B2 = -(fc * dt - 2) / den;
  sigIn1 = Eigen::VectorXd::Zero(vecSize);
  sigOut = Eigen::VectorXd::Zero(vecSize);
  sigOut1 = Eigen::VectorXd::Zero(vecSize);
}

double FirstLowPass ::filtering(double u) {
  y = B2 * ylast + A1 * u + A2 * ulast;
  ulast = u;
  ylast = y;
  return y;
}

Eigen::VectorXd FirstLowPass ::filtering(Eigen::VectorXd sigIn) {
  sigOut = B2 * sigOut1 + A1 * sigIn + A2 * sigIn1;
  sigIn1 = sigIn;
  sigOut1 = sigOut;
  return sigOut;
}

void FirstLowPass ::reset(double initSignal) {
  ulast = initSignal;
  y = 0.;
  ylast = 0.;
}

void FirstLowPass ::reset(Eigen::VectorXd sigIn) {
  sigIn1 = sigIn;
  sigOut.setZero(sigIn.size());
  sigOut1.setZero(sigIn.size());
}

Diff ::Diff(double dTime) {
  this->dt = dTime;
}

Diff ::Diff(double dTime, int vecSize) {
  this->dt = dTime;
  sigInLast = Eigen::VectorXd::Zero(vecSize);
  sigOut = Eigen::VectorXd::Zero(vecSize);
}

void Diff ::reset() {
  initFlag = true;
  sigInLast.setZero();
  sigOut.setZero();
  uLast = 0.;
}

Eigen::VectorXd Diff ::run(Eigen::VectorXd sigIn) {
  if (initFlag) {
    initFlag = false;
    sigOut.setZero();
  } else {
    sigOut = (sigIn - sigInLast) / dt;
  }
  sigInLast = sigIn;
  return sigOut;
}

double Diff ::run(double u) {
  double y;
  if (initFlag) {
    initFlag = false;
    y = 0.;
  } else {
    y = (u - uLast) / dt;
  }
  uLast = u;
  return y;
}

Diff ::~Diff() {
  sigInLast.setZero();
}

} // namespace utils