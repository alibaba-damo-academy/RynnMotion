#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ruckig/ruckig.hpp>

#include "trajgen.hpp"

using namespace ruckig;

void eeTrajgenTest1() {
  std::cout << "=== Test 1: EEPoseTrajGen with TCP speed constraints ===" << std::endl;

  EEPoseTrajGen eeTraj(0.002);

  Eigen::Vector3d eePos0(0.5, 0.2, 0.8);
  Eigen::Vector3d rpy0(-0.5, -0.8, 0.2);
  Eigen::Quaterniond eeQuat0 =
      Eigen::AngleAxisd(rpy0(2), Eigen::Vector3d::UnitZ()) * // Yaw
      Eigen::AngleAxisd(rpy0(1), Eigen::Vector3d::UnitY()) * // Pitch
      Eigen::AngleAxisd(rpy0(0), Eigen::Vector3d::UnitX());  // Roll
  std::cout << "Initial orientation (quat): [" << eeQuat0.w() << ", "
            << eeQuat0.x() << ", " << eeQuat0.y() << ", " << eeQuat0.z() << "]" << std::endl;
  Eigen::Vector3d eePos1(1.2, -2.3, 2.1);
  Eigen::Vector3d rpy1(0.5, 0.8, 1.5);
  Eigen::Quaterniond eeQuat1 =
      Eigen::AngleAxisd(rpy1(2), Eigen::Vector3d::UnitZ()) * // Yaw
      Eigen::AngleAxisd(rpy1(1), Eigen::Vector3d::UnitY()) * // Pitch
      Eigen::AngleAxisd(rpy1(0), Eigen::Vector3d::UnitX());  // Roll

  double maxTcpSpeed = 2.0; // m/s
  double maxTcpAcc = 5.0;   // m/s²
  eeTraj.setTcpLimits(maxTcpSpeed, maxTcpAcc);

  Eigen::Vector4d zeroVel = Eigen::Vector4d::Zero();
  Eigen::Vector4d zeroAcc = Eigen::Vector4d::Zero();

  eeTraj.setStartState(eePos0, eeQuat0, zeroVel, zeroAcc);
  eeTraj.setTargetState(eePos1, eeQuat1, zeroVel, zeroAcc);

  std::cout << "From position: [" << eePos0.transpose() << "] m" << std::endl;
  std::cout << "From orientation (quat): [" << eeQuat0.w() << ", "
            << eeQuat0.x() << ", " << eeQuat0.y() << ", " << eeQuat0.z() << "]" << std::endl;
  std::cout << "To position: [" << eePos1.transpose() << "] m" << std::endl;
  std::cout << "To orientation (quat): [" << eeQuat1.w() << ", "
            << eeQuat1.x() << ", " << eeQuat1.y() << ", " << eeQuat1.z() << "]" << std::endl;
  std::cout << "TCP speed limit: " << maxTcpSpeed << " m/s" << std::endl;
  std::cout << "TCP acceleration limit: " << maxTcpAcc << " m/s²" << std::endl;

  // Create CSV file for data export
  std::ofstream csv_file("ee_data.csv");
  csv_file << "time,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,"
           << "vel_x,vel_y,vel_z,angvel_x,angvel_y,angvel_z,"
           << "acc_x,acc_y,acc_z,angular_acc_x,angular_acc_y,angular_acc_z,"
           << "tcp_speed,tcp_acc" << std::endl;

  std::cout << "\nExecuting EE trajectory (showing every 10th step):" << std::endl;
  std::cout << "t [s] | position [m] | TCP speed [m/s]" << std::endl;
  std::cout << "------|--------------|----------------" << std::endl;

  int step = 0;
  Result result;
  double maxObservedSpeed = 0.0;
  double maxObservedAcc = 0.0;

  do {
    result = eeTraj.update();

    // Get current pose and velocities
    auto [currPos, currOrient] = eeTraj.getCurrPose();
    auto linearVel = eeTraj.getCurrVel();
    auto linearAcc = eeTraj.getCurrAcc();
    double current_time = eeTraj.getTime();

    // Calculate TCP speed and acceleration
    double tcpSpeed = eeTraj.getCurrTcpSpeed();
    double tcpAcc = eeTraj.getCurrTcpAcc();

    // Track maximum observed values
    maxObservedSpeed = std::max(maxObservedSpeed, tcpSpeed);
    maxObservedAcc = std::max(maxObservedAcc, tcpAcc);

    // Write to CSV
    csv_file << std::fixed << std::setprecision(6) << current_time;
    csv_file << "," << currPos.x() << "," << currPos.y() << "," << currPos.z();
    csv_file << "," << currOrient.w() << "," << currOrient.x() << "," << currOrient.y() << "," << currOrient.z();
    csv_file << "," << linearVel.x() << "," << linearVel.y() << "," << linearVel.z();
    csv_file << ",0.0,0.0,0.0"; // angular velocity (not tracked in simplified version)
    csv_file << "," << linearAcc.x() << "," << linearAcc.y() << "," << linearAcc.z();
    csv_file << ",0.0,0.0,0.0"; // angular acceleration (not tracked in simplified version)
    csv_file << "," << tcpSpeed << "," << tcpAcc << std::endl;

    // Print progress every 10 steps
    if (step % 10 == 0) {
      std::cout << std::fixed << std::setprecision(3)
                << std::setw(5) << current_time << " | ["
                << std::setw(6) << currPos.x() << ","
                << std::setw(6) << currPos.y() << ","
                << std::setw(6) << currPos.z() << "] | "
                << std::setw(6) << tcpSpeed << std::endl;
    }

    eeTraj.passToInput();
    step++;

  } while (result == Result::Working);

  csv_file.close();

  // Get final pose
  auto [finalPos, finalQuat] = eeTraj.getCurrPose();

  // Calculate pose errors
  Eigen::Vector3d posError = finalPos - eePos0;
  Eigen::Quaterniond orientError = finalQuat.inverse() * eeQuat1;
  double angleError = 2.0 * std::acos(std::abs(orientError.w())); // Angle difference in radians

  std::cout << "\nResult: " << static_cast<int>(result) << " (0=Working, 1=Finished, 2=Error)" << std::endl;
  std::cout << "Final position: [" << finalPos.transpose() << "] m" << std::endl;
  std::cout << "Target position: [" << eePos0.transpose() << "] m" << std::endl;
  std::cout << "Position error: [" << posError.transpose() << "] m (norm: "
            << posError.norm() << " m)" << std::endl;
  std::cout << "Final orientation (quat): [" << finalQuat.w() << ", "
            << finalQuat.x() << ", " << finalQuat.y() << ", " << finalQuat.z() << "]" << std::endl;
  std::cout << "Target orientation (quat): [" << eeQuat1.w() << ", "
            << eeQuat1.x() << ", " << eeQuat1.y() << ", " << eeQuat1.z() << "]" << std::endl;
  std::cout << "Orientation error: " << angleError * 180.0 / M_PI << " degrees" << std::endl;
  std::cout << "Duration: " << eeTraj.getDuration() << " s" << std::endl;
  std::cout << "Max observed TCP speed: " << maxObservedSpeed << " m/s (limit: " << maxTcpSpeed << " m/s)" << std::endl;
  std::cout << "Max observed TCP acceleration: " << maxObservedAcc << " m/s² (limit: " << maxTcpAcc << " m/s²)" << std::endl;
  std::cout << "Total steps: " << step << std::endl;
  std::cout << "Data exported to: ee_data.csv" << std::endl;
  std::cout << std::endl;
}

void eeTrajgenTest2() {
  std::cout << "=== Test 2: EEPoseTrajGen with zero boundary conditions ===" << std::endl;

  EEPoseTrajGen eeTraj(0.002);

  // Start pose
  Eigen::Vector3d eePos0(0.2, 0.1, 0.5);
  Eigen::Vector3d rpy0(0.1, -0.2, 0.3);
  Eigen::Quaterniond eeQuat0 =
      Eigen::AngleAxisd(rpy0(2), Eigen::Vector3d::UnitZ()) * // Yaw
      Eigen::AngleAxisd(rpy0(1), Eigen::Vector3d::UnitY()) * // Pitch
      Eigen::AngleAxisd(rpy0(0), Eigen::Vector3d::UnitX());  // Roll

  // Target pose
  Eigen::Vector3d eePos1(1.5, -1.8, 1.2);
  Eigen::Vector3d rpy1(-0.3, 0.4, -0.8);
  Eigen::Quaterniond eeQuat1 =
      Eigen::AngleAxisd(rpy1(2), Eigen::Vector3d::UnitZ()) * // Yaw
      Eigen::AngleAxisd(rpy1(1), Eigen::Vector3d::UnitY()) * // Pitch
      Eigen::AngleAxisd(rpy1(0), Eigen::Vector3d::UnitX());  // Roll

  double maxTcpSpeed = 1.5; // m/s
  double maxTcpAcc = 3.0;   // m/s²
  eeTraj.setTcpLimits(maxTcpSpeed, maxTcpAcc);

  // Use simplified interface with zero boundary conditions
  eeTraj.setStartState(eePos0, eeQuat0);
  eeTraj.setTargetState(eePos1, eeQuat1);

  std::cout << "From position: [" << eePos0.transpose() << "] m" << std::endl;
  std::cout << "From orientation (RPY): [" << rpy0.transpose() << "] rad" << std::endl;
  std::cout << "To position: [" << eePos1.transpose() << "] m" << std::endl;
  std::cout << "To orientation (RPY): [" << rpy1.transpose() << "] rad" << std::endl;
  std::cout << "TCP speed limit: " << maxTcpSpeed << " m/s" << std::endl;
  std::cout << "TCP acceleration limit: " << maxTcpAcc << " m/s²" << std::endl;

  // Create CSV file for data export
  std::ofstream csv_file("ee2_data.csv");
  csv_file << "time,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,"
           << "vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,tcp_speed,tcp_acc" << std::endl;

  std::cout << "\nExecuting EE trajectory (showing every 15th step):" << std::endl;
  std::cout << "t [s] | position [m] | TCP speed [m/s]" << std::endl;
  std::cout << "------|--------------|----------------" << std::endl;

  int step = 0;
  Result result;
  double maxObservedSpeed = 0.0;
  double maxObservedAcc = 0.0;

  do {
    result = eeTraj.update();

    // Get current pose and velocities
    auto [currPos, currOrient] = eeTraj.getCurrPose();
    auto linearVel = eeTraj.getCurrVel();
    auto linearAcc = eeTraj.getCurrAcc();
    double current_time = eeTraj.getTime();

    // Calculate TCP speed and acceleration
    double tcpSpeed = eeTraj.getCurrTcpSpeed();
    double tcpAcc = eeTraj.getCurrTcpAcc();

    // Track maximum observed values
    maxObservedSpeed = std::max(maxObservedSpeed, tcpSpeed);
    maxObservedAcc = std::max(maxObservedAcc, tcpAcc);

    // Write to CSV
    csv_file << std::fixed << std::setprecision(6) << current_time;
    csv_file << "," << currPos.x() << "," << currPos.y() << "," << currPos.z();
    csv_file << "," << currOrient.w() << "," << currOrient.x() << "," << currOrient.y() << "," << currOrient.z();
    csv_file << "," << linearVel.x() << "," << linearVel.y() << "," << linearVel.z();
    csv_file << "," << linearAcc.x() << "," << linearAcc.y() << "," << linearAcc.z();
    csv_file << "," << tcpSpeed << "," << tcpAcc << std::endl;

    // Print progress every 15 steps
    if (step % 15 == 0) {
      std::cout << std::fixed << std::setprecision(3)
                << std::setw(5) << current_time << " | ["
                << std::setw(6) << currPos.x() << ","
                << std::setw(6) << currPos.y() << ","
                << std::setw(6) << currPos.z() << "] | "
                << std::setw(6) << tcpSpeed << std::endl;
    }

    eeTraj.passToInput();
    step++;

  } while (result == Result::Working);

  csv_file.close();

  // Get final pose
  auto [finalPos, finalQuat] = eeTraj.getCurrPose();

  // Calculate final RPY using Eigen's toRotationMatrix and eulerAngles
  Eigen::Vector3d finalRpy = finalQuat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order

  // Calculate pose errors
  Eigen::Vector3d posError = finalPos - eePos1;
  Eigen::Vector3d rpyError = finalRpy - rpy1;

  std::cout << "\nResult: " << static_cast<int>(result) << " (0=Working, 1=Finished, 2=Error)" << std::endl;
  std::cout << "Final position: [" << finalPos.transpose() << "] m" << std::endl;
  std::cout << "Target position: [" << eePos1.transpose() << "] m" << std::endl;
  std::cout << "Position error: [" << posError.transpose() << "] m (norm: "
            << posError.norm() << " m)" << std::endl;
  std::cout << "Final orientation (RPY): [" << finalRpy.transpose() << "] rad" << std::endl;
  std::cout << "Target orientation (RPY): [" << rpy1.transpose() << "] rad" << std::endl;
  std::cout << "Orientation error (RPY): [" << rpyError.transpose() << "] rad" << std::endl;
  std::cout << "Duration: " << eeTraj.getDuration() << " s" << std::endl;
  std::cout << "Max observed TCP speed: " << maxObservedSpeed << " m/s (limit: " << maxTcpSpeed << " m/s)" << std::endl;
  std::cout << "Max observed TCP acceleration: " << maxObservedAcc << " m/s² (limit: " << maxTcpAcc << " m/s²)" << std::endl;
  std::cout << "Total steps: " << step << std::endl;
  std::cout << "Data exported to: ee2_data.csv" << std::endl;
  std::cout << std::endl;
}

int main() {
  std::cout << "=== EEPoseTrajGen Testing ===" << std::endl;
  std::cout << "Testing end-effector pose trajectory generation with TCP constraints\n"
            << std::endl;

  eeTrajgenTest1();

  eeTrajgenTest2();

  std::cout << "=== All EE trajectory tests completed ===" << std::endl;
  std::cout << "\nTest results exported as CSV files:" << std::endl;
  std::cout << "- ee_data.csv" << std::endl;
  std::cout << "- ee2_data.csv" << std::endl;

  return 0;
}
