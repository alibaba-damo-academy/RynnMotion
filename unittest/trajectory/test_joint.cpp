#include <Eigen/Core>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ruckig/ruckig.hpp>

#include "trajgen.hpp"

using namespace ruckig;

void jointTrajgenTest1() {
  std::cout << "=== Test 1: JointTrajGen with NO constraints ===" << std::endl;

  const int ndof = 2;
  JointTrajGen<ndof> JointTraj(0.05);

  Eigen::VectorXd qPos0 = Eigen::VectorXd::Zero(ndof);
  Eigen::VectorXd qPos1 = Eigen::VectorXd::Zero(ndof);
  Eigen::VectorXd qUpperLimits = Eigen::VectorXd::Zero(ndof);
  Eigen::VectorXd qLowerLimits = Eigen::VectorXd::Zero(ndof);
  qPos0 << 0.3, -0.6;
  qPos1 << 2.0, -4.5;
  qUpperLimits.setConstant(3.0);
  qLowerLimits.setConstant(-3.0);

  Eigen::VectorXd qdMax(ndof), qddMax(ndof), qdddMax(ndof);
  qdMax << 1.0, 1.0;
  qddMax << 2.0, 1.0;
  qdddMax << 10.0, 5.0;

  JointTraj.setStartState(qPos0);
  JointTraj.setTargetState(qPos1);
  JointTraj.setJointPosLimits(qUpperLimits, qLowerLimits);
  JointTraj.setJointMotionLimits(qdMax, qddMax);

  std::cout << "From: [" << qPos0.transpose() << "] rad" << std::endl;
  std::cout << "To:   [" << qPos1.transpose() << "] rad" << std::endl;
  std::cout << "Motion limits - Vel: [" << qdMax.transpose() << "] rad/s" << std::endl;

  std::ofstream csv_file("joint1_data.csv");
  csv_file << "time";
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",joint" << (i + 1);
  }
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",vel" << (i + 1);
  }
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",acc" << (i + 1);
  }
  csv_file << std::endl;

  std::cout << "\nExecuting trajectory (showing every 10th step):" << std::endl;
  std::cout << "t [s] | joint positions [rad] | joint velocities [rad/s]" << std::endl;
  std::cout << "------|----------------------|------------------------" << std::endl;

  int step = 0;
  Result result;
  do {
    result = JointTraj.update();

    // Print duration after first update
    if (step == 0) {
      double duration = JointTraj.getDuration();
      std::cout << "Trajectory duration: " << duration << "s" << std::endl;
    }

    auto qRef = JointTraj.getCurrPos();
    auto qdRef = JointTraj.getCurrVel();
    auto qddRef = JointTraj.getCurrAcc();
    double current_time = JointTraj.getTime();

    csv_file << std::fixed << std::setprecision(6) << current_time;
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qRef(i);
    }
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qdRef(i);
    }
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qddRef(i);
    }
    csv_file << std::endl;

    if (step % 10 == 0) {
      std::cout << std::fixed << std::setprecision(3)
                << std::setw(5) << current_time << " | [";
      for (int i = 0; i < ndof; ++i) {
        std::cout << std::setw(6) << qRef(i) << (i < ndof - 1 ? "," : "");
      }
      std::cout << "] | [";
      for (int i = 0; i < ndof; ++i) {
        std::cout << std::setw(6) << qdRef(i) << (i < ndof - 1 ? "," : "");
      }
      std::cout << "]" << std::endl;
    }

    JointTraj.passToInput();
    step++;

  } while (result == Result::Working);

  csv_file.close();

  auto final_joints = JointTraj.getCurrPos();
  std::cout << "\nResult: " << static_cast<int>(result) << " (0=Working, 1=Finished, 2=Error)" << std::endl;
  std::cout << "Final joint positions: [" << final_joints.transpose() << "] rad" << std::endl;
  std::cout << "Total steps: " << step << std::endl;
  std::cout << "Data exported to: joint1_data.csv" << std::endl;
  std::cout << std::endl;
}

void jointTrajgenTest2() {
  std::cout << "=== Test 2: JointTrajGen with POSITION limits only ===" << std::endl;

  const int ndof = 6;
  JointTrajGen<ndof> JointTraj(0.001);

  Eigen::VectorXd qPos0 = Eigen::VectorXd::Zero(ndof);
  Eigen::VectorXd qPos1 = Eigen::VectorXd::Zero(ndof);
  Eigen::VectorXd qUpperLimits = Eigen::VectorXd::Zero(ndof);
  Eigen::VectorXd qLowerLimits = Eigen::VectorXd::Zero(ndof);
  qPos0 << 0.0, 0.0, 0.0, 1.0, -1.0, 2.0;
  qPos1 << 2.0, -2.5, 0.8, 4.0, -3.0, 6.0;

  qUpperLimits.setConstant(1.0);
  qLowerLimits.setConstant(-6.0);

  JointTraj.setStartState(qPos0);
  JointTraj.setTargetState(qPos1);
  JointTraj.setJointPosLimits(qUpperLimits, qLowerLimits);

  Eigen::VectorXd qdMax(ndof), qddMax(ndof), qdddMax(ndof);
  qdMax << 6.0, 4.0, 2.0, 8.0, 6.0, 10.0;       // rad/s
  qddMax << 3.0, 2.0, 1.0, 4.0, 3.0, 5.0;       // rad/s^2
  qdddMax << 15.0, 10.0, 5.0, 20.0, 15.0, 25.0; // rad/s^3
  // Test with velocity constraints only (should work with our fix)
  // JointTraj.setJointMotionLimits(qdMax);
  JointTraj.setJointMotionLimits(qdMax, qddMax);
  // JointTraj.setJointMotionLimits(qdMax, qddMax, qdddMax);

  std::cout << "From: [" << qPos0.transpose() << "] rad" << std::endl;
  std::cout << "To:   [" << qPos1.transpose() << "] rad (will be clamped)" << std::endl;
  std::cout << "Upper limits: [" << qUpperLimits.transpose() << "] rad" << std::endl;
  std::cout << "Lower limits: [" << qLowerLimits.transpose() << "] rad" << std::endl;

  std::ofstream csv_file("joint2_data.csv");
  csv_file << "time";
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",joint" << (i + 1);
  }
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",vel" << (i + 1);
  }
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",acc" << (i + 1);
  }
  csv_file << std::endl;

  std::cout << "\nExecuting trajectory (showing every 10th step):" << std::endl;
  std::cout << "t [s] | joint positions [rad] | joint velocities [rad/s]" << std::endl;
  std::cout << "------|----------------------|------------------------" << std::endl;

  int step = 0;
  Result result;
  do {
    result = JointTraj.update();

    std::cout << "Update result: " << static_cast<int>(result) << " (0=Working, 1=Finished, 2=Error)" << std::endl;

    auto qRef = JointTraj.getCurrPos();
    auto qdRef = JointTraj.getCurrVel();
    auto qddRef = JointTraj.getCurrAcc();
    double current_time = JointTraj.getTime();

    std::cout << "Time: " << current_time << "s" << std::endl;

    csv_file << std::fixed << std::setprecision(6) << current_time;
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qRef(i);
    }
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qdRef(i);
    }
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qddRef(i);
    }
    csv_file << std::endl;

    if (step % 10 == 0) {
      std::cout << std::fixed << std::setprecision(3)
                << std::setw(5) << current_time << " | [";
      for (int i = 0; i < ndof; ++i) {
        std::cout << std::setw(6) << qRef(i) << (i < ndof - 1 ? "," : "");
      }
      std::cout << "] | [";
      for (int i = 0; i < ndof; ++i) {
        std::cout << std::setw(6) << qdRef(i) << (i < ndof - 1 ? "," : "");
      }
      std::cout << "]" << std::endl;
    }

    JointTraj.passToInput();
    step++;

  } while (result == Result::Working);

  csv_file.close();

  auto final_joints = JointTraj.getCurrPos();
  std::cout << "\nFinal joint positions: [" << final_joints.transpose() << "] rad" << std::endl;
  std::cout << "Expected clamped to: ["
            << std::min(qPos1(0), qUpperLimits(0)) << ", "
            << std::max(qPos1(1), qLowerLimits(1)) << ", "
            << std::min(qPos1(2), qUpperLimits(2)) << "] rad" << std::endl;
  std::cout << "Total steps: " << step << std::endl;
  std::cout << "Data exported to: joint2_data.csv" << std::endl;
  std::cout << std::endl;
}

void jointTrajgenTest3() {
  std::cout << "=== Test 3: JointTrajGen with NON-ZERO boundary conditions ===" << std::endl;

  const int ndof = 3;
  JointTrajGen<ndof> JointTraj(0.001);

  Eigen::VectorXd qPos0(ndof), qVel0(ndof), qAcc0(ndof);
  Eigen::VectorXd qPos1(ndof), qVel1(ndof), qAcc1(ndof);

  qPos0 << 0.2, -0.3, 0.5;
  qVel0 << 0.1, -0.5, 0.2;
  qAcc0 << 0.05, 0.1, -0.1;

  qPos1 << 2.0, -4.5, 6.8;
  qVel1 << -0.2, 0.3, -0.1;
  qAcc1 << 0.1, -0.2, 0.15;

  Eigen::VectorXd qdMax(ndof), qddMax(ndof);
  qdMax.setConstant(2 * M_PI);
  qddMax.setConstant(M_PI);

  JointTraj.setStartState(qPos0, qVel0, qAcc0);
  JointTraj.setTargetState(qPos1, qVel1, qAcc1);
  JointTraj.setJointMotionLimits(qdMax, qddMax);

  std::cout << "From: pos=[" << qPos0.transpose() << "] rad" << std::endl;
  std::cout << "      vel=[" << qVel0.transpose() << "] rad/s" << std::endl;
  std::cout << "      acc=[" << qAcc0.transpose() << "] rad/s²" << std::endl;
  std::cout << "To:   pos=[" << qPos1.transpose() << "] rad" << std::endl;
  std::cout << "      vel=[" << qVel1.transpose() << "] rad/s" << std::endl;
  std::cout << "      acc=[" << qAcc1.transpose() << "] rad/s²" << std::endl;
  std::cout << "Duration: " << JointTraj.getDuration() << "s" << std::endl;

  std::ofstream csv_file("joint3_data.csv");
  csv_file << "time";
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",joint" << (i + 1);
  }
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",vel" << (i + 1);
  }
  for (int i = 0; i < ndof; ++i) {
    csv_file << ",acc" << (i + 1);
  }
  csv_file << std::endl;

  std::cout << "\nExecuting trajectory (showing every 15th step):" << std::endl;
  std::cout << "t [s] | joint positions [rad] | joint velocities [rad/s]" << std::endl;
  std::cout << "------|----------------------|------------------------" << std::endl;

  int step = 0;
  Result result;
  Eigen::VectorXd max_observed_vel = Eigen::VectorXd::Zero(ndof);

  do {
    result = JointTraj.update();

    auto qRef = JointTraj.getCurrPos();
    auto qdRef = JointTraj.getCurrVel();
    auto qddRef = JointTraj.getCurrAcc();
    double current_time = JointTraj.getTime();

    // Track maximum observed velocities
    for (int i = 0; i < ndof; ++i) {
      max_observed_vel(i) = std::max(max_observed_vel(i), std::abs(qdRef(i)));
    }

    csv_file << std::fixed << std::setprecision(6) << current_time;
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qRef(i);
    }
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qdRef(i);
    }
    for (int i = 0; i < ndof; ++i) {
      csv_file << "," << qddRef(i);
    }
    csv_file << std::endl;

    if (step % 15 == 0) {
      std::cout << std::fixed << std::setprecision(3)
                << std::setw(5) << current_time << " | [";
      for (int i = 0; i < ndof; ++i) {
        std::cout << std::setw(6) << qRef(i) << (i < ndof - 1 ? "," : "");
      }
      std::cout << "] | [";
      for (int i = 0; i < ndof; ++i) {
        std::cout << std::setw(6) << qdRef(i) << (i < ndof - 1 ? "," : "");
      }
      std::cout << "]" << std::endl;
    }

    JointTraj.passToInput();
    step++;

  } while (result == Result::Working);

  csv_file.close();

  auto final_joints = JointTraj.getCurrPos();
  auto final_vels = JointTraj.getCurrVel();
  auto final_accs = JointTraj.getCurrAcc();

  std::cout << "\nResult: " << static_cast<int>(result) << " (0=Working, 1=Finished, 2=Error)" << std::endl;
  std::cout << "Final pos: [" << final_joints.transpose() << "] rad" << std::endl;
  std::cout << "Final vel: [" << final_vels.transpose() << "] rad/s" << std::endl;
  std::cout << "Final acc: [" << final_accs.transpose() << "] rad/s²" << std::endl;
  std::cout << "Target pos:[" << qPos1.transpose() << "] rad" << std::endl;
  std::cout << "Target vel:[" << qVel1.transpose() << "] rad/s" << std::endl;
  std::cout << "Target acc:[" << qAcc1.transpose() << "] rad/s²" << std::endl;
  std::cout << "Max observed velocities: [" << max_observed_vel.transpose() << "] rad/s" << std::endl;
  std::cout << "Velocity limits were: [" << qdMax.transpose() << "] rad/s" << std::endl;
  std::cout << "Total steps: " << step << std::endl;
  std::cout << "Data exported to: joint3_data.csv" << std::endl;
  std::cout << std::endl;
}

int main() {
  std::cout << "=== JointTrajGen Comprehensive Testing (Dynamic Version) ===" << std::endl;
  std::cout << "Testing different constraint scenarios for joint trajectory generation\n"
            << std::endl;

  jointTrajgenTest1();

  jointTrajgenTest2();

  jointTrajgenTest3();

  std::cout << "=== All JointTrajGen tests completed ===" << std::endl;
  std::cout << "\nTest results exported as CSV files:" << std::endl;
  std::cout << "- joint1_data.csv" << std::endl;
  std::cout << "- joint2_data.csv" << std::endl;
  std::cout << "- joint3_data.csv" << std::endl;

  return 0;
}
