#!/usr/bin/env python3
"""
Joint trajectory generation test with multiple scenarios.
Tests different constraint scenarios for joint trajectory generation.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import math
import sys
import os

# Add the python directory to the path to import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from utils.trajectory.trajgen import JointTrajGen


def joint_trajgen_test1():
    """Test 1: JointTrajGen with NO constraints"""
    print("=== Test 1: JointTrajGen with NO constraints ===")
    
    ndof = 2
    joint_traj = JointTrajGen(ndof, 0.05)
    
    q_pos0 = np.array([0.3, -0.6])
    q_pos1 = np.array([2.0, -4.5])
    q_upper_limits = np.ones(ndof) * 3.0
    q_lower_limits = np.ones(ndof) * -3.0
    
    qd_max = np.array([1.0, 1.0])
    qdd_max = np.array([2.0, 1.0])
    qddd_max = np.array([10.0, 5.0])
    
    joint_traj.set_start_state(q_pos0)
    joint_traj.set_target_state(q_pos1)
    joint_traj.set_joint_pos_limits(q_upper_limits, q_lower_limits)
    joint_traj.set_joint_motion_limits(qd_max, qdd_max)
    
    print(f"From: [{', '.join(f'{x:.3f}' for x in q_pos0)}] rad")
    print(f"To:   [{', '.join(f'{x:.3f}' for x in q_pos1)}] rad")
    print(f"Motion limits - Vel: [{', '.join(f'{x:.3f}' for x in qd_max)}] rad/s")
    
    # Create CSV file for data export
    with open("joint1_data.csv", "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        # Write header
        header = ["time"]
        header.extend([f"joint{i+1}" for i in range(ndof)])
        header.extend([f"vel{i+1}" for i in range(ndof)])
        header.extend([f"acc{i+1}" for i in range(ndof)])
        writer.writerow(header)
        
        print("\nExecuting trajectory (showing every 10th step):")
        print("t [s] | joint positions [rad] | joint velocities [rad/s]")
        print("------|----------------------|------------------------")
        
        step = 0
        result = 0
        while result == 0:  # 0=Working
            result = joint_traj.update()
            
            # Print duration after first update
            if step == 0:
                duration = joint_traj.get_duration()
                print(f"Trajectory duration: {duration:.3f}s")
            
            q_ref = joint_traj.get_curr_pos()
            qd_ref = joint_traj.get_curr_vel()
            qdd_ref = joint_traj.get_curr_acc()
            current_time = joint_traj.get_time()
            
            # Write to CSV
            row = [f"{current_time:.6f}"]
            row.extend([f"{q:.6f}" for q in q_ref])
            row.extend([f"{qd:.6f}" for qd in qd_ref])
            row.extend([f"{qdd:.6f}" for qdd in qdd_ref])
            writer.writerow(row)
            
            if step % 10 == 0:
                pos_str = ", ".join(f"{q:6.3f}" for q in q_ref)
                vel_str = ", ".join(f"{qd:6.3f}" for qd in qd_ref)
                print(f"{current_time:5.3f} | [{pos_str}] | [{vel_str}]")
            
            joint_traj.pass_to_input()
            step += 1
            
            # Safety break to prevent infinite loops
            if step > 10000:
                print("Safety break: Maximum steps reached")
                break
    
    final_joints = joint_traj.get_curr_pos()
    print(f"\nResult: {result} (0=Working, 1=Finished, 2=Error)")
    print(f"Final joint positions: [{', '.join(f'{x:.3f}' for x in final_joints)}] rad")
    print(f"Total steps: {step}")
    print("Data exported to: joint1_data.csv")
    print()


def joint_trajgen_test2():
    """Test 2: JointTrajGen with POSITION limits only"""
    print("=== Test 2: JointTrajGen with POSITION limits only ===")
    
    ndof = 6
    joint_traj = JointTrajGen(ndof, 0.001)
    
    q_pos0 = np.array([0.0, 0.0, 0.0, 1.0, -1.0, 2.0])
    q_pos1 = np.array([2.0, -2.5, 0.8, 4.0, -3.0, 6.0])
    q_upper_limits = np.ones(ndof) * 1.0
    q_lower_limits = np.ones(ndof) * -6.0
    
    joint_traj.set_start_state(q_pos0)
    joint_traj.set_target_state(q_pos1)
    joint_traj.set_joint_pos_limits(q_upper_limits, q_lower_limits)
    
    qd_max = np.array([6.0, 4.0, 2.0, 8.0, 6.0, 10.0])       # rad/s
    qdd_max = np.array([3.0, 2.0, 1.0, 4.0, 3.0, 5.0])       # rad/s^2
    qddd_max = np.array([15.0, 10.0, 5.0, 20.0, 15.0, 25.0]) # rad/s^3
    
    # Test with velocity constraints only (should work with our fix)
    # joint_traj.set_joint_motion_limits(qd_max)
    joint_traj.set_joint_motion_limits(qd_max, qdd_max)
    # joint_traj.set_joint_motion_limits(qd_max, qdd_max, qddd_max)
    
    print(f"From: [{', '.join(f'{x:.3f}' for x in q_pos0)}] rad")
    print(f"To:   [{', '.join(f'{x:.3f}' for x in q_pos1)}] rad (will be clamped)")
    print(f"Upper limits: [{', '.join(f'{x:.3f}' for x in q_upper_limits)}] rad")
    print(f"Lower limits: [{', '.join(f'{x:.3f}' for x in q_lower_limits)}] rad")
    
    # Create CSV file for data export
    with open("joint2_data.csv", "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        # Write header
        header = ["time"]
        header.extend([f"joint{i+1}" for i in range(ndof)])
        header.extend([f"vel{i+1}" for i in range(ndof)])
        header.extend([f"acc{i+1}" for i in range(ndof)])
        writer.writerow(header)
        
        print("\nExecuting trajectory (showing every 10th step):")
        print("t [s] | joint positions [rad] | joint velocities [rad/s]")
        print("------|----------------------|------------------------")
        
        step = 0
        result = 0
        while result == 0:  # 0=Working
            result = joint_traj.update()
            
            print(f"Update result: {result} (0=Working, 1=Finished, 2=Error)")
            
            q_ref = joint_traj.get_curr_pos()
            qd_ref = joint_traj.get_curr_vel()
            qdd_ref = joint_traj.get_curr_acc()
            current_time = joint_traj.get_time()
            
            print(f"Time: {current_time:.6f}s")
            
            # Write to CSV
            row = [f"{current_time:.6f}"]
            row.extend([f"{q:.6f}" for q in q_ref])
            row.extend([f"{qd:.6f}" for qd in qd_ref])
            row.extend([f"{qdd:.6f}" for qdd in qdd_ref])
            writer.writerow(row)
            
            if step % 10 == 0:
                pos_str = ", ".join(f"{q:6.3f}" for q in q_ref)
                vel_str = ", ".join(f"{qd:6.3f}" for qd in qd_ref)
                print(f"{current_time:5.3f} | [{pos_str}] | [{vel_str}]")
            
            joint_traj.pass_to_input()
            step += 1
            
            # Safety break to prevent infinite loops
            if step > 10000:
                print("Safety break: Maximum steps reached")
                break
    
    final_joints = joint_traj.get_curr_pos()
    expected_clamped = [
        min(q_pos1[0], q_upper_limits[0]),
        max(q_pos1[1], q_lower_limits[1]),
        min(q_pos1[2], q_upper_limits[2])
    ]
    print(f"\nFinal joint positions: [{', '.join(f'{x:.3f}' for x in final_joints)}] rad")
    print(f"Expected clamped to: [{expected_clamped[0]:.3f}, {expected_clamped[1]:.3f}, {expected_clamped[2]:.3f}] rad")
    print(f"Total steps: {step}")
    print("Data exported to: joint2_data.csv")
    print()


def joint_trajgen_test3():
    """Test 3: JointTrajGen with NON-ZERO boundary conditions"""
    print("=== Test 3: JointTrajGen with NON-ZERO boundary conditions ===")
    
    ndof = 3
    joint_traj = JointTrajGen(ndof, 0.001)
    
    q_pos0 = np.array([0.2, -0.3, 0.5])
    q_vel0 = np.array([0.1, -0.5, 0.2])
    q_acc0 = np.array([0.05, 0.1, -0.1])
    
    q_pos1 = np.array([2.0, -4.5, 6.8])
    q_vel1 = np.array([-0.2, 0.3, -0.1])
    q_acc1 = np.array([0.1, -0.2, 0.15])
    
    qd_max = np.ones(ndof) * 2 * math.pi
    qdd_max = np.ones(ndof) * math.pi
    
    joint_traj.set_start_state(q_pos0, q_vel0, q_acc0)
    joint_traj.set_target_state(q_pos1, q_vel1, q_acc1)
    joint_traj.set_joint_motion_limits(qd_max, qdd_max)
    
    print(f"From: pos=[{', '.join(f'{x:.3f}' for x in q_pos0)}] rad")
    print(f"      vel=[{', '.join(f'{x:.3f}' for x in q_vel0)}] rad/s")
    print(f"      acc=[{', '.join(f'{x:.3f}' for x in q_acc0)}] rad/s²")
    print(f"To:   pos=[{', '.join(f'{x:.3f}' for x in q_pos1)}] rad")
    print(f"      vel=[{', '.join(f'{x:.3f}' for x in q_vel1)}] rad/s")
    print(f"      acc=[{', '.join(f'{x:.3f}' for x in q_acc1)}] rad/s²")
    print(f"Duration: {joint_traj.get_duration():.3f}s")
    
    # Create CSV file for data export
    with open("joint3_data.csv", "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        # Write header
        header = ["time"]
        header.extend([f"joint{i+1}" for i in range(ndof)])
        header.extend([f"vel{i+1}" for i in range(ndof)])
        header.extend([f"acc{i+1}" for i in range(ndof)])
        writer.writerow(header)
        
        print("\nExecuting trajectory (showing every 15th step):")
        print("t [s] | joint positions [rad] | joint velocities [rad/s]")
        print("------|----------------------|------------------------")
        
        step = 0
        result = 0
        max_observed_vel = np.zeros(ndof)
        
        while result == 0:  # 0=Working
            result = joint_traj.update()
            
            q_ref = joint_traj.get_curr_pos()
            qd_ref = joint_traj.get_curr_vel()
            qdd_ref = joint_traj.get_curr_acc()
            current_time = joint_traj.get_time()
            
            # Track maximum observed velocities
            for i in range(ndof):
                max_observed_vel[i] = max(max_observed_vel[i], abs(qd_ref[i]))
            
            # Write to CSV
            row = [f"{current_time:.6f}"]
            row.extend([f"{q:.6f}" for q in q_ref])
            row.extend([f"{qd:.6f}" for qd in qd_ref])
            row.extend([f"{qdd:.6f}" for qdd in qdd_ref])
            writer.writerow(row)
            
            if step % 15 == 0:
                pos_str = ", ".join(f"{q:6.3f}" for q in q_ref)
                vel_str = ", ".join(f"{qd:6.3f}" for qd in qd_ref)
                print(f"{current_time:5.3f} | [{pos_str}] | [{vel_str}]")
            
            joint_traj.pass_to_input()
            step += 1
            
            # Safety break to prevent infinite loops
            if step > 10000:
                print("Safety break: Maximum steps reached")
                break
    
    final_joints = joint_traj.get_curr_pos()
    final_vels = joint_traj.get_curr_vel()
    final_accs = joint_traj.get_curr_acc()
    
    print(f"\nResult: {result} (0=Working, 1=Finished, 2=Error)")
    print(f"Final pos: [{', '.join(f'{x:.3f}' for x in final_joints)}] rad")
    print(f"Final vel: [{', '.join(f'{x:.3f}' for x in final_vels)}] rad/s")
    print(f"Final acc: [{', '.join(f'{x:.3f}' for x in final_accs)}] rad/s²")
    print(f"Target pos:[{', '.join(f'{x:.3f}' for x in q_pos1)}] rad")
    print(f"Target vel:[{', '.join(f'{x:.3f}' for x in q_vel1)}] rad/s")
    print(f"Target acc:[{', '.join(f'{x:.3f}' for x in q_acc1)}] rad/s²")
    print(f"Max observed velocities: [{', '.join(f'{x:.3f}' for x in max_observed_vel)}] rad/s")
    print(f"Velocity limits were: [{', '.join(f'{x:.3f}' for x in qd_max)}] rad/s")
    print(f"Total steps: {step}")
    print("Data exported to: joint3_data.csv")
    print()


def main():
    """Main function to run all joint trajectory tests."""
    print("=== JointTrajGen Comprehensive Testing (Dynamic Version) ===")
    print("Testing different constraint scenarios for joint trajectory generation\n")
    
    joint_trajgen_test1()
    joint_trajgen_test2()
    joint_trajgen_test3()
    
    print("=== All JointTrajGen tests completed ===")
    print("\nTest results exported as CSV files:")
    print("- joint1_data.csv")
    print("- joint2_data.csv")
    print("- joint3_data.csv")


if __name__ == "__main__":
    main()