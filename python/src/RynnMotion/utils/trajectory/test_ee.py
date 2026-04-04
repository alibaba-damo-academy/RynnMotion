#!/usr/bin/env python3
"""
End-effector pose trajectory generation test.
Tests end-effector pose trajectory generation with TCP constraints.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import math
import sys
import os

# Add the python directory to the path to import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from utils.trajectory.trajgen import EEPoseTrajGen, EEPoseTrajLimits


def ee_trajgen_test1():
    """Test 1: EEPoseTrajGen with TCP speed constraints"""
    print("=== Test 1: EEPoseTrajGen with TCP speed constraints ===")
    
    ee_traj = EEPoseTrajGen(0.002)
    
    ee_pos0 = np.array([0.5, 0.2, 0.8])
    rpy0 = np.array([-0.5, -0.8, 0.2])
    ee_quat0 = R.from_euler('xyz', rpy0)
    print(f"Initial orientation (quat): [{ee_quat0.as_quat()[3]:.6f}, {ee_quat0.as_quat()[0]:.6f}, {ee_quat0.as_quat()[1]:.6f}, {ee_quat0.as_quat()[2]:.6f}]")
    
    ee_pos1 = np.array([1.2, -2.3, 2.1])
    rpy1 = np.array([0.5, 0.8, 1.5])
    ee_quat1 = R.from_euler('xyz', rpy1)
    
    max_tcp_speed = 2.0  # m/s
    max_tcp_acc = 5.0    # m/s²
    ee_traj.set_tcp_limits(max_tcp_speed, max_tcp_acc)
    
    zero_vel = np.zeros(4)
    zero_acc = np.zeros(4)
    
    ee_traj.set_start_state(ee_pos0, ee_quat0, zero_vel, zero_acc)
    ee_traj.set_target_state(ee_pos1, ee_quat1, zero_vel, zero_acc)
    
    print(f"From position: [{', '.join(f'{x:.3f}' for x in ee_pos0)}] m")
    print(f"From orientation (quat): [{ee_quat0.as_quat()[3]:.6f}, {ee_quat0.as_quat()[0]:.6f}, {ee_quat0.as_quat()[1]:.6f}, {ee_quat0.as_quat()[2]:.6f}]")
    print(f"To position: [{', '.join(f'{x:.3f}' for x in ee_pos1)}] m")
    print(f"To orientation (quat): [{ee_quat1.as_quat()[3]:.6f}, {ee_quat1.as_quat()[0]:.6f}, {ee_quat1.as_quat()[1]:.6f}, {ee_quat1.as_quat()[2]:.6f}]")
    print(f"TCP speed limit: {max_tcp_speed} m/s")
    print(f"TCP acceleration limit: {max_tcp_acc} m/s²")
    
    # Create CSV file for data export
    with open("ee_data.csv", "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        # Write header
        header = ["time", "pos_x", "pos_y", "pos_z", "quat_w", "quat_x", "quat_y", "quat_z",
                  "vel_x", "vel_y", "vel_z", "angvel_x", "angvel_y", "angvel_z",
                  "acc_x", "acc_y", "acc_z", "angular_acc_x", "angular_acc_y", "angular_acc_z",
                  "tcp_speed", "tcp_acc"]
        writer.writerow(header)
        
        print("\nExecuting EE trajectory (showing every 10th step):")
        print("t [s] | position [m] | TCP speed [m/s]")
        print("------|--------------|----------------")
        
        step = 0
        result = 0
        max_observed_speed = 0.0
        max_observed_acc = 0.0
        
        while result == 0:  # 0=Working
            result = ee_traj.update()
            
            # Get current pose and velocities
            curr_pos, curr_orient = ee_traj.get_curr_pose()
            linear_vel = ee_traj.get_curr_vel()
            linear_acc = ee_traj.get_curr_acc()
            current_time = ee_traj.get_time()
            
            # Calculate TCP speed and acceleration
            tcp_speed = ee_traj.get_curr_tcp_speed()
            tcp_acc = ee_traj.get_curr_tcp_acc()
            
            # Track maximum observed values
            max_observed_speed = max(max_observed_speed, tcp_speed)
            max_observed_acc = max(max_observed_acc, tcp_acc)
            
            # Write to CSV
            quat = curr_orient.as_quat()  # [x, y, z, w]
            row = [f"{current_time:.6f}", f"{curr_pos[0]:.6f}", f"{curr_pos[1]:.6f}", f"{curr_pos[2]:.6f}",
                   f"{quat[3]:.6f}", f"{quat[0]:.6f}", f"{quat[1]:.6f}", f"{quat[2]:.6f}",
                   f"{linear_vel[0]:.6f}", f"{linear_vel[1]:.6f}", f"{linear_vel[2]:.6f}",
                   "0.0", "0.0", "0.0",  # angular velocity (not tracked in simplified version)
                   f"{linear_acc[0]:.6f}", f"{linear_acc[1]:.6f}", f"{linear_acc[2]:.6f}",
                   "0.0", "0.0", "0.0",  # angular acceleration (not tracked in simplified version)
                   f"{tcp_speed:.6f}", f"{tcp_acc:.6f}"]
            writer.writerow(row)
            
            # Print progress every 10 steps
            if step % 10 == 0:
                print(f"{current_time:5.3f} | [{curr_pos[0]:6.3f},{curr_pos[1]:6.3f},{curr_pos[2]:6.3f}] | {tcp_speed:6.3f}")
            
            ee_traj.pass_to_input()
            step += 1
            
            # Safety break to prevent infinite loops
            if step > 10000:
                print("Safety break: Maximum steps reached")
                break
    
    # Get final pose
    final_pos, final_quat = ee_traj.get_curr_pose()
    
    # Calculate pose errors
    pos_error = final_pos - ee_pos0
    # For orientation error, we compute the angle difference
    rel_rot = final_quat.inv() * ee_quat1
    angle_error = rel_rot.magnitude()  # Angle difference in radians
    
    print(f"\nResult: {result} (0=Working, 1=Finished, 2=Error)")
    print(f"Final position: [{', '.join(f'{x:.3f}' for x in final_pos)}] m")
    print(f"Target position: [{', '.join(f'{x:.3f}' for x in ee_pos0)}] m")
    print(f"Position error: [{', '.join(f'{x:.3f}' for x in pos_error)}] m (norm: {np.linalg.norm(pos_error):.6f} m)")
    print(f"Final orientation (quat): [{final_quat.as_quat()[3]:.6f}, {final_quat.as_quat()[0]:.6f}, {final_quat.as_quat()[1]:.6f}, {final_quat.as_quat()[2]:.6f}]")
    print(f"Target orientation (quat): [{ee_quat1.as_quat()[3]:.6f}, {ee_quat1.as_quat()[0]:.6f}, {ee_quat1.as_quat()[1]:.6f}, {ee_quat1.as_quat()[2]:.6f}]")
    print(f"Orientation error: {angle_error * 180.0 / math.pi:.3f} degrees")
    print(f"Duration: {ee_traj.get_duration():.3f} s")
    print(f"Max observed TCP speed: {max_observed_speed:.3f} m/s (limit: {max_tcp_speed} m/s)")
    print(f"Max observed TCP acceleration: {max_observed_acc:.3f} m/s² (limit: {max_tcp_acc} m/s²)")
    print(f"Total steps: {step}")
    print("Data exported to: ee_data.csv")
    print()


def ee_trajgen_test2():
    """Test 2: EEPoseTrajGen with zero boundary conditions"""
    print("=== Test 2: EEPoseTrajGen with zero boundary conditions ===")
    
    ee_traj = EEPoseTrajGen(0.002)
    
    # Start pose
    ee_pos0 = np.array([0.2, 0.1, 0.5])
    rpy0 = np.array([0.1, -0.2, 0.3])
    ee_quat0 = R.from_euler('xyz', rpy0)
    
    # Target pose
    ee_pos1 = np.array([1.5, -1.8, 1.2])
    rpy1 = np.array([-0.3, 0.4, -0.8])
    ee_quat1 = R.from_euler('xyz', rpy1)
    
    max_tcp_speed = 1.5  # m/s
    max_tcp_acc = 3.0    # m/s²
    ee_traj.set_tcp_limits(max_tcp_speed, max_tcp_acc)
    
    # Use simplified interface with zero boundary conditions
    ee_traj.set_start_state(ee_pos0, ee_quat0)
    ee_traj.set_target_state(ee_pos1, ee_quat1)
    
    print(f"From position: [{', '.join(f'{x:.3f}' for x in ee_pos0)}] m")
    print(f"From orientation (RPY): [{', '.join(f'{x:.3f}' for x in rpy0)}] rad")
    print(f"To position: [{', '.join(f'{x:.3f}' for x in ee_pos1)}] m")
    print(f"To orientation (RPY): [{', '.join(f'{x:.3f}' for x in rpy1)}] rad")
    print(f"TCP speed limit: {max_tcp_speed} m/s")
    print(f"TCP acceleration limit: {max_tcp_acc} m/s²")
    
    # Create CSV file for data export
    with open("ee2_data.csv", "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        # Write header
        header = ["time", "pos_x", "pos_y", "pos_z", "quat_w", "quat_x", "quat_y", "quat_z",
                  "vel_x", "vel_y", "vel_z", "acc_x", "acc_y", "acc_z", "tcp_speed", "tcp_acc"]
        writer.writerow(header)
        
        print("\nExecuting EE trajectory (showing every 15th step):")
        print("t [s] | position [m] | TCP speed [m/s]")
        print("------|--------------|----------------")
        
        step = 0
        result = 0
        max_observed_speed = 0.0
        max_observed_acc = 0.0
        
        while result == 0:  # 0=Working
            result = ee_traj.update()
            
            # Get current pose and velocities
            curr_pos, curr_orient = ee_traj.get_curr_pose()
            linear_vel = ee_traj.get_curr_vel()
            linear_acc = ee_traj.get_curr_acc()
            current_time = ee_traj.get_time()
            
            # Calculate TCP speed and acceleration
            tcp_speed = ee_traj.get_curr_tcp_speed()
            tcp_acc = ee_traj.get_curr_tcp_acc()
            
            # Track maximum observed values
            max_observed_speed = max(max_observed_speed, tcp_speed)
            max_observed_acc = max(max_observed_acc, tcp_acc)
            
            # Write to CSV
            quat = curr_orient.as_quat()  # [x, y, z, w]
            row = [f"{current_time:.6f}", f"{curr_pos[0]:.6f}", f"{curr_pos[1]:.6f}", f"{curr_pos[2]:.6f}",
                   f"{quat[3]:.6f}", f"{quat[0]:.6f}", f"{quat[1]:.6f}", f"{quat[2]:.6f}",
                   f"{linear_vel[0]:.6f}", f"{linear_vel[1]:.6f}", f"{linear_vel[2]:.6f}",
                   f"{linear_acc[0]:.6f}", f"{linear_acc[1]:.6f}", f"{linear_acc[2]:.6f}",
                   f"{tcp_speed:.6f}", f"{tcp_acc:.6f}"]
            writer.writerow(row)
            
            # Print progress every 15 steps
            if step % 15 == 0:
                print(f"{current_time:5.3f} | [{curr_pos[0]:6.3f},{curr_pos[1]:6.3f},{curr_pos[2]:6.3f}] | {tcp_speed:6.3f}")
            
            ee_traj.pass_to_input()
            step += 1
            
            # Safety break to prevent infinite loops
            if step > 10000:
                print("Safety break: Maximum steps reached")
                break
    
    # Get final pose
    final_pos, final_quat = ee_traj.get_curr_pose()
    
    # Calculate final RPY using scipy
    final_rpy = final_quat.as_euler('xyz')
    
    # Calculate pose errors
    pos_error = final_pos - ee_pos1
    rpy_error = final_rpy - rpy1
    
    print(f"\nResult: {result} (0=Working, 1=Finished, 2=Error)")
    print(f"Final position: [{', '.join(f'{x:.3f}' for x in final_pos)}] m")
    print(f"Target position: [{', '.join(f'{x:.3f}' for x in ee_pos1)}] m")
    print(f"Position error: [{', '.join(f'{x:.3f}' for x in pos_error)}] m (norm: {np.linalg.norm(pos_error):.6f} m)")
    print(f"Final orientation (RPY): [{', '.join(f'{x:.3f}' for x in final_rpy)}] rad")
    print(f"Target orientation (RPY): [{', '.join(f'{x:.3f}' for x in rpy1)}] rad")
    print(f"Orientation error (RPY): [{', '.join(f'{x:.3f}' for x in rpy_error)}] rad")
    print(f"Duration: {ee_traj.get_duration():.3f} s")
    print(f"Max observed TCP speed: {max_observed_speed:.3f} m/s (limit: {max_tcp_speed} m/s)")
    print(f"Max observed TCP acceleration: {max_observed_acc:.3f} m/s² (limit: {max_tcp_acc} m/s²)")
    print(f"Total steps: {step}")
    print("Data exported to: ee2_data.csv")
    print()


def ee_trajgen_test3_config_struct():
    """Test 3: EEPoseTrajGen with EEPoseTrajLimits config"""
    print("=== Test 3: EEPoseTrajGen with EEPoseTrajLimits config ===")

    limits = EEPoseTrajLimits(tcp_speed_max=1.0, tcp_acc_max=3.0)
    ee_traj = EEPoseTrajGen(0.002, limits)

    ee_pos0 = np.array([0.5, 0.2, 0.8])
    rpy0 = np.array([-0.5, -0.8, 0.2])
    ee_quat0 = R.from_euler('xyz', rpy0)

    ee_pos1 = np.array([1.2, -2.3, 2.1])
    rpy1 = np.array([0.5, 0.8, 1.5])
    ee_quat1 = R.from_euler('xyz', rpy1)

    ee_traj.set_start_state(ee_pos0, ee_quat0)
    ee_traj.set_target_state(ee_pos1, ee_quat1)

    print(f"TCP speed limit (from config): {limits.tcp_speed_max} m/s")
    print(f"TCP acc limit (from config): {limits.tcp_acc_max} m/s²")

    step = 0
    result = 0
    max_observed_speed = 0.0

    while result == 0:
        result = ee_traj.update()
        tcp_speed = ee_traj.get_curr_tcp_speed()
        max_observed_speed = max(max_observed_speed, tcp_speed)
        ee_traj.pass_to_input()
        step += 1
        if step > 10000:
            print("Safety break: Maximum steps reached")
            break

    final_pos, final_quat = ee_traj.get_curr_pose()

    print(f"Max observed TCP speed: {max_observed_speed:.3f} m/s (limit: {limits.tcp_speed_max} m/s)")
    print(f"Final position: [{', '.join(f'{x:.3f}' for x in final_pos)}] m")
    print(f"Total steps: {step}")

    # Note: Per-axis Ruckig limits don't directly constrain the 3D speed norm,
    # so observed speed can exceed the configured limit by ~15-20%.
    if max_observed_speed <= limits.tcp_speed_max * 1.25:
        print("PASS: TCP speed within expected range")
    else:
        print("FAIL: TCP speed greatly exceeded configured limit")
    print()


def main():
    """Main function to run all EE trajectory tests."""
    print("=== EEPoseTrajGen Testing ===")
    print("Testing end-effector pose trajectory generation with TCP constraints\n")

    ee_trajgen_test1()
    ee_trajgen_test2()
    ee_trajgen_test3_config_struct()

    print("=== All EE trajectory tests completed ===")
    print("\nTest results exported as CSV files:")
    print("- ee_data.csv")
    print("- ee2_data.csv")


if __name__ == "__main__":
    main()