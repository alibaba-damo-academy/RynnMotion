#!/usr/bin/env python3
"""
Plot end-effector trajectory data with three subplots:
1. End-Effector Position (x, y, z)
2. End-Effector Linear Velocity (x, y, z)
3. End-Effector Orientation (quaternion w, x, y, z)

Usage:
    python3 plot_eetraj.py <csv_file>
    python3 plot_eetraj.py ee_pose_trajectory.csv
"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys
import os


def plot_ee_trajectory(csv_file):
    """Plot end-effector trajectory data from CSV file"""
    try:
        # Read EE trajectory data
        data = pd.read_csv(csv_file)
        print(f"Loaded {len(data)} trajectory points from {csv_file}")

        # Get actual trajectory duration from data
        trajectory_duration = data["time"].iloc[-1]

        # Create figure with 3 subplots
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle(
            f"End-Effector Trajectory Analysis\n{os.path.basename(csv_file)}",
            fontsize=16,
            fontweight="bold",
        )

        # Subplot 1: End-Effector Position
        ax1.plot(data["time"], data["pos_x"], "r-", linewidth=2, label="X [m]")
        ax1.plot(data["time"], data["pos_y"], "g-", linewidth=2, label="Y [m]")
        ax1.plot(data["time"], data["pos_z"], "b-", linewidth=2, label="Z [m]")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Position [m]")
        ax1.set_title("End-Effector Position Trajectory")
        ax1.set_xlim(0, trajectory_duration * 1.05)  # Add 5% margin
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Add start/end markers
        ax1.scatter(
            data["time"].iloc[0],
            data["pos_x"].iloc[0],
            color="red",
            s=100,
            marker="o",
            edgecolors="black",
            zorder=5,
        )
        ax1.scatter(
            data["time"].iloc[-1],
            data["pos_x"].iloc[-1],
            color="red",
            s=100,
            marker="s",
            edgecolors="black",
            zorder=5,
        )
        ax1.text(
            data["time"].iloc[0],
            data["pos_x"].iloc[0],
            "  Start",
            fontsize=10,
            va="center",
        )
        ax1.text(
            data["time"].iloc[-1],
            data["pos_x"].iloc[-1],
            "  End",
            fontsize=10,
            va="center",
        )

        # Subplot 2: End-Effector Linear Velocity
        ax2.plot(data["time"], data["vel_x"], "r-", linewidth=2, label="Vel X [m/s]")
        ax2.plot(data["time"], data["vel_y"], "g-", linewidth=2, label="Vel Y [m/s]")
        ax2.plot(data["time"], data["vel_z"], "b-", linewidth=2, label="Vel Z [m/s]")

        # Calculate and plot velocity magnitude
        vel_mag = np.sqrt(data["vel_x"] ** 2 + data["vel_y"] ** 2 + data["vel_z"] ** 2)
        ax2.plot(
            data["time"],
            vel_mag,
            "k--",
            linewidth=2,
            alpha=0.8,
            label="Magnitude [m/s]",
        )

        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Linear Velocity [m/s]")
        ax2.set_title("End-Effector Linear Velocity")
        ax2.set_xlim(0, trajectory_duration * 1.05)  # Add 5% margin
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Print velocity statistics
        max_vel = vel_mag.max()
        max_vel_time = data["time"].iloc[vel_mag.argmax()]
        print(f"Max velocity: {max_vel:.3f} m/s at t={max_vel_time:.3f}s")

        # Subplot 3: End-Effector Orientation (Quaternion)
        ax3.plot(data["time"], data["quat_w"], "k-", linewidth=2, label="W")
        ax3.plot(data["time"], data["quat_x"], "r--", linewidth=2, label="X")
        ax3.plot(data["time"], data["quat_y"], "g--", linewidth=2, label="Y")
        ax3.plot(data["time"], data["quat_z"], "b--", linewidth=2, label="Z")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Quaternion Components")
        ax3.set_title("End-Effector Orientation (Quaternion)")
        ax3.set_xlim(0, trajectory_duration * 1.05)  # Add 5% margin
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Verify quaternion normalization
        quat_magnitude = np.sqrt(
            data["quat_w"] ** 2
            + data["quat_x"] ** 2
            + data["quat_y"] ** 2
            + data["quat_z"] ** 2
        )
        print(
            f"Quaternion magnitude range: {quat_magnitude.min():.6f} to {quat_magnitude.max():.6f}"
        )

        # Add quaternion magnitude as reference line
        ax3_twin = ax3.twinx()
        ax3_twin.plot(
            data["time"],
            quat_magnitude,
            "gray",
            alpha=0.5,
            linestyle=":",
            linewidth=1,
            label="|q| (should be 1)",
        )
        ax3_twin.set_ylabel("Quaternion Magnitude", color="gray")
        ax3_twin.tick_params(axis="y", labelcolor="gray")
        ax3_twin.set_ylim(0.99, 1.01)

        plt.tight_layout()

        # Save plot
        output_name = f"eetraj_{os.path.splitext(os.path.basename(csv_file))[0]}.png"
        plt.savefig(output_name, dpi=300, bbox_inches="tight")
        print(f"Plot saved to: {output_name}")

        # Print trajectory statistics
        duration = data["time"].iloc[-1]
        print(f"\n=== End-Effector Trajectory Statistics ===")
        print(f"Duration: {duration:.3f} seconds")
        print(f"Number of samples: {len(data)}")
        print(f"Sample rate: {len(data)/duration:.1f} Hz")
        print(
            f"Start position: [{data['pos_x'].iloc[0]:.3f}, {data['pos_y'].iloc[0]:.3f}, {data['pos_z'].iloc[0]:.3f}] m"
        )
        print(
            f"End position:   [{data['pos_x'].iloc[-1]:.3f}, {data['pos_y'].iloc[-1]:.3f}, {data['pos_z'].iloc[-1]:.3f}] m"
        )

        return fig

    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found!")
        return None
    except KeyError as e:
        print(f"Error: Missing column {e} in CSV file. Expected columns:")
        print(
            "time,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,vel_x,vel_y,vel_z,angvel_x,angvel_y,angvel_z"
        )
        return None
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return None


def main():
    """Main function with command line argument handling"""
    if len(sys.argv) != 2:
        print("Usage: python3 plot_eetraj.py <csv_file>")
        print("Example: python3 plot_eetraj.py ee_pose_trajectory.csv")
        sys.exit(1)

    csv_file = sys.argv[1]

    print("=== End-Effector Trajectory Plotter ===")

    fig = plot_ee_trajectory(csv_file)

    if fig is not None:
        plt.show()
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
