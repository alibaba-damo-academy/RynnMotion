#!/usr/bin/env python3
"""
Plot joint trajectory data with subplots for each joint.
Each joint gets one subplot showing position, velocity, and acceleration.

Usage:
    python3 plot_jointtraj.py <csv_file>
    python3 plot_jointtraj.py joint_trajectory.csv
    python3 plot_jointtraj.py test1_no_constraints.csv
"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys
import os


def detect_joint_count(data):
    """Automatically detect number of joints from CSV columns"""
    joint_cols = [col for col in data.columns if col.startswith("joint")]
    if joint_cols:
        # Extract joint numbers and find maximum
        joint_nums = [int(col.replace("joint", "")) for col in joint_cols]
        return max(joint_nums)
    else:
        # Try alternative naming (joint1, joint2, etc.)
        joint_cols = [
            col for col in data.columns if col.startswith("joint") and col[-1].isdigit()
        ]
        if joint_cols:
            joint_nums = [int(col[-1]) for col in joint_cols]
            return max(joint_nums)
    return 0


def plot_joint_trajectory(csv_file):
    """Plot joint trajectory data from CSV file"""
    try:
        # Read joint trajectory data
        data = pd.read_csv(csv_file)
        print(f"Loaded {len(data)} trajectory points from {csv_file}")

        # Auto-detect number of joints
        num_joints = detect_joint_count(data)
        if num_joints == 0:
            print("Error: No joint columns found in CSV file")
            print(
                "Expected columns like: joint1, joint2, ..., vel1, vel2, ..., acc1, acc2, ..."
            )
            return None

        print(f"Detected {num_joints} joints")

        # Create subplots - one for each joint
        rows = (num_joints + 2) // 3  # 3 joints per row
        cols = min(3, num_joints)
        fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))

        if rows == 1 and cols == 1:
            axes = [axes]
        elif rows == 1:
            axes = axes
        else:
            axes = axes.flatten()

        # Get actual trajectory duration from data
        trajectory_duration = data["time"].iloc[-1]

        fig.suptitle(
            f"Joint Trajectory Analysis ({num_joints} DOF)\n{os.path.basename(csv_file)}",
            fontsize=16,
            fontweight="bold",
        )

        # Define colors for position, velocity, acceleration
        pos_color = "blue"
        vel_color = "red"
        acc_color = "green"

        # Plot each joint
        for i in range(num_joints):
            ax = axes[i] if num_joints > 1 else axes[0]

            # Column names for this joint
            pos_col = f"joint{i+1}"
            vel_col = f"vel{i+1}"
            acc_col = f"acc{i+1}"

            # Check if all required columns exist
            if pos_col not in data.columns:
                print(f"Warning: Position column '{pos_col}' not found")
                continue

            # Plot position (primary y-axis)
            ax.plot(
                data["time"],
                data[pos_col],
                color=pos_color,
                linewidth=2,
                label="Position [rad]",
                solid_capstyle="round",
            )
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Position [rad]", color=pos_color)
            ax.tick_params(axis="y", labelcolor=pos_color)
            ax.set_xlim(0, trajectory_duration * 1.05)  # Add 5% margin
            ax.grid(True, alpha=0.3)

            # Add start/end markers for position
            ax.scatter(
                data["time"].iloc[0],
                data[pos_col].iloc[0],
                color=pos_color,
                s=80,
                marker="o",
                edgecolors="black",
                zorder=5,
            )
            ax.scatter(
                data["time"].iloc[-1],
                data[pos_col].iloc[-1],
                color=pos_color,
                s=80,
                marker="s",
                edgecolors="black",
                zorder=5,
            )

            # Create secondary y-axis for velocity if column exists
            if vel_col in data.columns:
                ax2 = ax.twinx()
                ax2.plot(
                    data["time"],
                    data[vel_col],
                    color=vel_color,
                    linewidth=1.5,
                    linestyle="--",
                    alpha=0.8,
                    label="Velocity [rad/s]",
                )
                ax2.set_ylabel("Velocity [rad/s]", color=vel_color)
                ax2.tick_params(axis="y", labelcolor=vel_color)

                # Create tertiary y-axis for acceleration if column exists
                if acc_col in data.columns:
                    ax3 = ax.twinx()
                    # Offset the third axis
                    ax3.spines["right"].set_position(("outward", 60))
                    ax3.plot(
                        data["time"],
                        data[acc_col],
                        color=acc_color,
                        linewidth=1,
                        linestyle=":",
                        alpha=0.7,
                        label="Acceleration [rad/s²]",
                    )
                    ax3.set_ylabel("Acceleration [rad/s²]", color=acc_color)
                    ax3.tick_params(axis="y", labelcolor=acc_color)

            ax.set_title(f"Joint {i+1}", fontweight="bold")

            # Add legend combining all three lines
            lines = [ax.get_lines()[0]]
            labels = ["Position [rad]"]
            if vel_col in data.columns:
                lines.append(ax2.get_lines()[0])
                labels.append("Velocity [rad/s]")
                if acc_col in data.columns:
                    lines.append(ax3.get_lines()[0])
                    labels.append("Acceleration [rad/s²]")

            ax.legend(lines, labels, loc="upper right", fontsize=8)

        # Hide unused subplots
        for i in range(num_joints, len(axes)):
            axes[i].set_visible(False)

        plt.tight_layout()

        # Save plot
        output_name = f"jointtraj_{os.path.splitext(os.path.basename(csv_file))[0]}.png"
        plt.savefig(output_name, dpi=300, bbox_inches="tight")
        print(f"Plot saved to: {output_name}")

        # Print trajectory statistics
        duration = data["time"].iloc[-1]
        print(f"\n=== Joint Trajectory Statistics ===")
        print(f"Duration: {duration:.3f} seconds")
        print(f"Number of samples: {len(data)}")
        print(f"Sample rate: {len(data)/duration:.1f} Hz")
        print("Final joint positions [rad]:")

        # Calculate and display statistics for each joint
        for i in range(num_joints):
            pos_col = f"joint{i+1}"
            vel_col = f"vel{i+1}"
            acc_col = f"acc{i+1}"

            if pos_col in data.columns:
                start_pos = data[pos_col].iloc[0]
                final_pos = data[pos_col].iloc[-1]
                pos_range = data[pos_col].max() - data[pos_col].min()
                print(
                    f"  Joint {i+1}: {start_pos:+7.3f} → {final_pos:+7.3f} rad (range: {pos_range:.3f})"
                )

                # Velocity statistics if available
                if vel_col in data.columns:
                    max_vel = abs(data[vel_col]).max()
                    print(f"           Max velocity: {max_vel:.3f} rad/s")

                # Acceleration statistics if available
                if acc_col in data.columns:
                    max_acc = abs(data[acc_col]).max()
                    print(f"           Max acceleration: {max_acc:.3f} rad/s²")

        return fig

    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found!")
        return None
    except KeyError as e:
        print(f"Error: Missing column {e} in CSV file")
        return None
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return None


def main():
    """Main function with command line argument handling"""
    if len(sys.argv) != 2:
        print("Usage: python3 plot_jointtraj.py <csv_file>")
        print("Example: python3 plot_jointtraj.py joint_trajectory.csv")
        print("         python3 plot_jointtraj.py test1_no_constraints.csv")
        sys.exit(1)

    csv_file = sys.argv[1]

    print("=== Joint Trajectory Plotter ===")

    fig = plot_joint_trajectory(csv_file)

    if fig is not None:
        plt.show()
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
