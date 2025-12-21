"""SO101 PolicyInterpolator tests for trajectory interpolation and async chunk handling.

Usage:
    # Run all tests (including non-blocking visualization test)
    pytest tests/test_algorithms/test_so101_policy_interpolator.py -v

    # Run visualization test only
    pytest tests/test_algorithms/test_so101_policy_interpolator.py -v -k visualize

    # Interactive visualization (standalone mode)
    python tests/test_algorithms/test_so101_policy_interpolator.py --visualize
    python tests/test_algorithms/test_so101_policy_interpolator.py --visualize --chunk-size 50
    python tests/test_algorithms/test_so101_policy_interpolator.py --visualize --save output.png
"""

import argparse
import numpy as np
import pytest
from pathlib import Path

from RynnMotion.algorithms.policy_interpolator import PolicyInterpolator, lerp


# ========== Constants ==========

SO101_JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper"
]
SO101_NDOF = 6


# ========== Fixtures ==========


@pytest.fixture
def sample_trajectory_data():
    """Load sample trajectory from so101_act_example.txt (gripper pre-normalized to [0,1])."""
    data_path = Path(__file__).parent / "data" / "so101_act_example.txt"
    return np.loadtxt(data_path)


@pytest.fixture
def interpolator():
    """Create PolicyInterpolator instance."""
    return PolicyInterpolator()


def flatten_trajectory(data: np.ndarray) -> np.ndarray:
    """Convert row-major trajectory to flattened format.

    Input: (num_timesteps, ndof) array
    Output: flattened [j0_t0, j1_t0, ..., jN_t0, j0_t1, j1_t1, ...]
    """
    num_timesteps, ndof = data.shape
    flattened = np.zeros(num_timesteps * ndof)
    for t in range(num_timesteps):
        for j in range(ndof):
            flattened[t * ndof + j] = data[t, j]
    return flattened


# ========== Test Classes ==========


class TestLerp:
    """Test linear interpolation utility."""

    def test_lerp_at_start(self):
        """Lerp at t=0 returns start value."""
        result = lerp(0.0, 1.0, 0.0, 1.0)
        assert result == 0.0

    def test_lerp_at_end(self):
        """Lerp at t=duration returns end value."""
        result = lerp(0.0, 1.0, 1.0, 1.0)
        assert result == 1.0

    def test_lerp_at_midpoint(self):
        """Lerp at t=0.5 returns midpoint."""
        result = lerp(0.0, 1.0, 0.5, 1.0)
        assert result == 0.5

    def test_lerp_with_arrays(self):
        """Lerp works with numpy arrays."""
        q0 = np.array([0.0, 0.0, 0.0])
        q1 = np.array([1.0, 2.0, 3.0])
        result = lerp(q0, q1, 0.5, 1.0)
        np.testing.assert_array_almost_equal(result, [0.5, 1.0, 1.5])

    def test_lerp_past_duration(self):
        """Lerp beyond duration returns end value."""
        result = lerp(0.0, 1.0, 2.0, 1.0)
        assert result == 1.0

    def test_lerp_negative_time(self):
        """Lerp with negative time returns start value."""
        result = lerp(0.0, 1.0, -0.5, 1.0)
        assert result == 0.0

    def test_lerp_zero_duration(self):
        """Lerp with zero duration returns end value."""
        result = lerp(0.0, 1.0, 0.0, 0.0)
        assert result == 1.0


class TestBasicInterpolation:
    """Test basic trajectory interpolation."""

    def test_prepare_and_update(self, interpolator, sample_trajectory_data):
        """Test basic prepare_trajectory and update cycle."""
        chunk_size = 30
        ndof = SO101_NDOF
        chunk_data = sample_trajectory_data[:chunk_size]
        flattened = flatten_trajectory(chunk_data)

        interpolator.prepare_trajectory(flattened, ndof, chunk_size)

        # Evaluate at start
        result = interpolator.update(0.0)
        assert len(result) == ndof
        np.testing.assert_array_almost_equal(result, chunk_data[0], decimal=5)

        # Evaluate at end
        result = interpolator.update(chunk_size - 1)
        np.testing.assert_array_almost_equal(result, chunk_data[-1], decimal=5)

    def test_interpolation_smoothness(self, interpolator, sample_trajectory_data):
        """Test that interpolated values are smooth between waypoints."""
        chunk_size = 30
        ndof = SO101_NDOF
        chunk_data = sample_trajectory_data[:chunk_size]
        flattened = flatten_trajectory(chunk_data)

        interpolator.prepare_trajectory(flattened, ndof, chunk_size)

        # Sample at 0.5 index increments
        prev_result = None
        max_jump = 0.0
        for i in range(chunk_size * 2 - 1):
            idx = i * 0.5
            result = interpolator.update(idx)
            if prev_result is not None:
                jump = np.max(np.abs(np.array(result) - np.array(prev_result)))
                max_jump = max(max_jump, jump)
            prev_result = result

        # Jumps should be small for smooth interpolation
        assert max_jump < 0.5, f"Max jump {max_jump} too large for smooth interpolation"

    def test_state_tracking(self, interpolator, sample_trajectory_data):
        """Test that update() tracks current index and position."""
        chunk_size = 30
        ndof = SO101_NDOF
        chunk_data = sample_trajectory_data[:chunk_size]
        flattened = flatten_trajectory(chunk_data)

        interpolator.prepare_trajectory(flattened, ndof, chunk_size)

        # Update at index 10
        result = interpolator.update(10.0)

        # Check state tracking
        assert interpolator._current_index == 10.0
        assert interpolator._last_position is not None
        np.testing.assert_array_almost_equal(interpolator._last_position, result)

        # get_current_reference should return same value
        ref = interpolator.get_current_reference()
        np.testing.assert_array_almost_equal(ref, result)


class TestAsyncChunkHandling:
    """Test async action chunk receiving."""

    def test_no_midstage_act_first_call(self, interpolator, sample_trajectory_data):
        """Test that first prepare_trajectory_async returns False (no blending)."""
        chunk_size = 30
        ndof = SO101_NDOF
        chunk_data = sample_trajectory_data[:chunk_size]
        flattened = flatten_trajectory(chunk_data)

        # First call should return False (no previous state to blend from)
        blended = interpolator.prepare_trajectory_async(flattened, ndof, chunk_size)
        assert blended is False

    def test_sequential_execution_no_blend(self, interpolator, sample_trajectory_data):
        """Test sequential execution: complete ACT_1, then start ACT_2 fresh."""
        chunk_size = 30
        ndof = SO101_NDOF

        # ACT_1
        act1_data = sample_trajectory_data[:chunk_size]
        act1_flat = flatten_trajectory(act1_data)
        interpolator.prepare_trajectory_async(act1_flat, ndof, chunk_size)

        # Execute ACT_1 completely
        for i in range(chunk_size):
            interpolator.update(float(i))

        # Clear state by setting curves to None (simulating trajectory completion)
        interpolator._piecewise_curves = None
        interpolator._last_position = None

        # ACT_2 should start fresh (no blend)
        act2_data = sample_trajectory_data[chunk_size : chunk_size * 2]
        act2_flat = flatten_trajectory(act2_data)
        blended = interpolator.prepare_trajectory_async(act2_flat, ndof, chunk_size)
        assert blended is False

    def test_with_midstage_act(self, interpolator, sample_trajectory_data):
        """Test async interrupt: receive ACT_2 at mid-point of ACT_1."""
        chunk_size = 30
        ndof = SO101_NDOF
        mid_index = 15

        # Start ACT_1
        act1_data = sample_trajectory_data[:chunk_size]
        act1_flat = flatten_trajectory(act1_data)
        interpolator.prepare_trajectory_async(act1_flat, ndof, chunk_size)

        # Execute until mid-point
        for i in range(mid_index + 1):
            interpolator.update(float(i))

        # Get current reference before receiving ACT_2
        ref_before = interpolator.get_current_reference()
        assert ref_before is not None

        # Receive ACT_2 at mid-point
        act2_data = sample_trajectory_data[chunk_size : chunk_size * 2]
        act2_flat = flatten_trajectory(act2_data)
        blended = interpolator.prepare_trajectory_async(act2_flat, ndof, chunk_size)

        # Should return True (blending occurred)
        assert blended is True

        # Index should be reset to 0
        assert interpolator._current_index == 0.0

    def test_continuity_at_blend(self, interpolator, sample_trajectory_data):
        """Verify no position discontinuity at blend point."""
        chunk_size = 30
        ndof = SO101_NDOF
        mid_index = 15

        # Start ACT_1
        act1_data = sample_trajectory_data[:chunk_size]
        act1_flat = flatten_trajectory(act1_data)
        interpolator.prepare_trajectory_async(act1_flat, ndof, chunk_size)

        # Execute until mid-point
        for i in range(mid_index + 1):
            interpolator.update(float(i))

        # Get reference at mid-point
        ref_at_blend = interpolator.get_current_reference()

        # Receive ACT_2 (async interrupt)
        act2_data = sample_trajectory_data[chunk_size : chunk_size * 2]
        act2_flat = flatten_trajectory(act2_data)
        interpolator.prepare_trajectory_async(act2_flat, ndof, chunk_size)

        # First update after blend should match previous reference
        result_after_blend = interpolator.update(0.0)

        # Position should be continuous (no jump)
        np.testing.assert_array_almost_equal(
            result_after_blend, ref_at_blend, decimal=5,
            err_msg="Position discontinuity detected at blend point"
        )

    def test_multiple_async_interrupts(self, interpolator, sample_trajectory_data):
        """Test multiple consecutive async interrupts maintain continuity."""
        chunk_size = 30
        ndof = SO101_NDOF

        # Start ACT_1
        act1_data = sample_trajectory_data[:chunk_size]
        interpolator.prepare_trajectory_async(flatten_trajectory(act1_data), ndof, chunk_size)

        # Execute partial
        for i in range(10):
            interpolator.update(float(i))
        ref1 = interpolator.get_current_reference()

        # ACT_2 interrupt
        act2_data = sample_trajectory_data[chunk_size : chunk_size * 2]
        interpolator.prepare_trajectory_async(flatten_trajectory(act2_data), ndof, chunk_size)
        result2 = interpolator.update(0.0)
        np.testing.assert_array_almost_equal(result2, ref1, decimal=5)

        # Execute partial
        for i in range(5):
            interpolator.update(float(i))
        ref2 = interpolator.get_current_reference()

        # ACT_3 interrupt
        act3_data = sample_trajectory_data[chunk_size * 2 : chunk_size * 3]
        interpolator.prepare_trajectory_async(flatten_trajectory(act3_data), ndof, chunk_size)
        result3 = interpolator.update(0.0)
        np.testing.assert_array_almost_equal(result3, ref2, decimal=5)


class TestGripperNormalization:
    """Test gripper normalization handling."""

    def test_gripper_normalized_range(self, sample_trajectory_data):
        """Verify gripper column is normalized to [0, 1]."""
        gripper_col = sample_trajectory_data[:, 5]
        assert gripper_col.min() >= 0.0, "Gripper min should be >= 0"
        assert gripper_col.max() <= 1.0, "Gripper max should be <= 1"
        # Should have full range (fixture normalizes to 0-1)
        assert gripper_col.min() < 0.1, "Gripper should have values near 0"
        assert gripper_col.max() > 0.9, "Gripper should have values near 1"

    def test_interpolation_with_normalized_gripper(self, interpolator, sample_trajectory_data):
        """Test that interpolation works correctly with normalized gripper values."""
        chunk_size = 30
        ndof = SO101_NDOF
        chunk_data = sample_trajectory_data[:chunk_size]
        flattened = flatten_trajectory(chunk_data)

        interpolator.prepare_trajectory(flattened, ndof, chunk_size)

        # Check all interpolated gripper values are in valid range
        for i in range(chunk_size * 2):
            idx = i * 0.5
            result = interpolator.update(idx)
            gripper_val = result[5]
            assert 0.0 <= gripper_val <= 1.0, f"Gripper {gripper_val} out of [0,1] at idx {idx}"


class TestEdgeCases:
    """Test edge cases and error handling."""

    def test_update_without_prepare_raises(self, interpolator):
        """Test that update() without prepare_trajectory() raises error."""
        with pytest.raises(ValueError, match="No trajectory prepared"):
            interpolator.update(0.0)

    def test_get_current_reference_before_update(self, interpolator):
        """Test get_current_reference before any update returns None."""
        ref = interpolator.get_current_reference()
        assert ref is None

    def test_single_waypoint_chunk_raises(self, interpolator):
        """Test that single-waypoint trajectory raises (CubicSpline requires 2+ points)."""
        chunk_size = 1
        ndof = SO101_NDOF
        data = np.array([[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]])
        flattened = flatten_trajectory(data)

        with pytest.raises(ValueError, match="at least 2 elements"):
            interpolator.prepare_trajectory(flattened, ndof, chunk_size)

    def test_two_waypoint_chunk(self, interpolator):
        """Test minimum valid trajectory with 2 waypoints."""
        chunk_size = 2
        ndof = SO101_NDOF
        data = np.array([
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        ])
        flattened = flatten_trajectory(data)

        interpolator.prepare_trajectory(flattened, ndof, chunk_size)
        result = interpolator.update(0.0)
        np.testing.assert_array_almost_equal(result, data[0])
        result = interpolator.update(1.0)
        np.testing.assert_array_almost_equal(result, data[1])


class TestVisualization:
    """Visualization tests for PolicyInterpolator.

    Run with: python test_so101_policy_interpolator.py --visualize
    """

    def test_visualize_interpolation(self, sample_trajectory_data):
        """Visualize original waypoints vs interpolated curve (non-blocking)."""
        # Non-blocking test - just verify visualization function runs without error
        visualize_interpolation(sample_trajectory_data, chunk_size=30, show=False)


# ========== Visualization Function ==========


def visualize_interpolation(
    data: np.ndarray,
    chunk_size: int = 30,
    upsample_factor: int = 10,
    show: bool = True,
    save_path: str | None = None,
):
    """Visualize original waypoints (scatter) vs interpolated curve (line).

    Args:
        data: Trajectory data (num_timesteps, ndof)
        chunk_size: Number of waypoints to use
        upsample_factor: Interpolation density multiplier
        show: Whether to display plot
        save_path: Optional path to save figure
    """
    import matplotlib.pyplot as plt

    chunk_data = data[:chunk_size]
    ndof = chunk_data.shape[1]

    # Prepare interpolator
    interpolator = PolicyInterpolator()
    flattened = flatten_trajectory(chunk_data)
    interpolator.prepare_trajectory_async(flattened, ndof, chunk_size)

    # Generate interpolated curve (upsampled)
    num_samples = chunk_size * upsample_factor
    interp_indices = np.linspace(0, chunk_size - 1, num_samples)
    interp_data = np.array([interpolator.update(idx) for idx in interp_indices])

    # Plot
    _, axes = plt.subplots(2, 3, figsize=(14, 8))
    axes = axes.flatten()

    waypoint_indices = np.arange(chunk_size)

    for i in range(ndof):
        ax = axes[i]
        # Original waypoints as scatter points (blue)
        ax.scatter(waypoint_indices, chunk_data[:, i],
                   c='blue', s=30, label='Original waypoints', zorder=5)
        # Interpolated curve as line (red)
        ax.plot(interp_indices, interp_data[:, i],
                c='red', linewidth=1.5, label='Interpolated curve')
        ax.set_title(SO101_JOINT_NAMES[i])
        ax.set_xlabel('Index')
        ax.set_ylabel('Position')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.suptitle(f'SO101 PolicyInterpolator (chunk_size={chunk_size})')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved to {save_path}")

    if show:
        plt.show()
    else:
        plt.close()


# ========== Standalone Runner ==========


def main():
    parser = argparse.ArgumentParser(description="SO101 PolicyInterpolator tests")
    parser.add_argument("--visualize", action="store_true", help="Run visualization")
    parser.add_argument("--chunk-size", type=int, default=30, help="Chunk size for visualization")
    parser.add_argument("--save", type=str, default=None, help="Save visualization to file")
    args = parser.parse_args()

    if args.visualize:
        # Load data and visualize
        data_path = Path(__file__).parent / "data" / "so101_act_example.txt"
        data = np.loadtxt(data_path)
        visualize_interpolation(data, chunk_size=args.chunk_size, save_path=args.save)
    else:
        # Run pytest
        pytest.main([__file__, "-v", "-s"])


if __name__ == "__main__":
    main()
