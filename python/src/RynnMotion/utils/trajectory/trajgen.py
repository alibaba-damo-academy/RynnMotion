"""
Trajectory generation module for robotics applications.
Provides classes for generating joint-space and end-effector trajectories
with various constraints and boundary conditions, based on the Ruckig library.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from enum import Enum
import ruckig


class TrajGen:
    """
    Base trajectory generation class using Ruckig online trajectory generation.
    """
    
    def __init__(self, dof, dt):
        """
        Initialize trajectory generator.
        
        Args:
            dof (int): Degrees of freedom
            dt (float): Time step in seconds
        """
        self.dof = dof
        self.dt = dt
        self.time = 0.0
        
        # Ruckig online trajectory generator
        self.otg = ruckig.Ruckig(dof, dt)
        self.input = ruckig.InputParameter(dof)
        self.output = ruckig.OutputParameter(dof)
        
        # Initialize with default values
        self.input.current_position = [0.0] * dof
        self.input.current_velocity = [0.0] * dof
        self.input.current_acceleration = [0.0] * dof
        self.input.target_position = [0.0] * dof
        self.input.target_velocity = [0.0] * dof
        self.input.target_acceleration = [0.0] * dof
        self.input.max_velocity = [1e3] * dof
        self.input.max_acceleration = [1e3] * dof
        self.input.max_jerk = [1e3] * dof
        
        # History
        self.last_target_pos = np.zeros(dof)
        self.last_target_vel = np.zeros(dof)
        self.last_target_acc = np.zeros(dof)
        self.last_plan_pos = np.zeros(dof)
        self.last_plan_vel = np.zeros(dof)
        self.last_plan_acc = np.zeros(dof)
    
    def reset(self):
        """Reset the trajectory generator state."""
        self.time = 0.0
        self.last_target_pos = np.zeros(self.dof)
        self.last_target_vel = np.zeros(self.dof)
        self.last_target_acc = np.zeros(self.dof)
        self.last_plan_pos = np.zeros(self.dof)
        self.last_plan_vel = np.zeros(self.dof)
        self.last_plan_acc = np.zeros(self.dof)
        
        # Reset Ruckig output
        self.output.time = 0.0
    
    def set_start_state(self, pos, vel=None, acc=None):
        """
        Set start state for trajectory generation.
        
        Args:
            pos (np.array): Start position
            vel (np.array, optional): Start velocity
            acc (np.array, optional): Start acceleration
        """
        self.input.current_position = pos.tolist()
        
        if vel is not None and len(vel) == self.dof:
            self.input.current_velocity = vel.tolist()
        else:
            # Default to zero velocity if not provided
            self.input.current_velocity = [0.0] * self.dof
                
        if acc is not None and len(acc) == self.dof:
            self.input.current_acceleration = acc.tolist()
        else:
            # Default to zero acceleration if not provided
            self.input.current_acceleration = [0.0] * self.dof
    
    def set_target_state(self, pos, vel=None, acc=None):
        """
        Set target state for trajectory generation.
        
        Args:
            pos (np.array): Target position
            vel (np.array, optional): Target velocity
            acc (np.array, optional): Target acceleration
        """
        self.input.target_position = pos.tolist()
        
        if vel is not None and len(vel) == self.dof:
            self.input.target_velocity = vel.tolist()
        else:
            # Default to zero velocity if not provided
            self.input.target_velocity = [0.0] * self.dof
                
        if acc is not None and len(acc) == self.dof:
            self.input.target_acceleration = acc.tolist()
        else:
            # Default to zero acceleration if not provided
            self.input.target_acceleration = [0.0] * self.dof
    
    def update(self):
        """
        Update trajectory calculation.
        
        Returns:
            int: Trajectory status (0=Working, 1=Finished, 2=Error)
        """
        # Apply saturation (to be overridden by subclasses)
        self.apply_saturation()
        
        # Store current states
        curr_pos = np.copy(self.input.current_position)
        curr_vel = np.copy(self.input.current_velocity)
        curr_acc = np.copy(self.input.current_acceleration)
        
        # Update with Ruckig
        result = self.otg.update(self.input, self.output)
        
        # Handle errors by trying with zero target acceleration
        if result not in [ruckig.Result.Working, ruckig.Result.Finished]:
            original_target_acc = np.copy(self.input.target_acceleration)
            self.input.target_acceleration = [0.0] * self.dof
            
            result = self.otg.update(self.input, self.output)
            
            if result not in [ruckig.Result.Working, ruckig.Result.Finished]:
                # Restore original target acceleration and reset output
                self.input.target_acceleration = original_target_acc.tolist()
                self.output.new_position = curr_pos
                self.output.new_velocity = curr_vel
                self.output.new_acceleration = curr_acc
        
        self.update_state_history()
        self.time = self.output.time
        
        # Convert Ruckig result to integer for compatibility
        if result == ruckig.Result.Working:
            return 0
        elif result == ruckig.Result.Finished:
            return 1
        else:
            return 2
    
    def update_state_history(self):
        """Update state history variables."""
        self.last_target_pos = np.array(self.input.target_position)
        self.last_target_vel = np.array(self.input.target_velocity)
        self.last_target_acc = np.array(self.input.target_acceleration)
        self.last_plan_pos = np.array(self.output.new_position)
        self.last_plan_vel = np.array(self.output.new_velocity)
        self.last_plan_acc = np.array(self.output.new_acceleration)
    
    def apply_saturation(self):
        """Apply saturation (to be overridden by subclasses)."""
        pass
    
    def pass_to_input(self):
        """Pass output to input for next iteration."""
        self.output.pass_to_input(self.input)
    
    def get_time(self):
        """Get current time."""
        return self.time
    
    def get_duration(self):
        """Get trajectory duration."""
        return self.output.trajectory.duration
    
    def get_input(self):
        """Get input parameters."""
        return self.input
    
    def get_output(self):
        """Get output parameters."""
        return self.output


class JointTrajGen(TrajGen):
    """
    Joint-space trajectory generation with joint limits, based on Ruckig.
    """
    
    def __init__(self, dof, dt):
        """
        Initialize joint trajectory generator.
        
        Args:
            dof (int): Number of degrees of freedom
            dt (float): Time step in seconds
        """
        super().__init__(dof, dt)
        
        # Initialize with default values
        self.q_upper_limits = np.ones(dof) * 1e3
        self.q_lower_limits = np.ones(dof) * -1e3
        self.qd_max = np.ones(dof) * 1e3
        self.qdd_max = np.ones(dof) * 1e3
        self.qddd_max = np.ones(dof) * 1e3
    
    def set_start_state(self, pos, vel=None, acc=None):
        """
        Set start state for joint trajectory generation.
        
        Args:
            pos (np.array): Start joint positions
            vel (np.array, optional): Start joint velocities
            acc (np.array, optional): Start joint accelerations
        """
        assert len(pos) == self.dof, f"Position vector size {len(pos)} does not match DOF {self.dof}"
        super().set_start_state(pos, vel, acc)
    
    def set_target_state(self, pos, vel=None, acc=None):
        """
        Set target state for joint trajectory generation.
        
        Args:
            pos (np.array): Target joint positions
            vel (np.array, optional): Target joint velocities
            acc (np.array, optional): Target joint accelerations
        """
        assert len(pos) == self.dof, f"Position vector size {len(pos)} does not match DOF {self.dof}"
        super().set_target_state(pos, vel, acc)
    
    def set_joint_motion_limits(self, qd_max, qdd_max=None, qddd_max=None):
        """
        Set joint motion limits.
        
        Args:
            qd_max (np.array): Maximum joint velocities
            qdd_max (np.array, optional): Maximum joint accelerations
            qddd_max (np.array, optional): Maximum joint jerks
        """
        assert len(qd_max) == self.dof, f"Velocity limit size {len(qd_max)} does not match DOF {self.dof}"
        
        self.qd_max = np.array(qd_max)
        self.input.max_velocity = qd_max.tolist()
        
        if qdd_max is not None:
            assert len(qdd_max) == self.dof, f"Acceleration limit size {len(qdd_max)} does not match DOF {self.dof}"
            self.qdd_max = np.array(qdd_max)
            self.input.max_acceleration = qdd_max.tolist()
        else:
            self.input.max_acceleration = [1e3] * self.dof
            self.qdd_max = np.ones(self.dof) * 1e3
            
        if qddd_max is not None:
            assert len(qddd_max) == self.dof, f"Jerk limit size {len(qddd_max)} does not match DOF {self.dof}"
            self.qddd_max = np.array(qddd_max)
            self.input.max_jerk = qddd_max.tolist()
        else:
            self.input.max_jerk = [1e3] * self.dof
            self.qddd_max = np.ones(self.dof) * 1e3
        
    
    def set_joint_pos_limits(self, q_upper_limits, q_lower_limits):
        """
        Set joint position limits.
        
        Args:
            q_upper_limits (np.array): Upper position limits
            q_lower_limits (np.array): Lower position limits
        """
        assert len(q_upper_limits) == self.dof, f"Upper limit size {len(q_upper_limits)} does not match DOF {self.dof}"
        assert len(q_lower_limits) == self.dof, f"Lower limit size {len(q_lower_limits)} does not match DOF {self.dof}"
        
        self.q_upper_limits = np.array(q_upper_limits)
        self.q_lower_limits = np.array(q_lower_limits)
        
        # Note: Position limits don't change constraint mode - velocity must always be set
    
    def clear_joint_limits(self):
        """Clear all joint limits."""
        self.q_upper_limits = np.ones(self.dof) * 1e3
        self.q_lower_limits = np.ones(self.dof) * -1e3
        self.qd_max = np.ones(self.dof) * 1e3
        self.qdd_max = np.ones(self.dof) * 1e3
        self.qddd_max = np.ones(self.dof) * 1e3
        
        # Reset Ruckig limits to defaults
        self.input.max_velocity = [1e3] * self.dof
        self.input.max_acceleration = [1e3] * self.dof
        self.input.max_jerk = [1e3] * self.dof
    
    def clear_position_limits(self):
        """Clear position limits only."""
        self.q_upper_limits = np.ones(self.dof) * 1e3
        self.q_lower_limits = np.ones(self.dof) * -1e3
    
    def apply_saturation(self):
        """Apply joint limits saturation."""
        # Apply position limits per joint
        target_position = list(self.input.target_position)  # Create a mutable copy
        for i in range(self.dof):
            if (target_position[i] > self.q_upper_limits[i] or 
                target_position[i] < self.q_lower_limits[i]):
                # Clamp position
                target_position[i] = np.clip(target_position[i], 
                                             self.q_lower_limits[i], 
                                             self.q_upper_limits[i])
                # Zero velocity and acceleration only for this joint
                self.input.target_velocity[i] = 0.0
                self.input.target_acceleration[i] = 0.0
        # Update the target position in Ruckig
        self.input.target_position = target_position
        
        # Apply velocity limits per joint
        for i in range(self.dof):
            if abs(self.input.target_velocity[i]) > self.qd_max[i]:
                # Clamp velocity
                self.input.target_velocity[i] = np.clip(self.input.target_velocity[i], 
                                                        -self.qd_max[i], 
                                                        self.qd_max[i])
                # Zero acceleration only for this joint
                self.input.target_acceleration[i] = 0.0
            # Set Ruckig internal limits
            self.input.max_velocity[i] = self.qd_max[i]
        
        # Apply acceleration limits per joint
        for i in range(self.dof):
            self.input.target_acceleration[i] = np.clip(self.input.target_acceleration[i], 
                                                        -self.qdd_max[i], 
                                                        self.qdd_max[i])
            # Set Ruckig internal limits
            self.input.max_acceleration[i] = self.qdd_max[i]
        
        # Apply jerk limits (only affects Ruckig internal limits)
        for i in range(self.dof):
            self.input.max_jerk[i] = self.qddd_max[i]
    
    def get_curr_pos(self):
        """Get current joint positions."""
        return np.array(self.output.new_position)
    
    def get_curr_vel(self):
        """Get current joint velocities."""
        return np.array(self.output.new_velocity)
    
    def get_curr_acc(self):
        """Get current joint accelerations."""
        return np.array(self.output.new_acceleration)


class EEPoseTrajGen(TrajGen):
    """
    End-effector pose trajectory generation with TCP constraints, based on Ruckig.
    """
    
    def __init__(self, dt):
        """
        Initialize EE pose trajectory generator.
        
        Args:
            dt (float): Time step in seconds
        """
        # 4 DOF for end-effector (x, y, z, angle)
        super().__init__(4, dt)
        
        # Default limits for EE
        self.input.max_velocity = [2.0, 2.0, 2.0, 1.0]      # [x, y, z, angle]
        self.input.max_acceleration = [5.0, 5.0, 5.0, 5.0]  # [x, y, z, angle]
        self.input.max_jerk = [20.0, 20.0, 20.0, 20.0]      # [x, y, z, angle]
        
        self.max_tcp_speed = float('inf')
        self.max_tcp_acc = float('inf')
        
        self.curr_quat = R.from_quat([0, 0, 0, 1])  # Identity quaternion [x, y, z, w]
        self.target_quat = R.from_quat([0, 0, 0, 1])
        self.action_axis = np.array([0, 0, 1])
        self.total_action_angle = 0.0
    
    def set_tcp_limits(self, max_speed, max_acc=None):
        """
        Set TCP (Tool Center Point) limits.
        
        Args:
            max_speed (float): Maximum TCP speed (m/s)
            max_acc (float, optional): Maximum TCP acceleration (m/s²)
        """
        self.max_tcp_speed = max_speed
        if max_acc is not None and max_acc < float('inf'):
            self.max_tcp_acc = max_acc
    
    def clear_tcp_limits(self):
        """Clear TCP limits."""
        self.max_tcp_speed = float('inf')
        self.max_tcp_acc = float('inf')
    
    def get_curr_tcp_speed(self):
        """Get current TCP speed."""
        return np.linalg.norm(np.array(self.output.new_velocity[:3]))
    
    def get_curr_tcp_acc(self):
        """Get current TCP acceleration."""
        return np.linalg.norm(np.array(self.output.new_acceleration[:3]))
    
    def set_start_state(self, pos0, quat0, vel=None, acc=None):
        """
        Set start state (pose interface).
        
        Args:
            pos0 (np.array): Start position (3D)
            quat0 (R object): Start orientation (quaternion)
            vel (np.array, optional): 4DOF velocity vector
            acc (np.array, optional): 4DOF acceleration vector
        """
        self.curr_quat = quat0
        
        # Set 4DOF state: [x, y, z, angle] - angle starts at 0
        state_4d = np.zeros(4)
        state_4d[:3] = pos0
        state_4d[3] = 0.0  # Start angle at 0
        self.input.current_position = state_4d.tolist()
        
        # Set 4DOF velocity and acceleration
        self.input.current_velocity = vel.tolist() if vel is not None else [0.0] * 4
        self.input.current_acceleration = acc.tolist() if acc is not None else [0.0] * 4
    
    def set_target_state(self, pos1, quat1, vel=None, acc=None):
        """
        Set target state (pose interface).
        
        Args:
            pos1 (np.array): Target position (3D)
            quat1 (R object): Target orientation (quaternion)
            vel (np.array, optional): 4DOF velocity vector
            acc (np.array, optional): 4DOF acceleration vector
        """
        self.target_quat = quat1
        
        # Calculate action quaternion and extract angle-axis
        action_quat = self.curr_quat.inv() * quat1
        action_rotvec = action_quat.as_rotvec()
        self.action_axis = action_rotvec / (np.linalg.norm(action_rotvec) + 1e-12)  # Add small value to avoid division by zero
        self.total_action_angle = np.linalg.norm(action_rotvec)
        
        # Set 4DOF target state: [x, y, z, total_angle]
        state_4d = np.zeros(4)
        state_4d[:3] = pos1
        state_4d[3] = self.total_action_angle  # Target angle is the total rotation
        self.input.target_position = state_4d.tolist()
        
        # Set 4DOF target velocity and acceleration
        self.input.target_velocity = vel.tolist() if vel is not None else [0.0] * 4
        self.input.target_acceleration = acc.tolist() if acc is not None else [0.0] * 4
    
    def get_curr_pose(self):
        """
        Get current pose.
        
        Returns:
            tuple: (position, orientation quaternion)
        """
        # Get current position from 4DOF output (first 3 components)
        pos = np.array(self.output.new_position[:3])
        
        # Get current angle from 4DOF output (4th component)
        current_angle = self.output.new_position[3]
        
        # Create action quaternion from current angle and stored axis
        if np.linalg.norm(self.action_axis) > 0:
            action_quat = R.from_rotvec(current_angle * self.action_axis)
            # Apply action to original quaternion
            quat = self.curr_quat * action_quat
        else:
            quat = self.curr_quat
        
        return pos, quat
    
    def get_curr_vel(self):
        """Get current linear velocity."""
        return np.array(self.output.new_velocity[:3])
    
    def get_curr_acc(self):
        """Get current linear acceleration."""
        return np.array(self.output.new_acceleration[:3])
    
    def update(self):
        """
        Update EE trajectory calculation.
        
        Returns:
            int: Trajectory status (0=Working, 1=Finished, 2=Error)
        """
        result = super().update()
        return result
    
    def apply_saturation(self):
        """Apply TCP limits saturation."""
        if self.max_tcp_speed < float('inf'):
            # Set TCP speed limit for first 3 axes (x, y, z)
            for i in range(3):
                self.input.max_velocity[i] = self.max_tcp_speed
            # Set angular velocity limit for 4th axis (angle)
            self.input.max_velocity[3] = self.max_tcp_speed  # Same limit for angular motion
        
        if self.max_tcp_acc < float('inf'):
            # Set TCP acceleration limit for first 3 axes (x, y, z)
            for i in range(3):
                self.input.max_acceleration[i] = self.max_tcp_acc
            # Set angular acceleration limit for 4th axis (angle)
            self.input.max_acceleration[3] = self.max_tcp_acc  # Same limit for angular motion