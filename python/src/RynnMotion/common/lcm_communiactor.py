"""
LCM Communication Handler

Handles all LCM communication for robot control including:
- Policy command reception (ACT commands)
- Feedback requests handling
- State and robot feedback publishing
"""

import time
import threading
import logging
import lcm
import numpy as np
import sys
from pathlib import Path

from RynnMotion.common.communicator_base import (
    CommunicatorBase,
    register_communicator_factory_func,
)

# Use absolute imports from RynnMotion package
from RynnMotion.common.lcm.lcmMotion.act_command import act_command
from RynnMotion.common.lcm.lcmMotion.act_request import act_request
from RynnMotion.common.lcm.lcmMotion.robot_feedback import robot_feedback
from RynnMotion.common.lcm.lcmMotion.state_feedback import state_feedback

from RynnMotion.common.data.basic_data import Pose


@register_communicator_factory_func("lcm")
def lcm_communicator_factory(robot_model, communicator_config: dict, logger=None):
    """
    lcm communicator class generator function, along with additional algo kwargs.

    Returns:
        lcmcommunicator_class: subclass of comunicator
        lcmcommunicator_kwargs (dict): dictionary of additional kwargs to pass to communicator
    """
    return LCMCommunicator(robot_model, communicator_config, logger)


class LCMCommunicator(CommunicatorBase):
    """
    Handles all LCM communication for robot control.

    Separates communication logic from the main controller,
    making the code more modular and maintainable.
    """

    def __init__(self, robot_model, communicator_config, logger=None):
        """
        Initialize LCM handler.

        Args:
            logger: Logger instance (optional)
        """
        super().__init__(
            robot_model=robot_model,
            communicator_config=communicator_config,
            logger=logger,
        )
        # self.logger = logger or logging.getLogger(__name__)
        self.lcm_instance = None

        self.first_timestamp = -1
        self.feedback_sequence_number = 0

        self.command_lock = threading.Lock()
        self.ACT_command_received = False
        self.current_ACT_command = None

        self.robot_feedback_requested = False
        self.state_feedback_requested = False
        self.gohome_requested = False
        self.lcm_timeout = 0  # milliseconds

        self.last_command_time = time.time()
        self.has_received_command = False
        self.act_status_index = 0
        self.stateID_index = 0
        self.substateID_index = 0
        self.connected = False

        self.act_status_cycle = [
            state_feedback.kIdle,
            state_feedback.kSuccess,
            state_feedback.kExecuting,
            state_feedback.kPaused,
            state_feedback.kCollision,
            state_feedback.kFail,
        ]
        self.stateID_cycle = [
            state_feedback.kGoStand,
            state_feedback.kGoHome,
            state_feedback.kMove1,
            state_feedback.kMove2,
            state_feedback.kError,
        ]
        self.substateID_cycle = [
            state_feedback.kEnter,
            state_feedback.kDuring,
            state_feedback.kExit,
        ]

    def connect(self):
        """Initialize LCM connection and subscribe to channels."""
        try:
            self.lcm_instance = lcm.LCM()
            self.lcm_instance.subscribe("rcp_robotmotion", self.handle_policy_commands)
            self.lcm_instance.subscribe("rcp_request_feedback", self.handle_requests)
            self.logger.info("✓ LCM communication ready")
            self.connected = True
            return True
        except Exception as e:
            self.logger.warning(f"⚠ LCM setup failed: {e}")
            self.lcm_instance = None
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect LCM."""
        if self.lcm_instance:
            self.lcm_instance = None
            self.connected = False
            self.logger.info("✓ LCM disconnected")

    def is_connected(self):
        """Check if LCM is connected."""
        return self.lcm_instance is not None

    def handle_policy_commands(self, channel, data):
        """Handle incoming policy commands, like ACT"""
        msg = act_command.decode(data)

        if self.first_timestamp == -1:
            self.first_timestamp = msg.utime
            self.logger.info("First LCM message received!")

        lcm_time = float(msg.utime - self.first_timestamp) * 1e-6
        self.logger.info(f"ACT #{msg.seq}, chunks: {msg.chunkSize}, " f"joints: {msg.numJoint}, time: {lcm_time:.3f}s")

        with self.command_lock:
            self.current_ACT_command = msg
            self.ACT_command_received = True
            self.last_command_time = time.time()
            self.has_received_command = True

    def handle_requests(self, channel, data):
        """Handle feedback request messages."""
        msg = act_request.decode(data)

        if msg.request_type == act_request.kStateFeedbackRequest:
            self.state_feedback_requested = True
        elif msg.request_type == act_request.kRobotFeedbackRequest:
            self.robot_feedback_requested = True
        elif msg.request_type == act_request.kGoHomeRequest:
            self.gohome_requested = True
            self.logger.info("Go home request received")

    def publish_state_feedback(self):
        """Send state feedback message."""
        if not self.lcm_instance:
            return

        msg = state_feedback()
        msg.sec = int(time.time())
        msg.nanosec = int((time.time() % 1) * 1e9)
        msg.utime = int(time.time() * 1000000)
        msg.seq = self.feedback_sequence_number

        msg.act_status_type = self.act_status_cycle[self.act_status_index]
        msg.stateID = self.stateID_cycle[self.stateID_index % len(self.stateID_cycle)]
        msg.substateID = self.substateID_cycle[self.substateID_index % len(self.substateID_cycle)]

        self.stateID_index += 1
        self.substateID_index += 1

        error_messages = {
            state_feedback.kIdle: "Action Idle",
            state_feedback.kSuccess: "Action Succeeded",
            state_feedback.kExecuting: "Action Executing",
            state_feedback.kPaused: "Action Paused",
            state_feedback.kCollision: "Collision Detected",
            state_feedback.kFail: "Action Failed",
        }
        msg.error_msg = error_messages.get(msg.act_status_type, "Unknown status")

        state_messages = {
            state_feedback.kGoStand: "Going to Stand",
            state_feedback.kGoHome: "Going Home",
            state_feedback.kMove1: "Executing Move 1",
            state_feedback.kMove2: "Executing Move 2",
            state_feedback.kError: "State Error Occurred",
        }
        msg.state_msg = state_messages.get(msg.stateID, "Current state unknown")

        self.lcm_instance.publish("state_feedback", msg.encode())
        self.logger.info(f"Sent state feedback #{msg.seq}")

    def publish_robot_feedback(self, robot_state):
        """
        Send robot feedback message.

        Args:
            joint_positions: Current joint positions from the robot interface
        """
        if not self.lcm_instance:
            return

        msg = robot_feedback()
        msg.sec = int(time.time())
        msg.nanosec = int((time.time() % 1) * 1e9)
        msg.utime = int(time.time() * 1000000)
        msg.seq = self.feedback_sequence_number

        msg.numJoint = robot_state.joint_number
        msg.qFb = robot_state.joint_positions[: msg.numJoint].tolist()
        msg.qdFb = robot_state.joint_velocities[: msg.numJoint].tolist()

        msg.numGripper = robot_state.gripper_number
        msg.gripperPosFb = robot_state.gripper_positions[: msg.numGripper].tolist()

        """
        msg.numPose = robot_state.ee_pose_number
        msg.eePosFb = [[0.0] * 3 for _ in range(msg.numEEpose)]
        msg.eeQuatFb = [[0.0] * 4 for _ in range(msg.numEEpose)]
        for i in range(msg.numEEpose):
            msg.eePosFb[i] = robot_state.ee_poses[i].tolist()
            msg.eeQuatFb[i] = robot_state.ee_quats[i].tolist()

        msg.numFTsensor = robot_state.ftsensor_number
        msg.ftSensorFb = [[0.0] * 6 for _ in range(msg.numFTsensor)]
        if robot_state.ftsensor_states is not None:
            msg.ftSensorFb[0] = robot_state.ftsensor_state.tolist()
        """

        self.lcm_instance.publish("robot_feedback", msg.encode())
        # self.logger.info(f"Sent robot feedback #{msg.seq}")

        self.feedback_sequence_number += 1

    def check_robot_feedback_request(self):
        """Check if robot feedback was requested and clear the flag."""
        if self.robot_feedback_requested:
            self.robot_feedback_requested = False
            return True
        return False

    def check_state_feedback_request(self):
        """Check if state feedback was requested and clear the flag."""
        if self.state_feedback_requested:
            self.state_feedback_requested = False
            return True
        return False

    def request_robot_feedback(self, joint_positions):
        """Handle robot feedback request with current joint positions."""
        self.publish_robot_feedback(joint_positions)

    def _process_trajectory(self):
        """Process a new trajectory chunk."""
        new_chunk = self.current_ACT_command
        self.robot_command.seq = new_chunk.seq

        self.robot_command.chunk_size = new_chunk.chunkSize
        # numbers
        num_joint = int(getattr(new_chunk, "numJoint", 0))
        num_gripper = int(getattr(new_chunk, "numGripper", 0))
        # try to find end-effector count field in message (tolerant)
        num_ee = 0 if new_chunk.numEEpose is None else 1

        for i in range(new_chunk.chunkSize):
            # joints
            jstart = i * num_joint
            jend = jstart + num_joint
            q_cur = new_chunk.jointPos[jstart:jend] if num_joint > 0 else []
            # prefer attribute names used in RobotState: joint_pos
            if i < len(self.robot_command.trajectory):
                self.robot_command.trajectory[i].joint_pos = list(q_cur)
            # grippers
            gstart = i * num_gripper
            gend = gstart + num_gripper
            g_cur = new_chunk.gripperPos[gstart:gend] if num_gripper > 0 else []
            if i < len(self.robot_command.trajectory):
                self.robot_command.trajectory[i].gripper_pos = list(g_cur)

            # end-effector poses (pos + quat). create Pose list if possible
            ee_list = []
            if num_ee > 0:
                for k in range(num_ee):
                    pstart = i * (num_ee * 3) + k * 3
                    pend = pstart + 3
                    qstart = i * (num_ee * 4) + k * 4
                    qend = qstart + 4
                    pos_slice = new_chunk.eePos[pstart:pend] if len(new_chunk.eePos) >= pend else []
                    quat_slice = new_chunk.eeQuat[qstart:qend] if len(new_chunk.eeQuat) >= qend else []
                    # fallback to zeros/default if missing
                    if len(pos_slice) != 3:
                        pos_slice = [0.0, 0.0, 0.0]
                    if len(quat_slice) != 4:
                        quat_slice = [1.0, 0.0, 0.0, 0.0]
                    ee_list.append(Pose(pos=pos_slice, quat=quat_slice))
            if i < len(self.robot_command.trajectory):
                self.robot_command.trajectory[i].ee_pose = ee_list

    def process_subscribe_command(self):
        """Process incoming LCM messages (non-blocking)."""
        new_command = False
        if self.lcm_instance:
            self.lcm_instance.handle_timeout(self.lcm_timeout)
            if self.seq != self.current_ACT_command.seq:
                self._process_trajectory()
                new_command = True
        return self.robot_command, new_command

    def process_publish_robot_state(self, robot_state):
        """Handle LCM feedback publishing."""
        if not self.connected:
            return

        # Handle robot feedback requests
        if self.check_robot_feedback_request():
            self.publish_robot_feedback(robot_state)

        # Handle state feedback requests
        if self.check_state_feedback_request():
            self.publish_state_feedback()

    def process_publish_trajgen_state(self, trajgen_state):
        """Publish trajgen state to channel."""
        if trajgen_state:
            self.act_status_index = state_feedback.kSuccess
            self.publish_state_feedback()
