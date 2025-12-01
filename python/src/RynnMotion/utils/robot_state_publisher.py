import numpy as np
import time
from queue import Queue
from collections import deque
from threading import Thread, Lock
import lcm
from RynnMotion.common.lcm.lcmMotion.robot_observation import robot_observation
from RynnMotion.common.data.robot_state import RobotState


class RobotStatePublisher:
    def __init__(self, robot_model, monitor_config):
        self.robot_model = robot_model
        self.control_freq = robot_model.get_robot_control_freq()
        self.dt = 1.0 / self.control_freq

        self.monitor_config = monitor_config
        self.enable_joint_monitor = self.monitor_config.get("enable_joint_pos", False)
        self.enable_eepose_monitor = self.monitor_config.get("enable_ee_pose", False)

        self.robot_joint_dim = self.robot_model.get_actuator_num()
        self.robot_ee_num = self.robot_model.get_ee_num()
        self.robot_state = RobotState()
        self.robot_action = RobotState()

        # Flag to control monitoring
        self.running = False

        self.init_lcm_topic()

    def init_lcm_topic(self):
        """Initialize LCM connection and subscribe to channels."""
        try:
            self.lcm_instance = lcm.LCM()
            print(f"robot monitor setup successful!")
        except Exception as e:
            print(f"robot monitor setup failed: {e}")
            self.lcm_instance = None

    def start(self):
        """Start monitoring - call this once at the beginning"""
        if self.running:
            return
        self.running = True
        print("robot monitor Started")

    def stop(self):
        """Stop monitoring"""
        self.running = False
        print("robot monitor thead stopped!")

    def publish_robot_state(self, robot_state):
        """Send state feedback message."""
        if not self.lcm_instance:
            return

        msg = robot_observation()
        msg.sec = int(time.time())
        msg.nanosec = int((time.time() % 1) * 1e9)
        msg.utime = int(time.time() * 1000000)

        msg.numJoint = self.robot_joint_dim
        msg.jointPos = robot_state.joint_pos
        msg.jointVel = robot_state.joint_vel
        msg.jointTorque = robot_state.joint_torque

        msg.numEE = self.robot_ee_num
        msg.eePose = [[0.0] * 7 for _ in range(self.robot_ee_num)]
        msg.eeVel = [[0.0] * 6 for _ in range(self.robot_ee_num)]
        for i in range(self.robot_ee_num):
            msg.eePose[i][:3] = robot_state.ee_pose[i].pos
            msg.eePose[i][3:] = robot_state.ee_pose[i].quat

        self.lcm_instance.publish("robot_state_monitor/robot_state", msg.encode())

    def publish_robot_action(self, robot_action):
        if not self.lcm_instance:
            return

        msg = robot_observation()
        msg.sec = int(time.time())
        msg.nanosec = int((time.time() % 1) * 1e9)
        msg.utime = int(time.time() * 1000000)

        msg.numJoint = self.robot_joint_dim
        msg.jointPos = robot_action.joint_pos
        msg.jointVel = robot_action.joint_vel
        msg.jointTorque = robot_action.joint_torque

        msg.numEE = self.robot_ee_num
        msg.eePose = [[0.0] * 7 for _ in range(self.robot_ee_num)]
        msg.eeVel = [[0.0] * 6 for _ in range(self.robot_ee_num)]
        for i in range(self.robot_ee_num):
            msg.eePose[i][:3] = robot_action.ee_pose[i].pos
            msg.eePose[i][3:] = robot_action.ee_pose[i].quat

        self.lcm_instance.publish("robot_state_monitor/robot_action", msg.encode())

    def update_robot_states(self, robot_state, robot_action):
        """
        Update all robot states. Call this in your main control loop.

        Args:
            robot_state: RobotState object containing current robot state
        """
        if not self.running:
            return

        # Update joint states
        self.robot_state = robot_state.copy()
        self.robot_action = robot_action.copy()
        self.publish_robot_state(robot_state)
        self.publish_robot_action(robot_action)

    def _run(self):
        try:

            while self.running:
                postcall_time = time.time()
                not_update = True
                self.update_robot_states(self.robot_state, self.robot_action)
                postcall_wall_time = time.time()
                sleep_time = self.dt - (postcall_wall_time - postcall_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            print(
                "robot monitor thread stopped! Can't use cv2 in this thead, Error:", e
            )
