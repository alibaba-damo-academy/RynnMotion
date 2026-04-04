"""Generic state machine for robotic systems."""

from statemachine import StateMachine, State
import sys
import time
from collections import OrderedDict
import logging
from RynnMotion.common.data.robot_state import RobotState
from RynnMotion.common.data.robot_command import RobotCommand

REGISTERED_FSM_FACTORY_FUNCS = OrderedDict()


def register_fsm_factory_func(fsm_name):
    """
    Function decorator to register algo generator functions that map generator class names.
    Each generator implements such a function, and decorates it with this decorator.

    Args:
        generator_name (str): the algorithm name to register the algorithm under
    """

    def decorator(factory_func):
        REGISTERED_FSM_FACTORY_FUNCS[fsm_name] = factory_func
        return factory_func

    return decorator


def fsm_factory(
    fsm_name,
    fsm_config,
    robot_model,
    communicator,
    generator,
    robot_interface,
    logger=None,
):
    """
    Factory function for creating trajectory generator based on the generator name and config.

    Args:
        generator_name (str): the generator name

        config (BaseConfig instance): config object
    """

    #
    fsm_class = REGISTERED_FSM_FACTORY_FUNCS[fsm_name]

    # create algo instance
    return fsm_class(
        fsm_config, robot_model, communicator, generator, robot_interface, logger
    )


@register_fsm_factory_func("basic_fsm")
def fsm_config_to_class(
    fsm_config: dict, robot_model, communicator, generator, robot_interface, logger=None
):
    """
    Maps algo config to the CQL algo class to instantiate, along with additional algo kwargs.

    Args:
        algo_config (Config instance): algo config

    Returns:
        algo_class: subclass of Algo
        algo_kwargs (dict): dictionary of additional kwargs to pass to algorithm
    """
    return RobotStateMachine(
        fsm_config, robot_model, communicator, generator, robot_interface, logger
    )


class RobotStateMachine(StateMachine):
    """Generic state machine for robotic control systems."""

    # Define states
    init = State("Init", initial=True)
    standby = State("Standby")
    running = State("Running")
    reset = State("Reset")
    error = State("Error")

    # Define transitions
    initialize = init.to(reset)
    reset_success = reset.to(standby)
    start_running = standby.to(running)
    stop_running = running.to(standby)
    request_reset = standby.to(reset) | running.to(reset)
    checkout_error = running.to(error) | standby.to(error)
    clear_error = error.to(reset)

    def __init__(
        self, fsm_config, robot_model, communicator, generator, robot_interface, logger
    ):
        self.logger = logger or logging.getLogger(__name__)
        super().__init__()
        self.fsm_timer = 0.0
        self.standby_max_count = 1000

        self.robot_model = robot_model
        self.communicator = communicator
        self.trajgen = generator
        self.robot_interface = robot_interface

        self._init_standby_positions()

    def _init_standby_positions(self):
        """Initialize robot standby positions."""
        # Here you would implement the logic to move the robot to its standby position.
        actuator_dofs = self.robot_model.get_actuator_num()
        gripper_dofs = self.robot_model.get_gripper_num()
        ee_num = self.robot_model.get_ee_num() or self.robot_model.get_ee_site_num()
        sites_num = self.robot_model.get_site_num()
        self.standby_positions = RobotCommand()
        self.standby_positions.init_from_dofs(
            num_joints=int(actuator_dofs),
            num_grippers=int(gripper_dofs),
            num_end_effectors=int(ee_num),
            num_ft_sensors=int(0),
            num_sites=int(sites_num),
            chunk_size=int(1),
        )
        standby_joint_pos = self.robot_model.get_full_q_standby(
            0
        )  # get_q_standby only motion dofs
        standby_dofs = len(standby_joint_pos)
        self.standby_positions.work_mode = 0
        self.standby_positions.seq = 0
        self.standby_positions.trajectory[0].joint_pos[
            :standby_dofs
        ] = standby_joint_pos.tolist()
        self.logger.info(f"get_qStandby: {standby_joint_pos}")

    def clear_fsm_timer(self):
        self.fsm_timer = 0

    def on_enter_init(self):
        self.clear_fsm_timer()
        self.logger.info("Entering INIT state - Robot initializing...")
        sys.stdout.flush()

    def on_enter_standby(self):
        self.clear_fsm_timer()
        self.trajgen.process_parameters_update("goto_standby")
        self.logger.info("Entering STANDBY state - go to standby position")
        sys.stdout.flush()

    def on_enter_running(self):
        self.clear_fsm_timer()
        self.trajgen.process_parameters_update("run")
        self.logger.info("Entering RUNNING state - System active")
        sys.stdout.flush()

    def on_enter_reset(self):
        self.clear_fsm_timer()
        self.logger.info("Entering RESET state - get initial state and reset modules")
        sys.stdout.flush()

    def on_enter_error(self):
        self.clear_fsm_timer()
        self.logger.info("Entering ERROR state")
        sys.stdout.flush()

    def init_step_process(self):
        """Process during init state."""
        if not self.communicator.is_connected():
            self.communicator.connect()
        if not self.robot_interface.is_connected():
            self.robot_interface.connect()
        if self.communicator.is_connected() and self.robot_interface.is_connected():
            self.initialize()
        else:
            if self.fsm_timer % 1000 == 0:
                self.logger.info(
                    "Waiting for communicator and robot interface to connect..."
                )
                print(
                    "\r\033[KWaiting for communicator and robot interface to connect..."
                )

    def reset_step_process(self):
        """Process during init state."""
        if self.communicator.is_connected() and self.robot_interface.is_connected():
            # if all connected, reset trajgen and initialize
            robot_state = self.robot_interface.get_robot_state_feedbacks()
            if robot_state is not None:
                self.trajgen.reset_trajectory_state(robot_state)
                self.reset_success()
            else:
                if self.fsm_timer % 1000 == 0:
                    self.logger.info("Waiting for robot state feedback...")
                    print("\r\033[KWaiting for robot state feedback...")
        else:
            if self.fsm_timer % 1000 == 0:
                self.logger.info(
                    "Waiting for communicator and robot interface to connect..."
                )
                print(
                    "\r\033[KWaiting for communicator and robot interface to connect..."
                )

    def gostandby_step_process(self):
        """Process during standby state."""
        robot_state = self.robot_interface.get_robot_state_feedbacks()
        latest_command = self.standby_positions
        self.trajgen.process_input_command(latest_command, True)
        robot_action, traj_complete = self.trajgen.update_trajectory()
        self.robot_interface.set_robot_command(robot_action)
        self.robot_interface.step()
        self.communicator.process_publish_robot_state(robot_state)

        # go standby success if the trajectory genenrator feedback is complete
        task_command = self.communicator.process_task_command()
        if task_command is not None:
            if traj_complete and (task_command != "go_standby"):
                self.start_running()
        else:
            self.start_running()

    def running_step_process(self):
        """Process during running state."""
        robot_state = self.robot_interface.get_robot_state_feedbacks()
        latest_command, new_command_flag = self.communicator.process_subscribe_command()
        self.trajgen.process_input_command(latest_command, new_command_flag)
        robot_action, traj_complete = self.trajgen.update_trajectory()
        self.robot_interface.set_robot_command(robot_action)
        self.robot_interface.step()
        self.communicator.process_publish_robot_state(robot_state)

        task_command = self.communicator.process_task_command()
        if task_command is not None:
            if task_command == "go_standby":
                self.stop_running()

    def update(self):
        """
        This function is called periodically by the main loop.
        It is used to update the state machine.
        """
        self.fsm_timer = self.fsm_timer + 1

        match self.current_state:
            case self.init:
                self.init_step_process()
            case self.reset:
                self.reset_step_process()
            case self.standby:
                self.gostandby_step_process()
            case self.running:
                self.running_step_process()
