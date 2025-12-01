"""Generic state machine for robotic systems."""

from statemachine import StateMachine, State
import sys
import time


class RobotStateMachine(StateMachine):
    """Generic state machine for robotic control systems."""

    # Define states
    init = State("Init", initial=True)
    standby = State("Standby")
    running = State("Running")
    reset = State("Reset")

    # Define transitions
    initialize = init.to(standby)
    start_running = standby.to(running)
    stop_running = running.to(standby)
    request_reset = standby.to(reset) | running.to(reset)
    finish_reset = reset.to(standby)

    def on_enter_init(self):
        print("\r\033[KEntering INIT state - Moving robots to initial positions")
        sys.stdout.flush()

    def on_enter_standby(self):
        print("\r\033[KEntering STANDBY state - System ready")
        sys.stdout.flush()

    def on_enter_running(self):
        print("\r\033[KEntering RUNNING state - System active")
        sys.stdout.flush()

    def on_enter_reset(self):
        print("\r\033[KEntering RESET state - Moving robots to initial positions")
        sys.stdout.flush()

    def on_exit_reset(self):
        print("\r\033[KExiting RESET state")
        sys.stdout.flush()


class PyFSM(StateMachine):
    """
    Python-based Finite State Machine for teleoperation control

    States:
    - Initr: Initial state, calculates standby positions and transitions immediately
    - GoStandby1: Lerp state, interpolates to standby positions over lerp_duration
    - DoAction1: Active teleoperation state
    - Reset: Reset state, recalculates standby positions and transitions immediately

    Transitions:
    - Initr → GoStandby1 (immediately after calculating positions)
    - GoStandby1 → DoAction1 (after lerp completes)
    - DoAction1 → Reset (on user request)
    - Reset → GoStandby1 (immediately after recalculating positions)
    """

    # State Definitions
    initr = State("Initr", initial=True)
    go_standby1 = State("GoStandby1")
    do_action1 = State("DoAction1")
    reset = State("Reset")

    # Transition Definitions
    to_go_standby1 = initr.to(go_standby1)
    to_do_action1 = go_standby1.to(do_action1)
    to_reset = do_action1.to(reset)
    finish_reset = reset.to(go_standby1)

    def __init__(self, verbose: bool = False, lerp_duration: float = 1.0):
        self.verbose = verbose  # Set before super().__init__() to avoid AttributeError
        self.lerp_duration = lerp_duration
        self.state_enter_time = None
        super().__init__()

    # Lifecycle Hooks
    def on_enter_initr(self):
        """Called when entering INITR state"""
        self.state_enter_time = time.time()
        if self.verbose:
            print("\r\033[KEntering INITR state - Initializing system")
            sys.stdout.flush()

    def on_enter_go_standby1(self):
        """Called when entering GOSTANDBY1 state"""
        self.state_enter_time = time.time()
        print("\r\033[KEntering GOSTANDBY1 state")
        sys.stdout.flush()

    def on_enter_do_action1(self):
        """Called when entering DOACTION1 state"""
        self.state_enter_time = time.time()
        print("\r\033[KEntering DOACTION1 state")
        sys.stdout.flush()

    def on_enter_reset(self):
        """Called when entering RESET state"""
        self.state_enter_time = time.time()
        if self.verbose:
            print("\r\033[KEntering RESET state - Resetting system")
        sys.stdout.flush()

    def on_exit_reset(self):
        """Called when exiting RESET state"""
        print("\r\033[KExiting RESET state")
        sys.stdout.flush()

    # Helper methods
    def get_elapsed_time(self):
        """Get elapsed time since entering current state"""
        if self.state_enter_time is None:
            return 0.0
        return time.time() - self.state_enter_time

    def should_auto_transition(self, duration=1.0):
        """Check if enough time has elapsed for auto-transition"""
        return self.get_elapsed_time() >= duration

    # State checkers
    def is_Initr(self):
        """Check if in Initr state"""
        return self.current_state == self.initr

    def is_GoStandby1(self):
        """Check if in GoStandby1 state"""
        return self.current_state == self.go_standby1

    def is_DoAction1(self):
        """Check if in DoAction1 state"""
        return self.current_state == self.do_action1

    def is_Reset(self):
        """Check if in Reset state"""
        return self.current_state == self.reset


class PolicyFSM(StateMachine):
    """state machine for robotic control systems with policy."""

    # Define states
    init = State("Init", initial=True)
    standby = State("Standby")
    running = State("Running")
    reset = State("Reset")
    error = State("Error")

    # Define transitions
    initialize = init.to(standby)
    start_running = standby.to(running)
    stop_running = running.to(standby)
    request_reset = standby.to(reset) | running.to(reset)
    finish_reset = reset.to(standby)
    checkout_error = running.to(error) | standby.to(error) | init.to(error)
    clear_error = error.to(reset)

    def __init__(self, update_rate):
        super().__init__()
        self.fsm_timer = 0.0
        self.standby_count = 0

    def on_enter_init(self):
        print("\r\033[KEntering INIT state - Moving robots to initial positions")
        sys.stdout.flush()

    def on_enter_standby(self):
        print("\r\033[KEntering STANDBY state - System ready")
        sys.stdout.flush()

    def on_enter_running(self):
        print("\r\033[KEntering RUNNING state - System active")
        sys.stdout.flush()

    def on_enter_reset(self):
        print("\r\033[KEntering RESET state - Moving robots to initial positions")
        sys.stdout.flush()

    def on_exit_reset(self):
        print("\r\033[KExiting RESET state")
        sys.stdout.flush()

    def on_enter_error(self):
        print("\r\033[KEntering ERROR state")
        sys.stdout.flush()

    def update(self, event):
        """
        This function is called periodically by the main loop.
        It is used to update the state machine.
        """
        self.fsm_timer = self.fsm_timer + 1
        match self.current_state:
            case "Init":
                self.current_state = "Standby"
            case "Standby":
                self.current_state = "motion"
            case "Running":
                self.on_enter_motion()
