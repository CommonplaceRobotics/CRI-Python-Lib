from dataclasses import dataclass, field
from enum import Enum


class RobotMode(Enum):
    """Enum of possible robot modes for jogging, `FSM` does not support jogging."""

    JOINT = "joint"
    CARTBASE = "cartbase"
    CARTTOOL = "carttool"
    PLATFORM = "platform"
    FSM = "fsm"


class KinematicsState(Enum):
    """Enum of possible states of kinematics"""

    NO_ERROR = 0
    JOINT_MIN = 13
    JOINT_MAX = 14
    SIGULARITY_CENTER = 21
    SIGULARITY_REACH = 23
    SIGULARITY_WRIST = 24
    VIRTUAL_BOX0 = 30
    VIRTUAL_BOX1 = 31
    VIRTUAL_BOX2 = 32
    VIRTUAL_BOX3 = 33
    VIRTUAL_BOX4 = 34
    VIRTUAL_BOX5 = 35
    UNKNOWN = 98
    MOTION_NOT_ALLOWED = 99


class OperationMode(Enum):
    """Enum of possible operartion modes"""

    NOT_ENABLED = -1
    NORMAL = 0
    MANUAL = 1
    MOTION_NOT_ALLOWED = 2


class RunState(Enum):
    """Enum of possible run states"""

    STOPPED = 0
    PAUSED = 1
    RUNNING = 2


class ReplayMode(Enum):
    """Enum of possible replay modes"""

    SINGLE = 0
    REPEAT = 1
    STEP = 2
    FAST = 3


@dataclass
class ErrorStates:
    """error states of axes, multiple errors can apply"""

    over_temp: bool = False
    estop_lowv: bool = False
    motor_not_enabled: bool = False
    com: bool = False
    position_lag: bool = False
    ENC: bool = False
    overcurrent: bool = False
    driver: bool = False


@dataclass
class RobotCartesianPosition:
    """Represents the cartesian position of a robot"""

    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0
    A: float = 0.0
    B: float = 0.0
    C: float = 0.0


@dataclass
class PlatformCartesianPosition:
    """Represents the cartesian position of a platform"""

    X: float = 0.0
    Y: float = 0.0
    RZ: float = 0.0


@dataclass
class JointsState:
    """Represents the joints state of a robot"""

    A1: float = 0.0
    A2: float = 0.0
    A3: float = 0.0
    A4: float = 0.0
    A5: float = 0.0
    A6: float = 0.0
    E1: float = 0.0
    E2: float = 0.0
    E3: float = 0.0
    G1: float = 0.0
    G2: float = 0.0
    G3: float = 0.0
    P1: float = 0.0
    P2: float = 0.0
    P3: float = 0.0
    P4: float = 0.0


@dataclass
class PosVariable:
    """Represents a position variable"""

    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0
    A: float = 0.0
    B: float = 0.0
    C: float = 0.0
    A1: float = 0.0
    A2: float = 0.0
    A3: float = 0.0
    A4: float = 0.0
    A5: float = 0.0
    A6: float = 0.0
    E1: float = 0.0
    E2: float = 0.0
    E3: float = 0.0


@dataclass
class OperationInfo:
    """Operation statistics sent by the robot controler"""

    program_starts_total: int = 0
    up_time_complete: float = 0.0
    up_time_enabled: float = 0.0
    up_time_motion: float = 0.0
    up_time_last: float = 0.0
    last_programm_duration: int = 0
    num_program_starts_since_startup: int = 0


class ReferencingAxisState(Enum):
    """Enum of possible referencing states of an axis"""

    NOT_REFERENCED = 0
    REFERENCED = 1
    REFERENCING = 2


@dataclass
class ReferencingState:
    """Represents the overall referencing state of the robot."""

    global_state: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    mandatory: bool = True
    ref_prog_enabled: bool = False
    ref_prog_running: bool = False

    A1: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    A2: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    A3: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    A4: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    A5: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    A6: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    E1: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    E2: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    E3: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    E4: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    E5: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED
    E6: ReferencingAxisState = ReferencingAxisState.NOT_REFERENCED


@dataclass
class RobotState:
    """
    Dataclass which holds the current state of the robot.
    """

    mode: RobotMode = RobotMode.JOINT
    """Robot execution mode"""

    joints_set_point: JointsState = field(default_factory=JointsState)
    """Set point of robot joints"""

    joints_current: JointsState = field(default_factory=JointsState)
    """Actual values of robot joints"""

    position_robot: RobotCartesianPosition = field(
        default_factory=RobotCartesianPosition
    )
    """Current cartesian position of robot"""

    position_platform: PlatformCartesianPosition = field(
        default_factory=PlatformCartesianPosition
    )
    """Current cartesian position of platform"""

    cart_speed_mm_per_s: float = 0.0
    """current speed of the cart in mm/s"""

    override: float = 100.0
    """global robot speed override"""

    din: list[bool] = field(default_factory=lambda: [False] * 64)
    """digital ins"""

    dout: list[bool] = field(default_factory=lambda: [False] * 64)
    """digital outs"""

    emergency_stop_ok = False
    """`True` if emergency stop circuit is closed"""

    main_relay = False
    """`True` if main power relay is closed"""

    supply_voltage: float = 0.0
    """current supply voltage"""

    battery_percent: float = 0.0
    """battery percent of mobile platform"""

    current_total = 0.0
    """total current drawn by robot"""

    current_joints: list[float] = field(default_factory=lambda: [0.0] * 16)
    """current drawn by individual axes"""

    kinematics_state: KinematicsState = KinematicsState.MOTION_NOT_ALLOWED
    """global kinematics state"""

    operation_mode: OperationMode = OperationMode.NOT_ENABLED
    """global operation mode"""

    global_signals: list[bool] = field(default_factory=lambda: [False] * 128)
    """global signals"""

    frame_name: str = ""
    """name of currently active frame"""

    frame_position_current: RobotCartesianPosition = field(
        default_factory=RobotCartesianPosition
    )
    """position in currently active frame"""

    main_main_program: str = ""
    """main program of main interpreter"""

    main_current_program: str = ""
    """currently active program of main interpreter, can be different than main_main_program in case of sub programm"""

    logic_main_program: str = ""
    """main programm of logic interpreter"""

    logic_current_program: str = ""
    """currently active program of logic interpreter, can be different than logic_main_program in case of sub programm"""

    main_commands_count: int = 0
    """total number of commands in main program"""

    logic_commands_count: int = 0
    """total number of commands in logic program"""

    main_current_command: int = 0
    """index of currently executed command in main interpreter"""

    logic_current_command: int = 0
    """index of currently executed command in logic interpreter"""

    main_runstate: RunState = RunState.STOPPED
    """runstate of main interpreter"""

    logic_runstate: RunState = RunState.STOPPED
    """runstate of logic interpreter"""

    main_replay_mode: ReplayMode = ReplayMode.SINGLE
    """replay mode of main interpreter"""

    logic_replay_mode: ReplayMode = ReplayMode.SINGLE
    """replay mode of logic interpreter"""

    error_states: list[ErrorStates] = field(
        default_factory=lambda: [ErrorStates()] * 16
    )
    """error states of individual axes"""

    combined_axes_error: str = "_not_ready"
    """combined error state of all axes as string"""

    cycle_time: float = 0.0
    """cycle time of robot control loop"""

    workload: float = 0.0
    """workload of robot control cpu"""

    gripper_state: float = 0.0
    """current opening value of the gripper"""

    variabels: dict[str : [PosVariable | float]] = field(default_factory=dict)
    """variables saved in robot controller"""

    operation_info: OperationInfo = field(default_factory=OperationInfo)
    """operation statistics of robot controller"""

    active_control: bool = False
    """indicates whether the connection has active control of the robot"""

    robot_control_version: str = ""
    """version of robot control software"""

    robot_configuration: str = ""
    """configuration of robot"""

    robot_type: str = ""
    """type of robot"""

    gripper_type: str = ""
    """type of gripper"""

    project_file: str = ""
    """currently active project file"""

    referencing_state: ReferencingState = field(default_factory=ReferencingState)
    """individual referencing state of all axes"""

    board_temps: list[float] = field(default_factory=lambda: [0.0] * 16)
    """Temperatures of motor controller PCBs"""

    motor_temps: list[float] = field(default_factory=lambda: [0.0] * 16)
    """Temperatures of motors"""
