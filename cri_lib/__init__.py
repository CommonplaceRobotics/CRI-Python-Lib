"""
.. include:: ../README.md
"""

from .cri_controller import CRIClient, CRIController, MotionType
from .cri_errors import (
    CRICommandError,
    CRICommandTimeOutError,
    CRIConnectionError,
    CRIError,
)
from .cri_protocol_parser import CRIProtocolParser
from .robot_state import (
    ErrorStates,
    JointsState,
    KinematicsState,
    OperationInfo,
    OperationMode,
    PlatformCartesianPosition,
    PosVariable,
    ReferencingAxisState,
    ReferencingState,
    ReplayMode,
    RobotCartesianPosition,
    RobotMode,
    RobotState,
    RunState,
)
