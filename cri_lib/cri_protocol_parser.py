import logging
from threading import Lock
from typing import Any, Sequence

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

logger = logging.getLogger(__name__)


class CRIProtocolParser:
    """Class handling the parsing of CRI messages to the robot state."""

    def __init__(self, robot_state: RobotState, robot_state_lock: Lock, program_list : list, program_list_lock: Lock):
        self.robot_state = robot_state
        self.robot_state_lock = robot_state_lock
        self.program_list = program_list
        self.program_list_lock = program_list_lock

    def parse_message(
        self, message: str
    ) -> dict[str, str] | dict[str, str | None] | None:
        """Parses a message to the RobotState of the class.

        Parameters
        ----------
        message: str
            Message to be parsed including `CRISTART` and `CRIEND`

        Returns
        -------
        None | dict[str, str]
            None if no Notification in necessary or
            a dict indicating which answer event to notify (key: "answer") and optionally an error message (key: "error")
        """
        parts = message.split()

        match parts[2]:
            case "STATUS":
                self._parse_status(parts[3:-1])
                return {"answer": "status"}

            case "RUNSTATE":
                self._parse_runstate(parts[3:-1])
                return None

            case "CYCLESTAT":
                self._parse_cyclestat(parts[3:-1])
                return None

            case "GRIPPERSTATE":
                self._parse_gripperstate(parts[3:-1])
                return None

            case "VARIABLES":
                self._parse_variables(parts[3:-1])
                return None

            case "OPINFO":
                self._parse_opinfo(parts[3:-1])
                return None

            case "CMD":
                return {"answer": self._parse_cmd(parts[3:-1])}

            case "MESSAGE":
                self._parse_message_message(parts[3:-1])
                return None

            case "CONFIG":
                self._parse_config(parts[3:-1])
                return None

            case "CANBridge":
                return self._parse_can_bridge(parts[3:-1])

            case "CMDACK":
                return {"answer": parts[3]}

            case "CMDERROR":
                return self._parse_cmderror(parts[3:-1])

            case "INFO":
                if (answer := self._parse_info(parts[3:-1])) is not None:
                    return {"answer": answer}
                else:
                    return None

            case "EXECEND":
                return {"answer": "EXECEND"}

            case "EXECERROR":
                return self._parse_execerror(parts[3:-1])

            case _:
                logger.debug(
                    "Unknown message type %s received:\n%s", parts[2], " ".join(parts)
                )
                return None

    def _parse_status(self, parameters: list[str]) -> None:
        """
        Parses a state message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `STATUS` and `CRIEND`
        """
        segment_start_idx = 0

        while segment_start_idx < len(parameters):
            match parameters[segment_start_idx]:
                case "MODE":
                    with self.robot_state_lock:
                        self.robot_state.mode = RobotMode(
                            parameters[segment_start_idx + 1]
                        )
                    segment_start_idx += 2

                case "POSJOINTSETPOINT":
                    joints = []
                    for i in range(16):
                        joints.append(float(parameters[segment_start_idx + 1 + i]))

                    with self.robot_state_lock:
                        self.robot_state.joints_set_point = JointsState(*joints)
                    segment_start_idx += 17

                case "POSJOINTCURRENT":
                    joints = []
                    for i in range(16):
                        joints.append(float(parameters[segment_start_idx + 1 + i]))

                    with self.robot_state_lock:
                        self.robot_state.joints_current = JointsState(*joints)
                    segment_start_idx += 17

                case "POSCARTROBOT":
                    coords = []
                    for i in range(6):
                        coords.append(float(parameters[segment_start_idx + 1 + i]))

                    with self.robot_state_lock:
                        self.robot_state.position_robot = RobotCartesianPosition(
                            *coords
                        )
                    segment_start_idx += 7

                case "POSCARTPLATFORM" | "POSCARTPLATTFORM":
                    coords = []
                    for i in range(3):
                        coords.append(float(parameters[segment_start_idx + 1 + i]))

                    with self.robot_state_lock:
                        self.robot_state.position_platform = PlatformCartesianPosition(
                            *coords
                        )
                    segment_start_idx += 4

                case "OVERRIDE":
                    with self.robot_state_lock:
                        self.robot_state.override = float(
                            parameters[segment_start_idx + 1]
                        )
                    segment_start_idx += 2

                case "DIN":
                    value_int = int(parameters[segment_start_idx + 1], base=16)
                    with self.robot_state_lock:
                        for i in range(64):
                            self.robot_state.din[i] = value_int & (1 << i) != 0
                    segment_start_idx += 2

                case "DOUT":
                    value_int = int(parameters[segment_start_idx + 1], base=16)
                    with self.robot_state_lock:
                        for i in range(64):
                            self.robot_state.dout[i] = value_int & (1 << i) != 0
                    segment_start_idx += 2

                case "ESTOP":
                    val = int(parameters[segment_start_idx + 1])
                    with self.robot_state_lock:
                        self.robot_state.emergency_stop_ok = val == 1 or val == 3
                        self.robot_state.main_relay = val == 2 or val == 3
                    segment_start_idx += 2

                case "SUPPLY":
                    val = int(parameters[segment_start_idx + 1])
                    with self.robot_state_lock:
                        self.robot_state.supply_voltage = float(val) / 1000.0
                    segment_start_idx += 2

                case "CURRENTALL":
                    val = int(parameters[segment_start_idx + 1])
                    with self.robot_state_lock:
                        self.robot_state.current_total = float(val) / 1000.0
                    segment_start_idx += 2

                case "CURRENTJOINTS":
                    currents = []
                    for i in range(16):
                        currents.append(
                            float(int(parameters[segment_start_idx + 1 + i])) / 1000
                        )

                    with self.robot_state_lock:
                        self.robot_state.current_joints = currents
                    segment_start_idx += 17

                case "ERROR":
                    errors = []
                    self.robot_state.combined_axes_error = parameters[
                        segment_start_idx + 1
                    ]
                    for axis_idx in range(16):
                        value_int = int(
                            parameters[segment_start_idx + 2 + axis_idx], base=10
                        )
                        error_bits = []
                        for i in range(8):
                            error_bits.append(value_int & (1 << i) != 0)
                        errors.append(ErrorStates(*error_bits))
                    with self.robot_state_lock:
                        self.robot_state.error_states = errors
                    segment_start_idx += 18

                case "KINSTATE":
                    with self.robot_state_lock:
                        self.robot_state.kinematics_state = KinematicsState(
                            int(parameters[segment_start_idx + 1])
                        )
                    segment_start_idx += 2

                case "OPMODE":
                    with self.robot_state_lock:
                        self.robot_state.operation_mode = OperationMode(
                            int(parameters[segment_start_idx + 1])
                        )
                    segment_start_idx += 2

                case "CARTSPEED":
                    with self.robot_state_lock:
                        self.robot_state.cart_speed_mm_per_s = float(
                            parameters[segment_start_idx + 1]
                        )
                    segment_start_idx += 2

                case "GSIG":
                    value_int = int(parameters[segment_start_idx + 1], base=16)
                    with self.robot_state_lock:
                        for i in range(128):
                            self.robot_state.global_signals[i] = (
                                value_int & (1 << i) != 0
                            )
                    segment_start_idx += 2

                case "FRAMEROBOT":
                    with self.robot_state_lock:
                        self.robot_state.frame_name = parameters[segment_start_idx + 1]
                        coords = []
                        for i in range(6):
                            coords.append(float(parameters[segment_start_idx + 2 + i]))
                        self.robot_state.frame_position_current = (
                            RobotCartesianPosition(*coords)
                        )
                    segment_start_idx += 8

                case _:
                    logger.debug(
                        "Unknown segment in status message: %s",
                        parameters[segment_start_idx],
                    )
                    segment_start_idx += 1

    def _parse_runstate(self, parameters: list[str]) -> None:
        """
        Parses a runstate message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `RUNSTATE` and `CRIEND`
        """
        with self.robot_state_lock:
            if parameters[0] == "MAIN":
                self.robot_state.main_main_program = parameters[1]
                self.robot_state.main_current_program = parameters[2]
                self.robot_state.main_commands_count = int(parameters[3])
                self.robot_state.main_current_command = int(parameters[4])
                self.robot_state.main_runstate = RunState(int(parameters[5]))
                self.robot_state.main_replay_mode = ReplayMode(int(parameters[6]))
            elif parameters[0] == "LOGIC":
                self.robot_state.logic_main_program = parameters[1]
                self.robot_state.logic_current_program = parameters[2]
                self.robot_state.logic_commands_count = int(parameters[3])
                self.robot_state.logic_current_command = int(parameters[4])
                self.robot_state.logic_runstate = RunState(int(parameters[5]))
                self.robot_state.logic_replay_mode = ReplayMode(int(parameters[6]))

    def _parse_cyclestat(self, parameters: list[str]) -> None:
        """
        Parses a cyclestat message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `CYCLESTAT` and `CRIEND`
        """
        with self.robot_state_lock:
            self.robot_state.cycle_time = float(parameters[0])
            self.robot_state.workload = float(parameters[1])

    def _parse_gripperstate(self, parameters: list[str]) -> None:
        """
        Parses a gripperstate message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `GRIPPERSTATE` and `CRIEND`
        """
        with self.robot_state_lock:
            self.robot_state.gripper_state = float(parameters[0])

    def _parse_variables(self, parameters: Sequence[str]) -> None:
        """
        Parses a variables message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `VARIABLES` and `CRIEND`
        """
        variables: dict[str, float | PosVariable] = {}
        idx = 0

        while idx < len(parameters):
            if parameters[idx] == "ValueNrVariable":
                variables[parameters[idx + 1]] = float(parameters[idx + 2])
                idx += 3
            elif parameters[idx] == "ValuePosVariable":
                values = []
                for i in range(15):
                    values.append(float(parameters[idx + 2 + i]))
                variables[parameters[idx + 1]] = PosVariable(*values)
                idx += 17
            else:
                logger.debug(
                    "Unknown variable type in VARIABLES message: %s", parameters[idx]
                )
                idx += 1

        with self.robot_state_lock:
            self.robot_state.variabels = variables

    def _parse_opinfo(self, parameters: Sequence[str]) -> None:
        """
        Parses a opinfo message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `OPINFO` and `CRIEND`
        """
        values = []
        for i in range(7):
            values.append(int(parameters[i]))

        with self.robot_state_lock:
            self.robot_state.operation_info = OperationInfo(*values)

    def _parse_cmd(self, parameters: Sequence[str]) -> str | None:
        """
        Parses a cmd message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `CMD` and `CRIEND`

        Returns
        -------
        str:
            id of answer event
        """
        if parameters[0] == "Active":
            if parameters[1].lower() == "true":
                with self.robot_state_lock:
                    self.robot_state.active_control = True
                return "Active_true"
            elif parameters[1].lower() == "false":
                with self.robot_state_lock:
                    self.robot_state.active_control = False
                return "Active_false"
            else:
                logger.debug("Unknown Active state: %s", parameters[1])

        return None

    def _parse_message_message(self, parameters: Sequence[str]) -> None:
        """
        Parses a message message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `MESSAGE` and `CRIEND`
        """
        if parameters[0] == "RobotControl":
            if parameters[1] == "Version":
                with self.robot_state_lock:
                    self.robot_state.robot_control_version = parameters[2]

        elif (
            parameters[0] == "Configuration:"
            or parameters[0] == "Type:"
            or parameters[0] == "Gripper:"
        ):
            complete_message = " ".join(parameters)
            parts = self._split_quotes_aware(complete_message)

            config = None
            r_type = None
            gripper = None
            idx = 0
            while idx < len(parts):
                if parts[idx] == "Configuration:":
                    config = parts[idx + 1]
                    idx += 2
                elif parts[idx] == "Type:":
                    r_type = parts[idx + 1]
                    idx += 2
                elif parts[idx] == "Gripper:":
                    gripper = parts[idx + 1]
                    idx += 2
                else:
                    idx += 1

            with self.robot_state_lock:
                if config is not None:
                    self.robot_state.robot_configuration = config
                if r_type is not None:
                    self.robot_state.robot_type = r_type
                if gripper is not None:
                    self.robot_state.gripper_type = gripper

        else:
            logger.debug("MESSAGE: %s", " ".join(parameters))

    def _parse_can_bridge(self, parameters: Sequence[str]) -> dict[str, Any] | None:
        """Parses a can bridge message.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `CANBridge` and `CRIEND`

        Returns
        -------
        dict[str, any] | None
            dict with the following keys:
            - id: CAN id
            - length: length of CAN packet
            - data: bytearray with can message
            - time: timestamp from CRI, 0 at the moment
            - system_time: system timestamp vrom CRI
        """
        if parameters[0] != "Msg":
            return None

        if parameters[1] != "ID":
            return None

        id = int(parameters[2])

        if parameters[3] != "Len":
            return None

        length = int(parameters[4])

        if parameters[5] != "Data":
            return None

        data = bytearray([int(i) for i in parameters[6:14]])

        if parameters[14] != "Time":
            return None

        time = int(parameters[15])

        if parameters[16] != "SystemTime":
            return None

        system_time = int(parameters[17])

        return {
            "answer": "CAN",
            "can": {
                "id": id,
                "length": length,
                "data": data,
                "time": time,
                "system_time": system_time,
            },
        }

    def _parse_config(self, parameters: Sequence[str]) -> None:
        """
        Parses a config message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `CONFIG` and `CRIEND`
        """
        if parameters[0] == "ProjectFile":
            with self.robot_state_lock:
                self.robot_state.project_file = parameters[1]

    def _parse_cmderror(self, parameters: Sequence[str]) -> dict[str, str]:
        """Parses a CMDERROR message to notify calling function

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `CMDERROR` and `CRIEND`

        Returns
        -------
        dict[str, str]
            A dict indicating which message id to notify (key: "answer") and the error message (key: "error")
        """

        return {"answer": parameters[0], "error": " ".join(parameters[1:])}

    def _parse_info(self, parameters: list[str]) -> str | None:
        """
        Parses a info message to the robot state.

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `INFO` and `CRIEND`
        """
        if parameters[0] == "ReferencingInfo":
            # handle bug in RobotControl with missing space before 'Mandatory'
            if len(parameters) < 20:
                parameters.insert(14, parameters[14][0])

            ref_state = ReferencingState(
                global_state=ReferencingAxisState(int(parameters[1])),
                A1=ReferencingAxisState(int(parameters[3])),
                A2=ReferencingAxisState(int(parameters[4])),
                A3=ReferencingAxisState(int(parameters[5])),
                A4=ReferencingAxisState(int(parameters[6])),
                A5=ReferencingAxisState(int(parameters[7])),
                A6=ReferencingAxisState(int(parameters[8])),
                E1=ReferencingAxisState(int(parameters[9])),
                E2=ReferencingAxisState(int(parameters[10])),
                E3=ReferencingAxisState(int(parameters[11])),
                E4=ReferencingAxisState(int(parameters[12])),
                E5=ReferencingAxisState(int(parameters[13])),
                E6=ReferencingAxisState(int(parameters[14])),
                mandatory=bool(parameters[16] == "1"),
                ref_prog_enabled=bool(parameters[18] == "1"),
                ref_prog_running=bool(parameters[19] == "1"),
            )

            with self.robot_state_lock:
                self.robot_state.referencing_state = ref_state

            return "info_referencing"
        elif parameters[0] == "BoardTemp":
            temperatures = [float(param) for param in parameters[1:]]

            with self.robot_state_lock:
                self.robot_state.board_temps = temperatures

            return "info_boardtemp"
        elif parameters[0] == "MotorTemp":
            temperatures = [float(param) for param in parameters[1:]]

            with self.robot_state_lock:
                self.robot_state.motor_temps = temperatures

            return "info_motortemp"
        elif parameters[0] == "FileList":
            with self.program_list_lock:
                self.program_list.clear() # direct point to parameters[] will break the shared reference
                self.program_list.extend(parameters[2:]) # first element is the target_folder
                print(self.program_list)
            return "info_flielist"
        
        else:
            return None

    def _parse_execerror(self, parameters: Sequence[str]) -> dict[str, str]:
        """Parses a EXECERROR message to notify calling function

        Parameters
        ----------
        parameters: list[str]
            List of splitted strings between `EXECERROR` and `CRIEND`

        Returns
        -------
        dict[str, str]
            A dict indicating which message id to notify (key: "answer") and the error message (key: "error")
        """

        return {"answer": "EXECEND", "error": " ".join(parameters)}

    @staticmethod
    def _split_quotes_aware(msg: str) -> Sequence[str]:
        """
        Splits a string at whitespaces but ignores whitespace whithin quotes.

        Parameters
        ----------
        msg: str
            String to be splitted

        Returns
        -------
        list[str]
            list containing the splitted parts of the string
        """
        parts = []
        current_part = ""
        last_was_whitespace = True
        in_quotes = False
        for char in msg:
            if char.isspace():
                if not last_was_whitespace and not in_quotes:
                    parts.append(current_part)
                    current_part = ""
                elif in_quotes:
                    current_part = current_part + char

                last_was_whitespace = True
            elif char == '"':
                if in_quotes:
                    parts.append(current_part)
                    current_part = ""
                in_quotes = not in_quotes
                last_was_whitespace = True
            else:
                current_part = current_part + char
                last_was_whitespace = False
        parts.append(current_part)

        return parts
