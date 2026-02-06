import logging
import socket
import threading
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from queue import Empty, Queue
from time import sleep, time
from typing import Any, Callable, Literal

from .cri_errors import CRICommandTimeOutError, CRIConnectionError
from .cri_protocol_parser import CRIProtocolParser
from .robot_state import KinematicsState, RobotState

logger = logging.getLogger(__name__)


DEFAULT = object()
"""Placeholder for defaulting a parameter to runtime-configurable default values."""


class MotionType(Enum):
    """Robot Motion Type for Jogging"""

    Joint = "Joint"
    CartBase = "CartBase"
    CartTool = "CartTool"
    Platform = "Platform"


class CRIClient:
    """Client with implementations for read-only communication."""

    ALIVE_JOG_INTERVAL_SEC = 0.2
    RECEIVE_TIMEOUT_SEC = 5
    DEFAULT_ANSWER_TIMEOUT = 10.0

    def __init__(self) -> None:
        """Create a ``CRIClient`` without connecting it yet.

        Call ``connect`` to connect and start receiving data.
        """
        self.robot_state: RobotState = RobotState()
        self.robot_state_lock = threading.Lock()

        self.parser = CRIProtocolParser(self.robot_state, self.robot_state_lock)

        self.connected = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_write_lock = threading.Lock()

        self.can_mode: bool = False
        self.can_queue: Queue = Queue()

        self.jog_thread = threading.Thread(target=self._bg_alivejog_thread, daemon=True)
        self.jog_intervall = self.ALIVE_JOG_INTERVAL_SEC
        self.receive_thread = threading.Thread(
            target=self._bg_receive_thread, daemon=True
        )

        self.sent_command_counter_lock = threading.Lock()
        self.sent_command_counter = 0
        self.answer_events_lock = threading.Lock()
        self.answer_events: dict[str, threading.Event] = {}
        self.error_messages: dict[str, str] = {}

        self.status_callback: Callable | None = None

    def connect(
        self,
        host: str,
        port: int = 3920,
        application_name: str = "CRI-Python-Lib",
        application_version: str = "0-0-0-0",
    ) -> Literal[True]:
        """
        Connect to iRC.

        Parameters
        ----------
        host : str
            IP address or hostname of iRC
        port : int
            port of iRC
        application_name : str
            optional name of your application sent to controller
        application_version: str
            optional version of your application sent to controller

        Returns
        -------
        bool
            True if connected.
            Otherwise an exception is raised.

        Raises
        ------
        CRIConnectionError
            When already connected or connection fails.
        """
        if self.connected:
            raise CRIConnectionError("Already connected.")
        self.sock.settimeout(0.1)  # Set a timeout of 0.1 seconds
        try:
            ip = socket.gethostbyname(host)
            self.sock.connect((ip, port))
            # Mark as connected before starting threads
            self.connected = True

            # Start receiving commands
            self.receive_thread.start()

            # Start sending ALIVEJOG message
            self.jog_thread.start()

            hello_msg = f'INFO Hello "{application_name}" {application_version} {datetime.now(timezone.utc).strftime(format="%Y-%m-%dT%H:%M:%S")}'

            self._send_command(hello_msg)
            logger.debug("Connected to %s:%d", host, port)
            return True

        except ConnectionRefusedError:
            raise CRIConnectionError(
                f"Connection refused: Unable to connect to {host}:{port}"
            )
        except Exception as e:
            raise CRIConnectionError("Failed to connect to iRC.") from e

    def close(self) -> None:
        """
        Close network connection. Might block for a while waiting for the threads to finish.
        """

        if not self.connected or self.sock is None:
            return

        self._send_command("QUIT")

        self.connected = False

        if self.jog_thread.is_alive():
            self.jog_thread.join()

        if self.receive_thread.is_alive():
            self.receive_thread.join()

        self.sock.close()

    def _register_answer(self, answer_id: str) -> None:
        with self.answer_events_lock:
            self.answer_events[answer_id] = threading.Event()

    def _send_command(
        self,
        command: str,
        register_answer: bool = False,
        fixed_answer_name: str | None = None,
    ) -> int:
        """Sends the given command to iRC.

        The method is marked private because technically it can be used to send control commands as well.

        Parameters
        ----------
        command : str
            Command to be sent without `CRISTART`, counter and `CRIEND`

        Returns
        -------
        int
            The sent message_id.

        Raises
        ------
        CRIConnectionError
            When not connected or connection was lost.
        """
        if not self.connected or self.sock is None:
            logger.error("Not connected. Use connect() to establish a connection.")
            raise CRIConnectionError(
                "Not connected. Use connect() to establish a connection."
            )

        with self.sent_command_counter_lock:
            command_counter = self.sent_command_counter

            if self.sent_command_counter >= 9999:
                self.sent_command_counter = 1
            else:
                self.sent_command_counter += 1

        message = f"CRISTART {command_counter} {command} CRIEND"

        if register_answer:
            with self.answer_events_lock:
                if fixed_answer_name is not None:
                    self.answer_events[fixed_answer_name] = threading.Event()
                else:
                    self.answer_events[str(command_counter)] = threading.Event()

        try:
            with self.socket_write_lock:
                self.sock.sendall(message.encode())
            logger.debug("Sent command: %s", message)

            return command_counter

        except Exception as e:
            logger.exception("Failed to send command.")
            if register_answer:
                with self.answer_events_lock:
                    if fixed_answer_name is not None:
                        del self.answer_events[fixed_answer_name]
                    else:
                        del self.answer_events[str(command_counter)]
            self.connected = False
            raise CRIConnectionError("ConnectionLost")

    def _bg_alivejog_thread(self) -> None:
        """
        Background Thread sending alivejog messages to keep connection alive.
        """
        while self.connected:
            if self._send_command("ALIVEJOG 0 0 0 0 0 0 0 0 0") is None:
                logger.error("AliveJog Thread: Connection lost.")
                self.connected = False
                return

            sleep(self.jog_intervall)

    def _bg_receive_thread(self) -> None:
        """
        Background thread receiving data and parsing it to the robot state.
        """
        if self.sock is None:
            logger.error("Receive Thread: Not connected.")
            return

        message_buffer = bytearray()

        while self.connected:
            try:
                recv_buffer = self.sock.recv(4096)
            except TimeoutError:
                continue

            if recv_buffer == b"":
                self.connected = False
                logger.error("Receive Thread: Connection lost.")
                return

            message_buffer.extend(recv_buffer)

            continue_parsing = True
            while continue_parsing:
                # check for an end of message
                end_idx = message_buffer.find(b"CRIEND")
                if end_idx != -1:
                    start_idx = message_buffer.find(b"CRISTART")

                    # check if there is a complete message
                    if start_idx != -1:
                        message = message_buffer[start_idx : end_idx + 6].decode()
                        self._parse_message(message)

                    # check if there is data left in the buffer
                    if len(message_buffer) > end_idx + 7:
                        message_buffer = message_buffer[end_idx + 7 :]
                    else:
                        message_buffer.clear()
                else:
                    continue_parsing = False

    def _wait_for_answer(
        self,
        message_id: str | int,
        timeout: float | None = DEFAULT,  # type: ignore
    ) -> None | str:
        """Waits for an answer to a message.
        The answer event will be removed after the call, even if there was a timeout. Choose timeout accordingly.

        Parameters
        ----------
        message_id : int or str
            message id of sent message of which an answer is expected

        timeout : float | DEFAULT | None
            timeout for wait in seconds.
            - `DEFAULT` uses `self.DEFAULT_ANSWER_TIMEOUT`
            - `None` will wait indefinetly

        Returns
        -------
        None | str
            returns `None` if an answer was received with no error
            returns an error message if an `CMDERROR` was received

        Raises
        ------
        CRITimeoutError
            raised if no answer was received in given timeout

        """
        message_id = str(message_id)
        with self.answer_events_lock:
            if message_id not in self.answer_events:
                return None
            wait_event = self.answer_events[message_id]

        if timeout is DEFAULT:
            timeout = self.DEFAULT_ANSWER_TIMEOUT
        success = wait_event.wait(timeout=timeout)

        if not success:
            raise CRICommandTimeOutError()

        # prevent deadlock through answer_events_lock
        with self.answer_events_lock:
            del self.answer_events[message_id]

            if message_id in self.error_messages:
                error_msg = self.error_messages[message_id]
                del self.error_messages[message_id]
                return error_msg
            else:
                return None

    def _parse_message(self, message: str) -> None:
        """Internal function to parse a message. If an answer event is registered for a certain msg_id it is triggered."""
        if "STATUS" not in message:
            logger.debug("Received: %s", message)

        if (notification := self.parser.parse_message(message)) is not None:
            if notification["answer"] == "status" and self.status_callback is not None:
                self.status_callback(self.robot_state)

            if notification["answer"] == "CAN":
                self.can_queue.put_nowait(notification["can"])

            with self.answer_events_lock:
                msg_id = notification["answer"]

                if msg_id in self.answer_events:
                    if (error_msg := notification.get("error", None)) is not None:
                        self.error_messages[msg_id] = error_msg

                    self.answer_events[msg_id].set()

    def wait_for_status_update(self, timeout: float | None = None) -> None:
        """Wait for next STATUS message.

        Parameters
        ----------
        timeout : float | None
            Maximum wait time, infinite if `None`

        Raises
        ------
        CRITimeoutError
            raised if no status update was received in given timeout
        """
        self._register_answer("status")
        self._wait_for_answer("status", timeout)

    def register_status_callback(self, callback: Callable | None) -> None:
        """Register a callback which is called every time a STATUS message was parsed to the state.
        The callback must have the following definition:
        def callback(state: RobotState)
        Keep the callback as fast as possible as it will be excute by the receive thread and no messages will be processed, while is runs.
        Also keep thread safety in mind, as the callback will be excuted by the receive thread.

        Parameters
        ----------
        callback : Callable
            callback function to be called, pass `None` to deregister a callback
        """
        self.status_callback = callback

    def wait_for_kinematics_ready(self, timeout: float = 30) -> bool:
        """Wait until drive state is indicated as ready.

        Parameters
        ----------
        timeout : float
            maximum time to wait in seconds

        Returns
        -------
        bool
            `True`if drives are ready, `False` if not ready or timeout
        """
        start_time = time()
        new_timeout = timeout
        while new_timeout > 0.0:
            self.wait_for_status_update(timeout=new_timeout)
            if (self.robot_state.kinematics_state == KinematicsState.NO_ERROR) and (
                self.robot_state.combined_axes_error == "NoError"
            ):
                return True

            new_timeout = timeout - (time() - start_time)

        return False

    def get_board_temperatures(
        self,
        blocking: bool = True,
        timeout: float | None = DEFAULT,  # type: ignore
    ) -> bool:
        """Receive motor controller PCB temperatures and save in robot state

        Parameters
        ----------
        blocking: bool
            wait for response, always returns True if not waiting

        timeout: float | None
            timeout for waiting in seconds or None for infinite waiting
        """
        self._send_command("SYSTEM GetBoardTemp", True, "info_boardtemp")
        if (
            error_msg := self._wait_for_answer("info_boardtemp", timeout=timeout)
        ) is not None:
            logger.debug("Error in GetBoardTemp command: %s", error_msg)
            return False
        else:
            return True

    def get_motor_temperatures(
        self,
        blocking: bool = True,
        timeout: float | None = DEFAULT,  # type: ignore
    ) -> bool:
        """Receive motor temperatures and save in robot state

        Parameters
        ----------
        blocking: bool
            wait for response, always returns True if not waiting

        timeout: float | None
            timeout for waiting in seconds or None for infinite waiting
        """
        self._send_command("SYSTEM GetMotorTemp", True, "info_motortemp")
        if (
            error_msg := self._wait_for_answer("info_motortemp", timeout=timeout)
        ) is not None:
            logger.debug("Error in GetMotorTemp command: %s", error_msg)
            return False
        else:
            return True


class CRIController(CRIClient):
    """A connected ``CRIClient`` with control capabilities."""

    ACTIVE_JOG_INTERVAL_SEC = 0.02

    def __init__(self) -> None:
        self.live_jog_active: bool = False
        self.jog_intervall = self.ALIVE_JOG_INTERVAL_SEC
        self.jog_speeds: dict[str, float] = {
            "A1": 0.0,
            "A2": 0.0,
            "A3": 0.0,
            "A4": 0.0,
            "A5": 0.0,
            "A6": 0.0,
            "E1": 0.0,
            "E2": 0.0,
            "E3": 0.0,
        }
        self.jog_speeds_lock = threading.Lock()
        super().__init__()

    def _bg_alivejog_thread(self) -> None:
        """Overrides the ``CRIClient._bg_alivejog_thread`` to send alivejog messages with possibly nonzero jog speeds."""
        while self.connected:
            if self.live_jog_active:
                with self.jog_speeds_lock:
                    command = f"ALIVEJOG {self.jog_speeds['A1']} {self.jog_speeds['A2']} {self.jog_speeds['A3']} {self.jog_speeds['A4']} {self.jog_speeds['A5']} {self.jog_speeds['A6']} {self.jog_speeds['E1']} {self.jog_speeds['E2']} {self.jog_speeds['E3']}"
            else:
                command = "ALIVEJOG 0 0 0 0 0 0 0 0 0"

            if self._send_command(command) is None:
                logger.error("AliveJog Thread: Connection lost.")
                self.connected = False
                return

            sleep(self.jog_intervall)

    def send_command(self, command, register_answer=False, fixed_answer_name=None):
        """Wraps the superclass method to make it public."""
        return super()._send_command(command, register_answer, fixed_answer_name)

    def reset(self) -> bool:
        """Reset robot

        Returns
        -------
        bool:
            `True` if request was successful
            `False` if request was not successful
        """
        msg_id = self._send_command("CMD Reset", True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in RESET command: %s", error_msg)
            return False
        else:
            return True

    def enable(self) -> bool:
        """Enable robot
           An potential error message received from the robot will be logged with priority DEBUG

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        msg_id = self._send_command("CMD Enable", True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in ENABLE command: %s", error_msg)
            return False
        else:
            return True

    def disable(self) -> bool:
        """Disable robot

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        msg_id = self._send_command("CMD Disable", True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in DISABLE command: %s", error_msg)
            return False
        else:
            return True

    def set_active_control(self, active: bool) -> bool:
        """Acquire or return active control of robot

        Parameters
        ----------
        active : bool
            `True` acquire active control
            `False` return active control
        """
        self._send_command(
            f"CMD SetActive {str(active).lower()}",
            True,
            f"Active_{str(active).lower()}",
        )
        if (
            error_msg := self._wait_for_answer(f"Active_{str(active).lower()}")
        ) is not None:
            logger.debug("Error in set active control command: %s", error_msg)
            return False
        else:
            return True

    def zero_all_joints(self) -> bool:
        """Set all joints to zero

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        msg_id = self._send_command("CMD SetJointsToZero", True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in SetJointsToZero command: %s", error_msg)
            return False
        else:
            return True

    def reference_all_joints(self, *, timeout: float = 30) -> bool:
        """Reference all joints. Long timout of 30 seconds.

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        msg_id = self._send_command("CMD ReferenceAllJoints", True)
        if (error_msg := self._wait_for_answer(msg_id, timeout=timeout)) is not None:
            logger.debug("Error in ReferenceAllJoints command: %s", error_msg)
            return False
        else:
            return True

    def reference_single_joint(self, joint: str, *, timeout: float = 30) -> bool:
        """Reference a single joint. Long timout of 30 seconds.

        Parameters
        ----------
        joint : str
            joint name with either 'A', 'E', 'T' or 'P' as first character and an corresponding index as second

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """

        if (
            joint[0] == "A" or joint[0] == "E" or joint[0] == "T" or joint[0] == "P"
        ) and (int(joint[1]) > 0):
            joint_msg = joint[0] + str(int(joint[1]) - 1)
        else:
            return False

        msg_id = self._send_command(f"CMD ReferenceSingleJoint {joint_msg}", True)
        if (error_msg := self._wait_for_answer(msg_id, timeout=timeout)) is not None:
            logger.debug("Error in ReferenceSingleJoint command: %s", error_msg)
            return False
        else:
            return True

    def get_referencing_info(self):
        """Reference all joints. Long timout of 30 seconds.

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        self._send_command("CMD GetReferencingInfo", True, "info_referencing")
        if (error_msg := self._wait_for_answer("info_referencing")) is not None:
            logger.debug("Error in GetReferencingInfo command: %s", error_msg)
            return False
        else:
            return True

    def move_joints(
        self,
        A1: float,
        A2: float,
        A3: float,
        A4: float,
        A5: float,
        A6: float,
        E1: float,
        E2: float,
        E3: float,
        velocity: float,
        wait_move_finished: bool = False,
        move_finished_timeout: float | None = 300.0,
        acceleration: float | None = None,
    ) -> bool:
        """Absolute joint move

        Parameters
        ----------
        A1-A6, E1-E3 : float
            Target angles of axes

        velocity : float
            Velocity in percent of maximum velocity, range 1.0-100.0

        wait_move_finished : bool
            true: wait until movement is finished
            false: only wait for command ack and not until move is finished

        move_finished_timeout : float
            timout in seconds for waiting for the move to finish, `None` will wait indefinetly

        acceleration : float | None
            optional acceleration of move in percent of maximum acceleration of robot. Controller defaults to 40%
            requires igus Robot Control version >= V14-004-1 on robot controller
        """
        command = (
            f"CMD Move Joint {A1} {A2} {A3} {A4} {A5} {A6} {E1} {E2} {E3} {velocity}"
        )

        if (
            (acceleration is not None)
            and (acceleration >= 0.0)
            and (acceleration <= 100.0)
        ):
            command = f"{command} {acceleration}"

        if wait_move_finished:
            self._register_answer("EXECEND")

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id, timeout=30.0)) is not None:
            logger.debug("Error in Move Joints command: %s", error_msg)
            return False

        if wait_move_finished:
            if (
                error_msg := self._wait_for_answer(
                    "EXECEND", timeout=move_finished_timeout
                )
            ) is not None:
                logger.debug("Exec Error in Move Joints command: %s", error_msg)
                return False
        return True

    def move_joints_relative(
        self,
        A1: float,
        A2: float,
        A3: float,
        A4: float,
        A5: float,
        A6: float,
        E1: float,
        E2: float,
        E3: float,
        velocity: float,
        wait_move_finished: bool = False,
        move_finished_timeout: float | None = 300.0,
        acceleration: float | None = None,
    ) -> bool:
        """Relative joint move

        Parameters
        ----------
        A1-A6, E1-E3 : float
            Target angles of axes

        velocity : float
            Velocity in percent of maximum velocity, range 1.0-100.0

        wait_move_finished : bool
            true: wait until movement is finished
            false: only wait for command ack and not until move is finished

        move_finished_timeout : float
            timout in seconds for waiting for the move to finish, `None` will wait indefinetly

        acceleration : float | None
            optional acceleration of move in percent of maximum acceleration of robot. Controller defaults to 40%
            requires igus Robot Control version >= V14-004-1 on robot controller
        """
        command = f"CMD Move RelativeJoint {A1} {A2} {A3} {A4} {A5} {A6} {E1} {E2} {E3} {velocity}"

        if (
            (acceleration is not None)
            and (acceleration >= 0.0)
            and (acceleration <= 100.0)
        ):
            command = f"{command} {acceleration}"

        if wait_move_finished:
            self._register_answer("EXECEND")

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id, timeout=30.0)) is not None:
            logger.debug("Error in Move Joints command: %s", error_msg)
            return False

        if wait_move_finished:
            if (
                error_msg := self._wait_for_answer(
                    "EXECEND", timeout=move_finished_timeout
                )
            ) is not None:
                logger.debug(
                    "Exec Error in Move Joints Relative command: %s", error_msg
                )
                return False
        return True

    def move_cartesian(
        self,
        X: float,
        Y: float,
        Z: float,
        A: float,
        B: float,
        C: float,
        E1: float,
        E2: float,
        E3: float,
        velocity: float,
        frame: str = "#base",
        wait_move_finished: bool = False,
        move_finished_timeout: float | None = 300.0,
        acceleration: float | None = None,
    ) -> bool:
        """Cartesian move

        Parameters
        ----------
        X,Y,Z,A,B,C,E1-E3 : float
            Target angles of axes

        velocity : float
            Velocity in mm/s

        frame : str
            frame of the coordinates, default is `#base`

        wait_move_finished : bool
            true: wait until movement is finished
            false: only wait for command ack and not until move is finished

        move_finished_timeout : float
            timout in seconds for waiting for the move to finish, `None` will wait indefinetly

        acceleration : float | None
            optional acceleration of move in percent of maximum acceleration of robot. Controller defaults to 40%
            requires igus Robot Control version >= V14-004-1 on robot controller
        """
        command = (
            f"CMD Move Cart {X} {Y} {Z} {A} {B} {C} {E1} {E2} {E3} {velocity} {frame}"
        )

        if (
            (acceleration is not None)
            and (acceleration >= 0.0)
            and (acceleration <= 100.0)
        ):
            command = f"{command} {acceleration}"

        if wait_move_finished:
            self._register_answer("EXECEND")

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id, timeout=30.0)) is not None:
            logger.debug("Error in Move Joints command: %s", error_msg)
            return False

        if wait_move_finished:
            if (
                error_msg := self._wait_for_answer(
                    "EXECEND", timeout=move_finished_timeout
                )
            ) is not None:
                logger.debug("Exec Error in Move Cartesian command: %s", error_msg)
                return False

        return True

    def move_base_relative(
        self,
        X: float,
        Y: float,
        Z: float,
        A: float,
        B: float,
        C: float,
        E1: float,
        E2: float,
        E3: float,
        velocity: float,
        wait_move_finished: bool = False,
        move_finished_timeout: float | None = 300.0,
        acceleration: float | None = None,
    ) -> bool:
        """Relative cartesian move in base coordinate system

        Parameters
        ----------
        X,Y,Z,A,B,C,E1-E3 : float
            Target angles of axes

        velocity : float
            Velocity in mm/s

        frame : str
            frame of the coordinates, default is `#base`

        wait_move_finished : bool
            true: wait until movement is finished
            false: only wait for command ack and not until move is finished

        move_finished_timeout : float
            timout in seconds for waiting for the move to finish, `None` will wait indefinetly

        acceleration : float | None
            optional acceleration of move in percent of maximum acceleration of robot. Controller defaults to 40%
            requires igus Robot Control version >= V14-004-1 on robot controller
        """
        command = (
            f"CMD Move RelativeBase {X} {Y} {Z} {A} {B} {C} {E1} {E2} {E3} {velocity}"
        )

        if (
            (acceleration is not None)
            and (acceleration >= 0.0)
            and (acceleration <= 100.0)
        ):
            command = f"{command} {acceleration}"

        if wait_move_finished:
            self._register_answer("EXECEND")

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id, timeout=30.0)) is not None:
            logger.debug("Error in Move Joints command: %s", error_msg)
            return False

        if wait_move_finished:
            if (
                error_msg := self._wait_for_answer(
                    "EXECEND", timeout=move_finished_timeout
                )
            ) is not None:
                logger.debug("Exec Error in Move BaseRelative command: %s", error_msg)
                return False

        return True

    def move_tool_relative(
        self,
        X: float,
        Y: float,
        Z: float,
        A: float,
        B: float,
        C: float,
        E1: float,
        E2: float,
        E3: float,
        velocity: float,
        wait_move_finished: bool = False,
        move_finished_timeout: float | None = 300.0,
        acceleration: float | None = None,
    ) -> bool:
        """Relative cartesian move in tool coordinate system

        Parameters
        ----------
        X,Y,Z,A,B,C,E1-E3 : float
            Target angles of axes

        velocity : float
            Velocity in mm/s

        frame : str
            frame of the coordinates, default is `#base`

        wait_move_finished : bool
            true: wait until movement is finished
            false: only wait for command ack and not until move is finished

        move_finished_timeout : float
            timout in seconds for waiting for the move to finish, `None` will wait indefinetly

        acceleration : float | None
            optional acceleration of move in percent of maximum acceleration of robot. Controller defaults to 40%
            requires igus Robot Control version >= V14-004-1 on robot controller
        """
        command = (
            f"CMD Move RelativeTool {X} {Y} {Z} {A} {B} {C} {E1} {E2} {E3} {velocity}"
        )

        if (
            (acceleration is not None)
            and (acceleration >= 0.0)
            and (acceleration <= 100.0)
        ):
            command = f"{command} {acceleration}"

        if wait_move_finished:
            self._register_answer("EXECEND")

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id, timeout=30.0)) is not None:
            logger.debug("Error in Move Joints command: %s", error_msg)
            return False

        if wait_move_finished:
            if (
                error_msg := self._wait_for_answer(
                    "EXECEND", timeout=move_finished_timeout
                )
            ) is not None:
                logger.debug("Exec Error in Move BaseTool command: %s", error_msg)
                return False

        return True

    def stop_move(self) -> bool:
        """Stop movement

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """

        msg_id = self._send_command("CMD Move Stop", True)
        if (error_msg := self._wait_for_answer(msg_id, timeout=5.0)) is not None:
            logger.debug("Error in Move Stop command: %s", error_msg)
            return False
        else:
            return True

    def start_jog(self):
        """starts live jog. Set speeds via set_jog_values"""
        self.jog_intervall = self.ACTIVE_JOG_INTERVAL_SEC
        self.live_jog_active = True

    def stop_jog(self):
        """stops live jog."""
        self.live_jog_active = False
        self.jog_intervall = self.ALIVE_JOG_INTERVAL_SEC
        self.jog_speeds = {
            "A1": 0.0,
            "A2": 0.0,
            "A3": 0.0,
            "A4": 0.0,
            "A5": 0.0,
            "A6": 0.0,
            "E1": 0.0,
            "E2": 0.0,
            "E3": 0.0,
        }

    def set_jog_values(
        self,
        A1: float,
        A2: float,
        A3: float,
        A4: float,
        A5: float,
        A6: float,
        E1: float,
        E2: float,
        E3: float,
    ) -> None:
        """
        Sets live jog axes speeds.

        Parameters
        ----------
            A1-A6, E1-3 : float
                axes speeds in percent of maximum speed
        """
        with self.jog_speeds_lock:
            self.jog_speeds = {
                "A1": A1,
                "A2": A2,
                "A3": A3,
                "A4": A4,
                "A5": A5,
                "A6": A6,
                "E1": E1,
                "E2": E2,
                "E3": E3,
            }

    def set_motion_type(self, motion_type: MotionType):
        """Set motion type

        Parameters
        ----------
        motion_type : MotionType
            motion type

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = f"CMD MotionType{motion_type.value}"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in MotionType command: %s", error_msg)
            return False
        else:
            return True

    def set_override(self, override: float):
        """Set override

        Parameters
        ----------
        override : float
            override percent

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = f"CMD Override {override}"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in Override command: %s", error_msg)
            return False
        else:
            return True

    def set_dout(self, id: int, value: bool):
        """Set digital out

        Parameters
        ----------
        id : int
            index of DOUT (0 to 63)

        value : bool
            value to set DOUT to

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        if (id < 0) or (id > 63):
            raise ValueError

        command = f"CMD DOUT {id} {str(value).lower()}"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in DOUT command: %s", error_msg)
            return False
        else:
            return True

    def set_din(self, id: int, value: bool):
        """Set digital inout, only available in simulation

        Parameters
        ----------
        id : int
            index of DIN (0 to 63)

        value : bool
            value to set DIN to

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        if (id < 0) or (id > 63):
            raise ValueError

        command = f"CMD DIN {id} {str(value).lower()}"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in DIN command: %s", error_msg)
            return False
        else:
            return True

    def set_global_signal(self, id: int, value: bool):
        """Set global signal

        Parameters
        ----------
        id : int
            index of signal (0 to 99)

        value : bool
            value to set signal to

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        if (id < 0) or (id > 99):
            raise ValueError

        command = f"CMD GSIG {id} {str(value).lower()}"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in DIN command: %s", error_msg)
            return False
        else:
            return True

    def load_programm(self, program_name: str) -> bool:
        """Load a program file from disk into the robot controller

        Parameters
        ----------
        program_name : str
            the name in the directory /Data/Programs/, e.g. “test.xml”

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = f"CMD LoadProgram {program_name}"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in load_program command: %s", error_msg)
            return False
        else:
            return True

    def load_logic_programm(self, program_name: str) -> bool:
        """Load a logic program file from disk into the robot controller

        Parameters
        ----------
        program_name : str
            the name in the directory /Data/Programs/, e.g. “test.xml”

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = f"CMD LoadLogicProgram {program_name}"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in load_logic_program command: %s", error_msg)
            return False
        else:
            return True

    def start_programm(self) -> bool:
        """Start currently loaded Program

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = "CMD StartProgram"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in start_program command: %s", error_msg)
            return False
        else:
            return True

    def stop_programm(self) -> bool:
        """Stop currently running Program

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = "CMD StopProgram"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in stop_program command: %s", error_msg)
            return False
        else:
            return True

    def pause_programm(self) -> bool:
        """Pause currently running Program

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = "CMD PauseProgram"

        msg_id = self._send_command(command, True)
        if (error_msg := self._wait_for_answer(msg_id)) is not None:
            logger.debug("Error in pause_program command: %s", error_msg)
            return False
        else:
            return True

    def upload_file(self, path: str | Path, target_directory: str) -> bool:
        """Uploads file to iRC into `/Data/<target_directory>`

        Parameters
        ----------
        path : str | Path
            Path to file which should be uploaded

        target_directory : str
            directory on iRC `/Data/<target_directory>` into which file will be uploaded, e.g. `Programs` for normal robot programs

        Returns
        -------
        bool
            `True` file was uploaded successfully
            `False` there was an error during upload
        """

        if isinstance(path, Path):
            file_path = path
        elif isinstance(path, str):
            file_path = Path(path)
        else:
            return False

        try:
            with open(file_path, "r") as fp:
                lines = []
                while line := fp.readline():
                    logger.debug(line)
                    lines.append(line)

        except OSError as e:
            logger.error("Error reading %s: %s", str(Path), str(e))
            return False

        command = f"CMD UploadFileInit {target_directory + '/' + str(file_path.name)} {len(lines)} 0"

        self._send_command(command, True)

        for line in lines:
            command = f"CMD UploadFileLine {line.rstrip()}"

            self._send_command(command, True)

        command = "CMD UploadFileFinish"

        self._send_command(command, True)
        return True

    def enable_can_bridge(self, enabled: bool) -> None:
        """Enables or diables CAN bridge mode. All other functions are disabled in CAN bridge mode.

        Parameters
        ----------
        enabled : bool
            `True` bridge mode enabled
            `False` bridge mode disabled
        """
        if enabled is True:
            self.can_mode = True
            self._send_command("CANBridge SwitchOn")
        else:
            self._send_command("CANBridge SwitchOff")
            self.can_mode = False

    def can_send(self, msg_id: int, length: int, data: bytearray) -> None:
        """Send CAN message in CAN bridge mode.

        Parameters
        ----------
        msg_id : int
            message id of can message
        length : int
            length of data to send. Actual length used of the 8 data bytes
        data : bytearray
            data for CAN message always 8 bytes
        """
        if not self.can_mode:
            logger.debug("can_send: CAN mode not enabled")
            return

        command = f"CANBridge Msg ID {msg_id} Len {length} Data " + " ".join(
            [str(int(i)) for i in data]
        )

        self._send_command(command)

    def can_receive(
        self, blocking: bool = True, timeout: float | None = None
    ) -> dict[str, Any] | None:
        """Receive CAN message in CAN bridge mode from the recveive queue.

        Returns
        -------
        tuple[int, int, bytearray] | None
            Returns a tuple of (msg_id, length, data) if a message was received or None if nothing was received within the timeout.
        """
        if not self.can_mode:
            logger.debug("can_receive: CAN mode not enabled")
            return None

        try:
            item = self.can_queue.get(blocking, timeout)
        except Empty:
            return None

        return item


# Monkey patch to maintain backward compatibility
CRIController.MotionType = MotionType  # type: ignore
