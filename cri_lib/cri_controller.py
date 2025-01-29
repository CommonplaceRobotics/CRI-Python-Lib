from enum import Enum
import logging
import socket
import threading
from time import sleep, time
from pathlib import Path
from typing import Callable
from queue import Queue, Empty

from .robot_state import RobotState, KinematicsState

from .cri_protocol_parser import CRIProtocolParser

from .cri_errors import CRICommandTimeOutError, CRIConnectionError


class CRIController:
    """
    Class implementing the CRI network protocol for igus Robot Control.
    """

    ALIVE_JOG_INTERVAL_SEC = 0.2
    ACTIVE_JOG_INTERVAL_SEC = 0.02
    RECEIVE_TIMEOUT_SEC = 5
    DEFAULT_ANSWER_TIMEOUT = 10.0

    class MotionType(Enum):
        """Robot Motion Type for Jogging"""

        Joint = "Joint"
        CartBase = "CartBase"
        CartTool = "CartTool"
        Platform = "Platform"

    def __init__(self):
        self.robot_state: RobotState = RobotState()
        self.robot_state_lock = threading.Lock()

        self.parser = CRIProtocolParser(self.robot_state, self.robot_state_lock)

        self.connected = False
        self.sock = None
        self.socket_write_lock = threading.Lock()

        self.can_mode: bool = False
        self.can_queue: Queue = Queue()

        self.jog_thread = threading.Thread(target=self._bg_alivejog_thread, daemon=True)
        self.receive_thread = threading.Thread(
            target=self._bg_receive_thread, daemon=True
        )

        self.sent_command_counter_lock = threading.Lock()
        self.sent_command_counter = 0
        self.answer_events_lock = threading.Lock()
        self.answer_events: dict[str, threading.Event] = {}
        self.error_messages: dict[str, str] = {}

        self.status_callback = None

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

    def connect(self, host: str, port: int = 3920) -> bool:
        """
        Connect to iRC.

        Parameters
        ----------
        host : str
            IP address or hostname of iRC
        port : int
            port of iRC

        Returns
        -------
        bool
            True if connected
            False if not connected

        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.1)  # Set a timeout of 0.1 seconds
        try:
            ip = socket.gethostbyname(host)
            self.sock.connect((ip, port))
            logging.debug("\t Robot connected: %s:%d", host, port)
            self.connected = True

            # Start receiving commands
            self.receive_thread.start()

            # Start sending ALIVEJOG message
            self.jog_thread.start()

            return True

        except ConnectionRefusedError:
            logging.error(
                f"Connection refused: Unable to connect to {host}:{port}"
            )
            return False
        except Exception as e:
            logging.error(f"An error occurred: {str(e)}")
            return False

    def close(self) -> None:
        """
        Close network connection. Might block for a while waiting for the threads to finish.
        """

        if not self.connected:
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
    ) -> int | None:
        """Sends the given command to iRC.

        Parameters
        ----------
        command : str
            Command to be sent without `CRISTART`, counter and `CRIEND`

        Returns
        -------
        int | None
            If the command was sent the message_id gets returned or None if there was an error.

        """
        if not self.connected:
            logging.error("Not connected. Use connect() to establish a connection.")
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
            logging.debug("Sent command: %s", message)

            return command_counter

        except Exception as e:
            logging.error(f"Failed to send command: {str(e)}")
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
            if self.live_jog_active:
                with self.jog_speeds_lock:
                    command = f"ALIVEJOG {self.jog_speeds['A1']} {self.jog_speeds['A2']} {self.jog_speeds['A3']} {self.jog_speeds['A4']} {self.jog_speeds['A5']} {self.jog_speeds['A6']} {self.jog_speeds['E1']} {self.jog_speeds['E2']} {self.jog_speeds['E3']}"
            else:
                command = "ALIVEJOG 0 0 0 0 0 0 0 0 0"

            if self._send_command(command) is None:
                logging.error("AliveJog Thread: Connection lost.")
                self.connected = False
                return

            sleep(self.jog_intervall)

    def _bg_receive_thread(self) -> None:
        """
        Background thread receiving data and parsing it to the robot state.
        """
        message_buffer = bytearray()

        while self.connected:
            try:
                recv_buffer = self.sock.recv(4096)
            except TimeoutError:
                continue

            if recv_buffer == b"":
                self.connected = False
                logging.error("Receive Thread: Connection lost.")
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
        self, message_id: str, timeout: float | None = None
    ) -> None | str:
        """Waits for an answer to a message.
        The answer event will be removed after the call, even if there was a timeout. Choose timeout accordingly.

        Parameters
        ----------
        message_id : int
            message id of sent message of which an answer is expected

        timeout : float | None
            timeout for wait in seconds. `None` will wait indefinetly

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

        with self.answer_events_lock:
            if message_id not in self.answer_events:
                return False
            wait_event = self.answer_events[message_id]

        # prevent deadlock through answer_events_lock
        success = wait_event.wait(timeout=timeout)

        if not success:
            raise CRICommandTimeOutError()

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
            logging.debug("Received: %s", message)

        if (notification := self.parser.parse_message(message)) is not None:
            if notification["answer"] == "status" and self.status_callback is not None:
                self.status_callback(self.robot_state)

            if notification["answer"] == "CAN":
                self.can_queue.put_nowait(notification["can"])

            with self.answer_events_lock:
                msg_id = notification["answer"]

                if msg_id in self.answer_events:
                    if "error" in notification:
                        self.error_messages[msg_id] = notification["error"]

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

    def reset(self) -> bool:
        """Reset robot

        Returns
        -------
        bool:
            `True` if request was successful
            `False` if request was not successful
        """
        if (msg_id := self._send_command("CMD Reset", True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in RESET command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def enable(self) -> bool:
        """Enable robot
           An potential error message received from the robot will be logged with priority DEBUG

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        if (msg_id := self._send_command("CMD Enable", True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in ENABLE command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def disable(self) -> bool:
        """Disable robot

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        if (msg_id := self._send_command("CMD Disable", True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in DISABLE command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def set_active_control(self, active: bool) -> None:
        """Acquire or return active control of robot

        Parameters
        ----------
        active : bool
            `True` acquire active control
            `False` return active control
        """
        if (
            self._send_command(
                f"CMD SetActive {str(active).lower()}",
                True,
                f"Active_{str(active).lower()}",
            )
            is not None
        ):
            if (
                error_msg := self._wait_for_answer(
                    f"Active_{str(active).lower()}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in set active control command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def zero_all_joints(self) -> bool:
        """Set all joints to zero

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """

        if (msg_id := self._send_command("CMD SetJointsToZero", True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in SetJointsToZero command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def reference_all_joints(self) -> bool:
        """Reference all joints. Long timout of 30 seconds.

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """

        if (msg_id := self._send_command("CMD ReferenceAllJoints", True)) is not None:
            if (
                error_msg := self._wait_for_answer(f"{msg_id}", timeout=30.0)
            ) is not None:
                logging.debug("Error in ReferenceAllJoints command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def reference_single_joint(self, joint: str) -> bool:
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

        if (
            msg_id := self._send_command(f"CMD ReferenceSingleJoint {joint_msg}", True)
        ) is not None:
            if (
                error_msg := self._wait_for_answer(f"{msg_id}", timeout=30.0)
            ) is not None:
                logging.debug("Error in ReferenceSingleJoint command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def get_referencing_info(self):
        """Reference all joints. Long timout of 30 seconds.

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        if (
            self._send_command("CMD GetReferencingInfo", True, "info_referencing")
            is not None
        ):
            if (
                error_msg := self._wait_for_answer(
                    "info_referencing", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in GetReferencingInfo command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def wait_for_kinematics_ready(self, timeout: float | None = 30) -> bool:
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
        """
        command = (
            f"CMD Move Joint {A1} {A2} {A3} {A4} {A5} {A6} {E1} {E2} {E3} {velocity}"
        )
        if wait_move_finished:
            self._register_answer("EXECEND")

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(f"{msg_id}", timeout=30.0)
            ) is not None:
                logging.debug("Error in Move Joints command: %s", error_msg)
                return False

            if wait_move_finished:
                if (
                    error_msg := self._wait_for_answer(
                        "EXECEND", timeout=move_finished_timeout
                    )
                ) is not None:
                    logging.debug("Exec Error in Move Joints command: %s", error_msg)
                    return False

            return True

        else:
            return False

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
        """
        command = f"CMD Move RelativeJoint {A1} {A2} {A3} {A4} {A5} {A6} {E1} {E2} {E3} {velocity}"
        if wait_move_finished:
            self._register_answer("EXECEND")

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(f"{msg_id}", timeout=30.0)
            ) is not None:
                logging.debug("Error in Move Joints command: %s", error_msg)
                return False

            if wait_move_finished:
                if (
                    error_msg := self._wait_for_answer(
                        "EXECEND", timeout=move_finished_timeout
                    )
                ) is not None:
                    logging.debug(
                        "Exec Error in Move Joints Relative command: %s", error_msg
                    )
                    return False

            return True

        else:
            return False

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
        """
        command = (
            f"CMD Move Cart {X} {Y} {Z} {A} {B} {C} {E1} {E2} {E3} {velocity} {frame}"
        )

        if wait_move_finished:
            self._register_answer("EXECEND")

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(f"{msg_id}", timeout=30.0)
            ) is not None:
                logging.debug("Error in Move Joints command: %s", error_msg)
                return False

            if wait_move_finished:
                if (
                    error_msg := self._wait_for_answer(
                        "EXECEND", timeout=move_finished_timeout
                    )
                ) is not None:
                    logging.debug("Exec Error in Move Cartesian command: %s", error_msg)
                    return False

            return True

        else:
            return False

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
        """
        command = (
            f"CMD Move RelativeBase {X} {Y} {Z} {A} {B} {C} {E1} {E2} {E3} {velocity}"
        )

        if wait_move_finished:
            self._register_answer("EXECEND")

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(f"{msg_id}", timeout=30.0)
            ) is not None:
                logging.debug("Error in Move Joints command: %s", error_msg)
                return False

            if wait_move_finished:
                if (
                    error_msg := self._wait_for_answer(
                        "EXECEND", timeout=move_finished_timeout
                    )
                ) is not None:
                    logging.debug(
                        "Exec Error in Move BaseRelative command: %s", error_msg
                    )
                    return False

            return True

        else:
            return False

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
        """
        command = (
            f"CMD Move RelativeTool {X} {Y} {Z} {A} {B} {C} {E1} {E2} {E3} {velocity}"
        )

        if wait_move_finished:
            self._register_answer("EXECEND")

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(f"{msg_id}", timeout=30.0)
            ) is not None:
                logging.debug("Error in Move Joints command: %s", error_msg)
                return False

            if wait_move_finished:
                if (
                    error_msg := self._wait_for_answer(
                        "EXECEND", timeout=move_finished_timeout
                    )
                ) is not None:
                    logging.debug("Exec Error in Move BaseTool command: %s", error_msg)
                    return False

            return True

        else:
            return False

    def stop_move(self) -> None:
        """Stop movement

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """

        if (msg_id := self._send_command("CMD Move Stop", True)) is not None:
            if (
                error_msg := self._wait_for_answer(f"{msg_id}", timeout=5.0)
            ) is not None:
                logging.debug("Error in Move Stop command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

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

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in MotionType command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

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

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in Override command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

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

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in DOUT command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

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

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in DIN command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

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

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in DIN command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

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

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in load_program command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

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

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in load_logic_program command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def start_programm(self) -> bool:
        """Start currently loaded Program

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = "CMD StartProgram"

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in start_program command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def stop_programm(self) -> bool:
        """Stop currently running Program

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = "CMD StopProgram"

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in stop_program command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

    def pause_programm(self) -> bool:
        """Pause currently running Program

        Returns
        -------
        bool
            `True` if request was successful
            `False` if request was not successful
        """
        command = "CMD PauseProgram"

        if (msg_id := self._send_command(command, True)) is not None:
            if (
                error_msg := self._wait_for_answer(
                    f"{msg_id}", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in pause_program command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False

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
                    print(line)
                    lines.append(line)

        except OSError as e:
            logging.error("Error reading %s: %s", str(Path), str(e))
            return False

        command = f"CMD UploadFileInit {target_directory + '/' + str(file_path.name)} {len(lines)} 0"

        if self._send_command(command, True) is None:
            return False

        for line in lines:
            command = f"CMD UploadFileLine {line.rstrip()}"

            if self._send_command(command, True) is None:
                return False

        command = "CMD UploadFileFinish"

        if self._send_command(command, True) is None:
            return False

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
            logging.debug("can_send: CAN mode not enabled")
            return

        command = f"CANBridge Msg ID {msg_id} Len {length} Data " + " ".join(
            [str(int(i)) for i in data]
        )

        self._send_command(command)

    def can_receive(
        self, blocking: bool = True, timeout: float | None = None
    ) -> dict[str, any] | None:
        """Receive CAN message in CAN bridge mode from the recveive queue.

        Returns
        -------
        tuple[int, int, bytearray] | None
            Returns a tuple of (msg_id, length, data) if a message was received or None if nothing was received within the timeout.
        """
        if not self.can_mode:
            logging.debug("can_receive: CAN mode not enabled")
            return

        try:
            item = self.can_queue.get(blocking, timeout)
        except Empty:
            return None

        return item

    def get_board_temperatures(self, blocking: bool = True, timeout: float | None = None) -> bool:
        """Receive motor controller PCB temperatures and save in robot state

            Parameters
            ----------
            blocking: bool
                wait for response, always returns True if not waiting
            
            timeout: float | None
                timeout for waiting in seconds or None for infinite waiting
        """
        if (
            self._send_command("SYSTEM GetBoardTemp", True, "info_boardtemp")
            is not None
        ):
            if (
                error_msg := self._wait_for_answer(
                    "info_boardtemp", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in GetBoardTemp command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False
        
    def get_motor_temperatures(self, blocking: bool = True, timeout: float | None = None) -> bool:
        """Receive motor temperatures and save in robot state

            Parameters
            ----------
            blocking: bool
                wait for response, always returns True if not waiting
            
            timeout: float | None
                timeout for waiting in seconds or None for infinite waiting
        """
        if (
            self._send_command("SYSTEM GetMotorTemp", True, "info_motortemp")
            is not None
        ):
            if (
                error_msg := self._wait_for_answer(
                    "info_motortemp", timeout=self.DEFAULT_ANSWER_TIMEOUT
                )
            ) is not None:
                logging.debug("Error in GetMotorTemp command: %s", error_msg)
                return False
            else:
                return True
        else:
            return False