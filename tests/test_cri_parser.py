import asyncio
import copy
import threading

import pytest

from cri_lib import (
    CRICommandTimeOutError,
    CRIController,
    CRIProtocolParser,
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
    SentCommand,
)


def robot_state_equal(actual: RobotState, expected: RobotState) -> bool:
    """Compare robot state without requiring update nanosecond values to match."""
    if set(actual.category_time_ns) != set(expected.category_time_ns):
        return False
    # copy the timestamp values to facilitate equals comparison of the dataclasses
    expected_copy = copy.deepcopy(expected)
    expected_copy.category_time_ns.update(actual.category_time_ns)
    return actual == expected_copy


def register_test_event(controller: CRIController, answer_id: str) -> asyncio.Event:
    event = asyncio.Event()
    controller.answer_events.setdefault(answer_id, []).append(event)
    return event


def test_parse_state():
    """This tests parsing the STATUS message with the assumption that all possible axes are present"""

    test_message = """
CRISTART 1234 STATUS MODE joint
POSJOINTSETPOINT 1.00 2.00 3.00 4.00 5.00 6.00 7.00 8.00 9.00 10.00 11.00 12.00 13.00 14.00 15.00 16.00
POSJOINTCURRENT 1.00 2.00 3.00 4.00 5.00 6.00 7.00 8.00 9.00 10.00 11.00 12.00 13.00 14.00 15.00 16.00
POSCARTROBOT 10.0 20.0 30.0 0.00 90.00 0.00
POSCARTPLATFORM 10.0 20.0 180.00
OVERRIDE 80.0
DIN 0000000000000FF00 DOUT 0000000000000FF00
ESTOP 3 SUPPLY 23000 CURRENTALL 2600
CURRENTJOINTS 10 20 30 40 50 60 70 80 90 100 110 120 130 140 150 160
ERROR no_error 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255
KINSTATE 30
OPMODE -1
CARTSPEED 123.4
GSIG 00ff00ff00ff
FRAMEROBOT MyFrame 1.0 2.0 3.0 4.0 5.0 6.0
UNKNOWNSEGMENT 1 2 3 4
CRIEND
    """

    robot_state_correct = RobotState()
    robot_state_correct.robot_axes_count = 6
    robot_state_correct.external_axes_count = 3
    robot_state_correct.tool_axes_count = 3
    robot_state_correct.platform_axes_count = 4
    robot_state_correct.category_time_ns["STATUS"] = 0
    robot_state_correct.mode = RobotMode.JOINT
    robot_state_correct.joints_set_point = JointsState(
        1.00,
        2.00,
        3.00,
        4.00,
        5.00,
        6.00,
        7.00,
        8.00,
        9.00,
        10.00,
        11.00,
        12.00,
        13.00,
        14.00,
        15.00,
        16.00,
    )
    robot_state_correct.joints_current = JointsState(
        1.00,
        2.00,
        3.00,
        4.00,
        5.00,
        6.00,
        7.00,
        8.00,
        9.00,
        10.00,
        11.00,
        12.00,
        13.00,
        14.00,
        15.00,
        16.00,
    )
    robot_state_correct.position_robot = RobotCartesianPosition(
        10.0, 20.0, 30.0, 0.0, 90.0, 0.00
    )
    robot_state_correct.position_platform = PlatformCartesianPosition(10.0, 20.0, 180.0)
    robot_state_correct.override = 80.0
    robot_state_correct.din = [False] * 64
    robot_state_correct.din[8:16] = [True] * 8
    robot_state_correct.dout = [False] * 64
    robot_state_correct.dout[8:16] = [True] * 8
    robot_state_correct.emergency_stop_ok = True
    robot_state_correct.main_relay = True
    robot_state_correct.supply_voltage = 23.0
    robot_state_correct.current_total = 2.6
    robot_state_correct.current_joints = [i * 10 for i in range(1, 17)]
    robot_state_correct.error_states = [ErrorStates(*([True] * 8))] * 16
    robot_state_correct.kinematics_state = KinematicsState(30)
    robot_state_correct.operation_mode = OperationMode(-1)
    robot_state_correct.cart_speed_mm_per_s = 123.4
    robot_state_correct.global_signals = (
        [True] * 8
        + [False] * 8
        + [True] * 8
        + [False] * 8
        + [True] * 8
        + [False] * 8
        + [False] * 80
    )
    robot_state_correct.frame_name = "MyFrame"
    robot_state_correct.frame_position_current = RobotCartesianPosition(
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0
    )
    robot_state_correct.current_joints = [
        0.01,
        0.02,
        0.03,
        0.04,
        0.05,
        0.06,
        0.07,
        0.08,
        0.09,
        0.1,
        0.11,
        0.12,
        0.13,
        0.14,
        0.15,
        0.16,
    ]
    robot_state_correct.combined_axes_error = "no_error"

    controller = CRIController()
    controller.parser.robot_state.robot_axes_count = 6
    controller.parser.robot_state.external_axes_count = 3
    controller.parser.robot_state.tool_axes_count = 3
    controller.parser.robot_state.platform_axes_count = 4
    controller._parse_message(test_message)

    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_state2():
    """This tests parsing the STATUS message with missing axes"""

    test_message = """
CRISTART 1234 STATUS MODE joint
POSJOINTSETPOINT 1.00 2.00 3.00 4.00 5.00 6.00 7.00 8.00 9.00 10.00 11.00 12.00 13.00 14.00 15.00 16.00
POSJOINTCURRENT 1.00 2.00 3.00 4.00 5.00 6.00 7.00 8.00 9.00 10.00 11.00 12.00 13.00 14.00 15.00 16.00
POSCARTROBOT 10.0 20.0 30.0 0.00 90.00 0.00
POSCARTPLATFORM 10.0 20.0 180.00
OVERRIDE 80.0
DIN 0000000000000FF00 DOUT 0000000000000FF00
ESTOP 3 SUPPLY 23000 CURRENTALL 2600
CURRENTJOINTS 10 20 30 40 50 60 70 80 90 100 110 120 130 140 150 160
ERROR no_error 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255
KINSTATE 30
OPMODE -1
CARTSPEED 123.4
GSIG 00ff00ff00ff
FRAMEROBOT MyFrame 1.0 2.0 3.0 4.0 5.0 6.0
UNKNOWNSEGMENT 1 2 3 4
CRIEND
    """

    robot_state_correct = RobotState()
    robot_state_correct.robot_axes_count = 3
    robot_state_correct.external_axes_count = 2
    robot_state_correct.tool_axes_count = 1
    robot_state_correct.platform_axes_count = 2
    robot_state_correct.category_time_ns["STATUS"] = 0
    robot_state_correct.mode = RobotMode.JOINT
    robot_state_correct.joints_set_point = JointsState(
        1.00,
        2.00,
        3.00,
        0.00,
        0.00,
        0.00,
        4.00,
        5.00,
        0.00,
        6.00,
        0.00,
        0.00,
        7.00,
        8.00,
        0.00,
        0.00,
    )
    robot_state_correct.joints_current = JointsState(
        1.00,
        2.00,
        3.00,
        0.00,
        0.00,
        0.00,
        4.00,
        5.00,
        0.00,
        6.00,
        0.00,
        0.00,
        7.00,
        8.00,
        0.00,
        0.00,
    )
    es0 = ErrorStates()
    es255 = ErrorStates(True, True, True, True, True, True, True, True)
    robot_state_correct.position_robot = RobotCartesianPosition(
        10.0, 20.0, 30.0, 0.0, 90.0, 0.00
    )
    robot_state_correct.position_platform = PlatformCartesianPosition(10.0, 20.0, 180.0)
    robot_state_correct.override = 80.0
    robot_state_correct.din = [False] * 64
    robot_state_correct.din[8:16] = [True] * 8
    robot_state_correct.dout = [False] * 64
    robot_state_correct.dout[8:16] = [True] * 8
    robot_state_correct.emergency_stop_ok = True
    robot_state_correct.main_relay = True
    robot_state_correct.supply_voltage = 23.0
    robot_state_correct.current_total = 2.6
    robot_state_correct.current_joints = [i * 10 for i in range(1, 17)]
    # robot_state_correct.error_states = [ErrorStates(*([True] * 8))] * 16
    robot_state_correct.error_states = [
        es255,
        es255,
        es255,
        es0,
        es0,
        es0,
        es255,
        es255,
        es0,
        es255,
        es0,
        es0,
        es255,
        es255,
        es0,
        es0,
    ]
    robot_state_correct.kinematics_state = KinematicsState(30)
    robot_state_correct.operation_mode = OperationMode(-1)
    robot_state_correct.cart_speed_mm_per_s = 123.4
    robot_state_correct.global_signals = (
        [True] * 8
        + [False] * 8
        + [True] * 8
        + [False] * 8
        + [True] * 8
        + [False] * 8
        + [False] * 80
    )
    robot_state_correct.frame_name = "MyFrame"
    robot_state_correct.frame_position_current = RobotCartesianPosition(
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0
    )
    robot_state_correct.current_joints = [
        0.01,
        0.02,
        0.03,
        0.00,
        0.00,
        0.0,
        0.04,
        0.05,
        0.00,
        0.06,
        0.00,
        0.00,
        0.07,
        0.08,
        0.00,
        0.00,
    ]
    robot_state_correct.combined_axes_error = "no_error"

    controller = CRIController()
    # controller.parser.robot_state.robot_axes_count = 3
    # controller.parser.robot_state.external_axes_count = 2
    # controller.parser.robot_state.tool_axes_count = 1
    # controller.parser.robot_state.platform_axes_count = 2
    # Read axis counts from config message
    controller._parse_message(
        "CRISTART 1234 CONFIG Axes A1 1 2 3 4 A2 1 2 3 4 A3 1 2 3 4 E1 1 2 3 4 E2 1 2 3 4 T1 1 2 3 4 P1 1 2 3 4 P2 1 2 3 4"
    )
    controller._parse_message(test_message)

    # ignore the category time
    robot_state_correct.category_time_ns = controller.robot_state.category_time_ns
    print(controller.robot_state)
    print(robot_state_correct)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_config_axes1():
    """Tests parsing CONFIG Axis with all possible axes"""

    test_message = "CRISTART 1234 CONFIG Axes A1 1 2 3 4 A2 1 2 3 4 A3 1 2 3 4 A4 1 2 3 4 A5 1 2 3 4 A6 1 2 3 4 E1 1 2 3 4 E2 1 2 3 4 E3 1 2 3 4 T1 1 2 3 4 T2 1 2 3 4 T3 1 2 3 4 P1 1 2 3 4 P2 1 2 3 4 P3 1 2 3 4 P4 1 2 3 4"

    robot_state_correct = RobotState()
    robot_state_correct.robot_axes_count = 6
    robot_state_correct.external_axes_count = 3
    robot_state_correct.tool_axes_count = 3
    robot_state_correct.platform_axes_count = 4

    controller = CRIController()
    controller._parse_message(test_message)

    assert (
        controller.robot_state.robot_axes_count == robot_state_correct.robot_axes_count
    )
    assert (
        controller.robot_state.external_axes_count
        == robot_state_correct.external_axes_count
    )
    assert controller.robot_state.tool_axes_count == robot_state_correct.tool_axes_count
    assert (
        controller.robot_state.platform_axes_count
        == robot_state_correct.platform_axes_count
    )

    # ignore the category time
    robot_state_correct.category_time_ns = controller.robot_state.category_time_ns
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_config_axes2():
    """Tests parsing CONFIG Axis with missing axes"""

    test_message = "CRISTART 1234 CONFIG Axes A1 1 2 3 4 A2 1 2 3 4 A3 1 2 3 4 E1 1 2 3 4 E2 1 2 3 4 T1 1 2 3 4 P1 1 2 3 4 P2 1 2 3 4"

    robot_state_correct = RobotState()
    robot_state_correct.robot_axes_count = 3
    robot_state_correct.external_axes_count = 2
    robot_state_correct.tool_axes_count = 1
    robot_state_correct.platform_axes_count = 2

    controller = CRIController()
    controller._parse_message(test_message)

    assert (
        controller.robot_state.robot_axes_count == robot_state_correct.robot_axes_count
    )
    assert (
        controller.robot_state.external_axes_count
        == robot_state_correct.external_axes_count
    )
    assert controller.robot_state.tool_axes_count == robot_state_correct.tool_axes_count
    assert (
        controller.robot_state.platform_axes_count
        == robot_state_correct.platform_axes_count
    )

    # ignore the category time
    robot_state_correct.category_time_ns = controller.robot_state.category_time_ns
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_runstate_main():
    test_message = (
        "CRISTART 1234 RUNSTATE MAIN testmotion.xml pickpart.xml 12 3 0 2 CRIEND"
    )

    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["RUNSTATE"] = 0
    robot_state_correct.main_main_program = "testmotion.xml"
    robot_state_correct.main_current_program = "pickpart.xml"
    robot_state_correct.main_commands_count = 12
    robot_state_correct.main_current_command = 3
    robot_state_correct.main_runstate = RunState(0)
    robot_state_correct.main_replay_mode = ReplayMode(2)

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_runstate_logic():
    test_message = (
        "CRISTART 1234 RUNSTATE LOGIC testlogic.xml testlogic.xml 11 4 0 3 CRIEND"
    )

    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["RUNSTATE"] = 0
    robot_state_correct.logic_main_program = "testlogic.xml"
    robot_state_correct.logic_current_program = "testlogic.xml"
    robot_state_correct.logic_commands_count = 11
    robot_state_correct.logic_current_command = 4
    robot_state_correct.logic_runstate = RunState(0)
    robot_state_correct.logic_replay_mode = ReplayMode(3)

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_regr_parse_status_plattform():
    """Regression Test for "POSCARTPLATTFORM" typo in Robot Control"""

    test_message = "CRISTART 1234 STATUS POSCARTPLATTFORM 10.0 20.0 180.00 CRIEND"

    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["STATUS"] = 0
    robot_state_correct.position_platform = PlatformCartesianPosition(10.0, 20.0, 180.0)

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_cyclestat():
    """Test for cyclestat message"""

    test_message = "CRISTART 1234 CYCLESTAT 9.5 12.3 CRIEND"
    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["CYCLESTAT"] = 0
    robot_state_correct.cycle_time = 9.5
    robot_state_correct.workload = 12.3

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_gripperstate():
    """Test for gripperstate message"""

    test_message = "CRISTART 1234 GRIPPERSTATE 0.7 CRIEND"
    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["GRIPPERSTATE"] = 0
    robot_state_correct.gripper_state = 0.7

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_unknown_message_tpye():
    """Test for unknown message type"""
    test_message = "CRISTART 1234 UNKNOWN 1 2 3 4 5 CRIEND"
    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["UNKNOWN"] = 0

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_variables():
    """Test for variable message"""
    test_message = """CRISTART 7 VARIABLES
    ValuePosVariable #position 217.395 0 350.155 180 3.57994e-05 180 0 -20 110 0 90 0 0 0 0
    ValueNrVariable #programrunning 0
    ValueNrVariable #logicprogramrunning 0
    ValueNrVariable #parts-good 0
    ValueNrVariable #parts-bad 0
    ValuePosVariable #userframe-a 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0
    ValuePosVariable #userframe-b 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0
    ValuePosVariable #userframe-c 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0
    ValuePosVariable #position-userframe 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0
    FooVariableType sdgsdsd 1 2 3 4 CRIEND"""

    variables = {
        "#position": PosVariable(
            217.395, 0, 350.155, 180, 3.57994e-05, 180, 0, -20, 110, 0, 90, 0, 0, 0, 0
        ),
        "#programrunning": 0.0,
        "#logicprogramrunning": 0.0,
        "#parts-good": 0.0,
        "#parts-bad": 0.0,
        "#userframe-a": PosVariable(
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0,
            13.0,
            14.0,
            15.0,
        ),
        "#userframe-b": PosVariable(
            2.0,
            3.0,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0,
            13.0,
            14.0,
            15.0,
            16.0,
        ),
        "#userframe-c": PosVariable(
            3.0,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0,
            13.0,
            14.0,
            15.0,
            16.0,
            17.0,
        ),
        "#position-userframe": PosVariable(
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0,
            13.0,
            14.0,
            15.0,
            16.0,
            17.0,
            18.0,
        ),
    }
    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["VARIABLES"] = 0
    robot_state_correct.variabels = variables

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_opinfo():
    test_message = "CRISTART 6 OPINFO 0 235 235 114 5 0 0 CRIEND"
    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["OPINFO"] = 0

    robot_state_correct.operation_info = OperationInfo(0, 235, 235, 114, 5, 0, 0)

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_cmd_active():
    test_message = "CRISTART 4 CMD Active false CRIEND"
    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["CMD"] = 0

    robot_state_correct.active_control = False

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)

    test_message = "CRISTART 4 CMD Active true CRIEND"
    robot_state_correct.active_control = True
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)

    test_message = "CRISTART 4 CMD Active foo CRIEND"
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_message_robotcontrol():
    test_message = "CRISTART 1 MESSAGE RobotControl Version V980-14-002-3 CRIEND"

    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["MESSAGE"] = 0
    robot_state_correct.robot_control_version = "V980-14-002-3"

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_splits_quotes_aware():
    test_string = 'Configuration: "igus REBEL-6DOF" Type: "igus-REBEL/REBEL-6DOF-02" Gripper: ""test'
    correct_split = [
        "Configuration:",
        "igus REBEL-6DOF",
        "Type:",
        "igus-REBEL/REBEL-6DOF-02",
        "Gripper:",
        "",
        "test",
    ]

    test_split = CRIProtocolParser._split_quotes_aware(test_string)

    assert correct_split == test_split


def test_parse_message_configuration():
    test_message = 'CRISTART 2 MESSAGE Configuration: "igus REBEL-6DOF" Type: "igus-REBEL/REBEL-6DOF-02" Gripper: "Multigrip" CRIEND'

    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["MESSAGE"] = 0
    robot_state_correct.robot_configuration = "igus REBEL-6DOF"
    robot_state_correct.robot_type = "igus-REBEL/REBEL-6DOF-02"
    robot_state_correct.gripper_type = "Multigrip"

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)

    test_message = 'CRISTART 2 MESSAGE Type: "igus-REBEL/REBEL-6DOF-02test1" Gripper: "Multigriptest1" CRIEND'
    robot_state_correct.robot_type = "igus-REBEL/REBEL-6DOF-02test1"
    robot_state_correct.gripper_type = "Multigriptest1"

    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_config():
    test_message = "CRISTART 1234 CONFIG ProjectFile robotprj.xml CRIEND"

    robot_state_correct = RobotState()
    robot_state_correct.category_time_ns["CONFIG"] = 0
    robot_state_correct.project_file = "robotprj.xml"

    controller = CRIController()
    controller._parse_message(test_message)
    assert robot_state_equal(controller.robot_state, robot_state_correct)


def test_parse_cmdack():
    test_message = "CRISTART 1234 CMDACK 123 CRIEND"

    controller = CRIController()
    event = register_test_event(controller, "123")
    controller._parse_message(test_message)

    assert event.is_set()


def test_status_notification_wakes_all_registered_waiters():
    controller = CRIController()
    first_event = controller._register_answer("status")
    second_event = controller._register_answer("status")

    controller._parse_message("CRISTART 1234 STATUS MODE joint CRIEND")

    assert first_event is not second_event
    assert first_event.is_set()
    assert second_event.is_set()


def test_send_command_returns_its_answer_event(monkeypatch):
    controller = CRIController()

    class ImmediateReplySocket:
        sent_message = b""

        def sendall(self, message: bytes) -> None:
            self.sent_message = message
            controller._parse_message("CRISTART 1234 CMDACK 0 CRIEND")

    fake_socket = ImmediateReplySocket()
    monkeypatch.setattr(controller, "sock", fake_socket)
    controller.connected = True

    sent_command = controller._send_command("CMD Test", register_answer=True)
    unregistered_command = controller._send_command("CMD FireAndForget")

    assert sent_command.message_id == 0
    assert sent_command.answer_event is not None
    assert sent_command.answer_event.is_set()
    assert fake_socket.sent_message == b"CRISTART 1 CMD FireAndForget CRIEND"
    assert unregistered_command.message_id == 1
    assert unregistered_command.answer_event is None


def test_concurrent_status_waiters_complete_and_clean_up():
    async def scenario():
        controller = CRIController()
        callbacks = []
        controller.register_status_callback(callbacks.append)
        waiters = [
            asyncio.create_task(controller.wait_for_status_update_async(timeout=1.0))
            for _ in range(2)
        ]

        for _ in range(100):
            if len(controller.answer_events.get("status", [])) == 2:
                break
            await asyncio.sleep(0.001)

        assert len(controller.answer_events["status"]) == 2
        controller._parse_message("CRISTART 1234 STATUS MODE joint CRIEND")
        await asyncio.gather(*waiters)

        assert controller.answer_events == {}
        assert len(callbacks) == 1

    asyncio.run(scenario())


def test_shared_error_is_returned_to_each_waiter():
    async def scenario():
        controller = CRIController()
        first_event = controller._register_answer("123")
        second_event = controller._register_answer("123")
        waiters = [
            asyncio.create_task(
                controller._wait_for_answer_async("123", answer_event=event)
            )
            for event in (first_event, second_event)
        ]

        controller._parse_message(
            "CRISTART 1234 CMDERROR 123 There was an exception CRIEND"
        )

        assert await asyncio.gather(*waiters) == [
            "There was an exception",
            "There was an exception",
        ]
        assert controller.answer_events == {}
        assert controller.error_messages == {}

    asyncio.run(scenario())


def test_cancelled_waiter_does_not_remove_other_registration():
    async def scenario():
        controller = CRIController()
        cancelled_event = controller._register_answer("123")
        remaining_event = controller._register_answer("123")
        cancelled_waiter = asyncio.create_task(
            controller._wait_for_answer_async("123", answer_event=cancelled_event)
        )
        remaining_waiter = asyncio.create_task(
            controller._wait_for_answer_async("123", answer_event=remaining_event)
        )
        await asyncio.sleep(0)
        cancelled_waiter.cancel()
        try:
            await cancelled_waiter
        except asyncio.CancelledError:
            pass

        assert controller.answer_events["123"] == [remaining_event]
        controller._parse_message("CRISTART 1234 CMDACK 123 CRIEND")
        assert await remaining_waiter is None
        assert controller.answer_events == {}

    asyncio.run(scenario())


def test_timeout_removes_fixed_key_event_by_identity():
    async def scenario():
        controller = CRIController()
        event = controller._register_answer("info_filelist")

        with pytest.raises(CRICommandTimeOutError):
            await controller._wait_for_answer_async(SentCommand(42, event), timeout=0.0)

        assert controller.answer_events == {}

    asyncio.run(scenario())


def test_concurrent_move_waiters_share_completion_notification(monkeypatch):
    async def scenario():
        controller = CRIController()
        next_message_id = iter((10, 11))

        def send_command(command: str, register_answer: bool) -> SentCommand:
            message_id = next(next_message_id)
            answer_event = controller._register_answer(str(message_id))
            controller._parse_message(f"CRISTART 1234 CMDACK {message_id} CRIEND")
            return SentCommand(message_id, answer_event)

        monkeypatch.setattr(controller, "_send_command", send_command)
        waiters = [
            asyncio.create_task(
                controller._send_move_command_and_wait(
                    "CMD Move Joint", "MOVETOEXECEND", True, 1.0
                )
            )
            for _ in range(2)
        ]

        for _ in range(100):
            if len(controller.answer_events.get("MOVETOEXECEND", [])) == 2:
                break
            await asyncio.sleep(0.001)

        assert len(controller.answer_events["MOVETOEXECEND"]) == 2
        controller._parse_message("CRISTART 1234 MOVETOEXECEND 0 0 CRIEND")
        assert await asyncio.gather(*waiters) == [(None, None), (None, None)]
        assert controller.answer_events == {}

    asyncio.run(scenario())


def test_parse_cmderror():
    test_message = "CRISTART 1234 CMDERROR 123 There was an exception CRIEND"

    controller = CRIController()
    event = register_test_event(controller, "123")
    controller._parse_message(test_message)

    assert event.is_set()
    assert controller.error_messages[event] == "There was an exception"


def test_info_referencinginfo():
    test_message = "CRISTART 1234 INFO ReferencingInfo 1 Joints 1 1 1 1 2 0 0 0 0 0 0 0 Mandatory 1 RefWithProg 1 1 CRIEND"

    ref_state = ReferencingState(
        global_state=ReferencingAxisState.REFERENCED,
        A1=ReferencingAxisState.REFERENCED,
        A2=ReferencingAxisState.REFERENCED,
        A3=ReferencingAxisState.REFERENCED,
        A4=ReferencingAxisState.REFERENCED,
        A5=ReferencingAxisState.REFERENCING,
        A6=ReferencingAxisState.NOT_REFERENCED,
        E1=ReferencingAxisState.NOT_REFERENCED,
        E2=ReferencingAxisState.NOT_REFERENCED,
        E3=ReferencingAxisState.NOT_REFERENCED,
        E4=ReferencingAxisState.NOT_REFERENCED,
        E5=ReferencingAxisState.NOT_REFERENCED,
        E6=ReferencingAxisState.NOT_REFERENCED,
        mandatory=True,
        ref_prog_enabled=True,
        ref_prog_running=True,
    )

    controller = CRIController()
    controller._parse_message(test_message)

    assert controller.robot_state.referencing_state == ref_state


def test_regr_info_referencinginfo_missing_space_mandatory():
    """Regression test for missing space befor 'mandatory' in ReferencingInfo message."""
    test_message = "CRISTART 1234 INFO ReferencingInfo 1 Joints 1 1 1 1 2 0 0 0 0 0 0 0Mandatory 1 RefWithProg 1 1 CRIEND"

    ref_state = ReferencingState(
        global_state=ReferencingAxisState.REFERENCED,
        A1=ReferencingAxisState.REFERENCED,
        A2=ReferencingAxisState.REFERENCED,
        A3=ReferencingAxisState.REFERENCED,
        A4=ReferencingAxisState.REFERENCED,
        A5=ReferencingAxisState.REFERENCING,
        A6=ReferencingAxisState.NOT_REFERENCED,
        E1=ReferencingAxisState.NOT_REFERENCED,
        E2=ReferencingAxisState.NOT_REFERENCED,
        E3=ReferencingAxisState.NOT_REFERENCED,
        E4=ReferencingAxisState.NOT_REFERENCED,
        E5=ReferencingAxisState.NOT_REFERENCED,
        E6=ReferencingAxisState.NOT_REFERENCED,
        mandatory=True,
        ref_prog_enabled=True,
        ref_prog_running=True,
    )

    controller = CRIController()
    controller._parse_message(test_message)

    assert controller.robot_state.referencing_state == ref_state


def test_parse_execend():
    test_message = "CRISTART 1234 EXECEND 0 0 CRIEND"

    controller = CRIController()
    event = register_test_event(controller, "EXECEND")
    controller._parse_message(test_message)

    assert event.is_set()


def test_parse_execerror():
    test_message = "CRISTART 67 EXECERROR 0 0 PGLinear exception: 'Out of reach' CRIEND"

    controller = CRIController()
    event = register_test_event(controller, "EXECEND")
    controller._parse_message(test_message)

    assert event.is_set()
    assert controller.error_messages[event] == "0 0 PGLinear exception: 'Out of reach'"


def test_parse_can_brdige():
    test_message = "CRISTART 123 CANBridge Msg ID 32 Len 5 Data 0 1 2 3 4 5 6 7 Time 0 SystemTime 456789 CRIEND"

    can_message = {
        "id": 32,
        "length": 5,
        "data": bytearray([0, 1, 2, 3, 4, 5, 6, 7]),
        "time": 0,
        "system_time": 456789,
    }

    controller = CRIController()
    event = register_test_event(controller, "CAN")
    controller._parse_message(test_message)

    assert event.is_set()
    assert controller.can_queue.get_nowait() == can_message


def test_info_boardtemps():
    test_message = "CRISTART 6789 INFO BoardTemp 21.1 22.2 23.3 24.4 25.5 26.6 27.7 28.8 29.9 30.0 31.1 32.2 33.3 34.4 35.5 36.6 CRIEND"

    board_temps = [
        21.1,
        22.2,
        23.3,
        24.4,
        25.5,
        26.6,
        27.7,
        28.8,
        29.9,
        30.0,
        31.1,
        32.2,
        33.3,
        34.4,
        35.5,
        36.6,
    ]

    controller = CRIController()
    event = register_test_event(controller, "info_boardtemp")
    controller._parse_message(test_message)

    assert event.is_set()
    assert controller.robot_state.board_temps == pytest.approx(board_temps)


def test_info_motortemps():
    test_message = "CRISTART 6789 INFO MotorTemp 21.1 22.2 23.3 24.4 25.5 26.6 27.7 28.8 29.9 30.0 31.1 32.2 33.3 34.4 35.5 36.6 CRIEND"

    motor_temps = [
        21.1,
        22.2,
        23.3,
        24.4,
        25.5,
        26.6,
        27.7,
        28.8,
        29.9,
        30.0,
        31.1,
        32.2,
        33.3,
        34.4,
        35.5,
        36.6,
    ]

    controller = CRIController()
    event = register_test_event(controller, "info_motortemp")
    controller._parse_message(test_message)

    assert event.is_set()
    assert controller.robot_state.motor_temps == pytest.approx(motor_temps)


def test_list_files():
    test_message = (
        "CRISTART 1234 INFO FileList BaseDir FirstFile.xml SecondFile.txt CRIEND"
    )

    controller = CRIController()
    event = register_test_event(controller, "info_filelist")
    controller._parse_message(test_message)

    assert event.is_set()
    assert "FirstFile.xml" in controller.file_list
    assert "SecondFile.txt" in controller.file_list
