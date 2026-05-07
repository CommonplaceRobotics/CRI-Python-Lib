import copy
import threading

import pytest

from cri_lib import (
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
)


def robot_state_equal(actual: RobotState, expected: RobotState) -> bool:
    """Compare robot state without requiring update nanosecond values to match."""
    if set(actual.category_time_ns) != set(expected.category_time_ns):
        return False
    # copy the timestamp values to facilitate equals comparison of the dataclasses
    expected_copy = copy.deepcopy(expected)
    expected_copy.category_time_ns.update(actual.category_time_ns)
    return actual == expected_copy


def test_parse_state():
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
    controller._parse_message(test_message)

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
    controller.answer_events["123"] = threading.Event()
    controller._parse_message(test_message)

    assert controller.answer_events["123"].is_set()


def test_parse_cmderror():
    test_message = "CRISTART 1234 CMDERROR 123 There was an exception CRIEND"

    controller = CRIController()
    controller.answer_events["123"] = threading.Event()
    controller._parse_message(test_message)

    assert controller.answer_events["123"].is_set()
    assert controller.error_messages["123"] == "There was an exception"


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
    controller.answer_events["EXECEND"] = threading.Event()
    controller._parse_message(test_message)

    assert controller.answer_events["EXECEND"].is_set()


def test_parse_execerror():
    test_message = "CRISTART 67 EXECERROR 0 0 PGLinear exception: 'Out of reach' CRIEND"

    controller = CRIController()
    controller.answer_events["EXECEND"] = threading.Event()
    controller._parse_message(test_message)

    assert controller.answer_events["EXECEND"].is_set()
    assert (
        controller.error_messages["EXECEND"] == "0 0 PGLinear exception: 'Out of reach'"
    )


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
    controller.answer_events["CAN"] = threading.Event()
    controller._parse_message(test_message)

    assert controller.answer_events["CAN"].is_set()
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
    controller.answer_events["info_boardtemp"] = threading.Event()
    controller._parse_message(test_message)

    assert controller.answer_events["info_boardtemp"].is_set()
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
    controller.answer_events["info_motortemp"] = threading.Event()
    controller._parse_message(test_message)

    assert controller.answer_events["info_motortemp"].is_set()
    assert controller.robot_state.motor_temps == pytest.approx(motor_temps)


def test_list_files():
    test_message = (
        "CRISTART 1234 INFO FileList BaseDir FirstFile.xml SecondFile.txt CRIEND"
    )

    controller = CRIController()
    controller.answer_events["info_filelist"] = threading.Event()
    controller._parse_message(test_message)

    assert controller.answer_events["info_filelist"].is_set()
    assert "FirstFile.xml" in controller.file_list
    assert "SecondFile.txt" in controller.file_list
