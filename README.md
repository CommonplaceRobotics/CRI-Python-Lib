# Introduction 
Python package to interface an igus Robot Control via the CRI protocol.

# Current Features
- Robot State
- Basic functions
    - Reset
    - Enable / Disable
    - Acquisition of active control
    - Override
- Referencing of 
    - single axis
    - all axes
    - Set joints to zero
- Direct movements
    - Joint and relative joint
    - Cartesian
    - Cartesian base and tool relative
- Live jog
- Digital IO and global signals
- Programs
    - Upload
    - Start / Pause / Stop
- CAN Bridge

# Getting Started
## Installation
Using the library requires no additional libraries, only testing has external dependencies.

In the top directory of this repository execute `pip install .`

## Usage
The library provides the `CRIController` class which is the main interface to controll the iRC. It handles the network connection and starts background threads for keep alive messages and processing of received messages. Movement functions directly return even though the move might not be finisched. If you want to wait for the move to finish, set the `wait_move_finished` parameter to `True`.
Most function return a bool to check whether the execution was successful.

### Typical procedure
1. Connect to iRC: `CRIController.connect(...)` Default IP is `192.168.3.11` with port `3921`. For using the simulation in the iRC desktop software, connect to `127.0.0.1`, most of the time the port in the simulation is `3921`, but can be different. Have a look at the log if you are unable to connect. Check whether connection was successfull via the returned bool.
2. If you want to control the robot, acquire active control via `CRIController.set_active_control(True)`.
3. Enable drives with `CRIController.enable()`
4. Wait unitl axes are ready `CRIcontroller.wait_for_kinematics_ready()`
5. Do some work with your robot, see API documentation for all functionality.
6. Diable drives with `CRIController.disable()`
7. Close connection with `CRIController.close()`

## Examples
See `examples` directory.

# Tests
This repository provides pytests for the message parser. They require `pytest` and `pytest-cov`, which can be installed via `pip install -r requirements.txt`. To run the tests (including coverage) execute the following command: `pytest -vv --cov=cri_lib --cov-report term-missing tests`. 

