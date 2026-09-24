import logging

from cri_lib import CRIController

# 🔹 Configure logging
logging.basicConfig(
    # Set to DEBUG to log all received CRI messages
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# CRIController is the main interface for controlling the iRC
controller = CRIController()

# Connect to default iRC IP
# Simulator
ip = "127.0.0.1"
port = 3922
# Real robot
# ip = "192.168.3.11"
# port = 3921
if not controller.connect(ip, port):
    logger.error("Unable to connect to iRC! Ensure the simulator is running.")
    quit()

# Acquire active control.
controller.set_active_control(True)

logger.info("Acquired active control.")

# Enable motors
logger.info("Enabling motors...")
controller.enable()

# Wait until kinematics are ready
logger.info("Waiting for kinematics to be ready...")
controller.wait_for_kinematics_ready(10)

controller.set_override(100.0)

# Perform relative movement
logger.info("Moving base relative: +20mm in X, Y, Z...")
controller.move_base_relative(
    20.0,
    20.0,
    20.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    10.0,
    wait_move_finished=True,
    move_finished_timeout=1000,
)

logger.info("Moving back: -20mm in X, Y, Z...")
controller.move_base_relative(
    -20.0,
    -20.0,
    -20.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    10.0,
    wait_move_finished=True,
    move_finished_timeout=1000,
)

# Disable motors and disconnect
logger.info("Disabling motors and disconnecting...")
controller.disable()
controller.close()

logger.info("Script execution completed successfully.")
