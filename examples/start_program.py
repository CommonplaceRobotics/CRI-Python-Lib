import logging
from time import sleep

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

# connect to default iRC IP
# Simulator
ip = "127.0.0.1"
port = 3922
# Real robot
# ip = "192.168.3.11"
# port = 3921
if not controller.connect(ip, port):
    logger.error("Unable to connect")
    quit()

# acquire active control.
controller.set_active_control(True)

logger.info("Enabling motors...")
# enable motors
controller.enable()

logger.info("Waiting for kinematics to be ready...")
# wait until kinematics are ready to move
controller.wait_for_kinematics_ready(10)

controller.set_override(50.0)

logger.info("Load program")
if not controller.load_programm("ReBeL_MoveToZero.xml"):
    logger.error("unable to load programm")
    controller.disable()
    controller.close()
    quit()

logger.info("Start program")
if not controller.start_programm():
    logger.error("Unable to start programm")
    controller.disable()
    controller.close()
    quit()

sleep(5)

logger.info("Pause program")
if not controller.pause_programm():
    logger.error("Unable to pause programm")
    controller.disable()
    controller.close()
    quit()

sleep(5)

logger.info("Start programm again")
if not controller.start_programm():
    logger.error("Unable to start programm")
    controller.disable()
    controller.close()
    quit()

sleep(5)

logger.info("Stop program")
if not controller.stop_programm():
    logger.error("Unable to stop programm")
    controller.disable()
    controller.close()
    quit()

# Disable motors and disconnect
controller.disable()
controller.close()
