import logging
from time import sleep

from cri_lib import CRIController

# 🔹 Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# CRIController is the main interface for controlling the iRC
controller = CRIController()

# Connect to default iRC IP
# controller.connect("192.168.3.11")
if not controller.connect("127.0.0.1", 3921):
    logger.error("Unable to connect to iRC! Ensure the simulator is running.")
    quit()

# Acquire active control.
# not necessary for this example
# controller.set_active_control(True)

# 10 times in 500ms intervals
for i in range(0, 10):
    sleep(0.5)
    j = controller.robot_state.joints_set_point
    logger.info(
        f"Axis positions: A1={j.A1} A2={j.A2} A3={j.A3} A4={j.A4} A5={j.A5} A6={j.A6} E1={j.E1} E2={j.E2} E3={j.E3} T1={j.G1} T2={j.G2} T3={j.G3} P1={j.P1} P2={j.P2} P3={j.P3} P4={j.P4}"
    )

# Disable motors and disconnect
logger.info("Disconnecting...")
controller.close()

logger.info("Script execution completed successfully.")
