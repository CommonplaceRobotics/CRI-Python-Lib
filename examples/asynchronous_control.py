"""Example of asynchronous control of an iRC robot.

This is relevant for applications that need to perform other tasks concurrently while controlling the robot,
for example monitoring other inputs, controlling other actuators, or coordinating with other systems.
"""

import asyncio
import logging

from cri_lib import CRIConnector

logging.basicConfig(
    # Set to DEBUG to log all received CRI messages
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def main():
    # The connector creates passive or active control sessions with proper resource management.
    # Simulator
    connector = CRIConnector(
        host="127.0.0.1",
        port=3922,
    )
    # Real robot
    # connector = CRIConnector(
    #     host="192.168.3.11",
    #     port=3921,
    # )

    # connect asynchronously
    async with connector.observe() as client:
        logger.info("Current state is: %s", client.robot_state)
    # disconnect automatically when exiting the context

    # connect and take control
    async with connector.control(auto_disable=False) as controller:
        await controller.set_override_async(100.0)

        # Perform relative movement
        logger.info("Moving base relative: +20mm in X, Y, Z...")
        await controller.move_base_relative_async(
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
        await controller.move_base_relative_async(
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
    # disconnect and release control automatically by exiting the context

    logger.info("Script execution completed successfully.")
    return


if __name__ == "__main__":
    asyncio.run(main())
