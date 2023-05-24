#!/usr/bin/env python3

import rclpy
import rclpy.logging
import rclpy.node

from clearpath_config_live.clearpath_config_handler import (
    ClearpathConfigHandler
)
from clearpath_config_live.clearpath_config_watcher import (
    ClearpathConfigWatcher
)
from clearpath_config_live.robot_description_client import (
    RobotDescriptionClient
)


def main():
    # Node
    rclpy.init()
    node = rclpy.create_node("clearpath_config_live")
    node.declare_parameter("config_file", "/etc/clearpath/test_robot.yaml")
    logger = rclpy.logging.get_logger("clearpath_config_live")

    # Get File
    config_param = node.get_parameter("config_file")
    config_file = config_param.get_parameter_value().string_value

    # Watcher, Handler, and Client
    client = RobotDescriptionClient(
        node,
        "robot_state_publisher",
        "robot_description"
    )
    watcher = ClearpathConfigWatcher(config_file, logger)
    handler = ClearpathConfigHandler(watcher, client, logger)

    # Start Handler
    try:
        watcher.start(handler)
    except Exception as ex:
        logger.error("Live updater failed: \n")
        logger.error(ex.args)

    # Spin
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Live updater exited with keyboard interrupt.")
    finally:
        watcher.stop()


if __name__ == "__main__":
    main()
