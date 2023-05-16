import rclpy
import rclpy.logging
import rclpy.node

from clearpath_config_live.robot_description_client import (
    RobotDescriptionClient
)

from clearpath_common.clearpath_description_generator.clearpath_description_generator.description_generator import DescriptionGenerator

import xacro


def main():
    rclpy.init()
    node = rclpy.create_node("clearpath_config_live")
    client = RobotDescriptionClient(node, "robot_state_publisher")

    # Description Generation
    dg = DescriptionGenerator(
        config="/etc/clearpath/test_robot.yaml",
        output_path="/etc/clearpath",
    )

    dg.generate()

    # Re-load Description
    urdf = xacro.process_file("/etc/clearpath/robot.urdf.xacro")
    client.call_async(urdf)
