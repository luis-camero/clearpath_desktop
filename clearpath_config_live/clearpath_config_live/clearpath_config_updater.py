import os
import xacro
from clearpath_config.common import File
from clearpath_description_generator.description_generator import (
    DescriptionGenerator
)


class ClearpathConfigUpdater:

    def __init__(
            self,
            config_file: str,
            output_path: str = "/etc/clearpath",
            ) -> None:
        self.config_file = os.path.realpath(config_file)
        self.output_path = os.path.realpath(output_path)
        self.dirs = {os.path.dirname(self.config_file)}
        self.doc = None

    def get_robot_description(self):
        """Get Robot Description Parameter"""
        return self.doc.toprettyxml(indent="  ")

    def is_file(self, path: str) -> bool:
        """Check if File is the Same"""
        return os.path.realpath(path) == self.config_file

    def update(self) -> None:
        """Re-load File and Create Description"""
        # Generate URDF
        dg = DescriptionGenerator(
            config=File(self.config_file),
            output_path=self.output_path
        )
        dg.generate()

        # Re-load Description
        self.doc = xacro.process_file(
            os.path.join(self.output_path, "robot.urdf.xacro")
        )
