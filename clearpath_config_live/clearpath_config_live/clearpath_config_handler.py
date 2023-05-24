from rclpy.task import Future
from watchdog.events import EVENT_TYPE_MODIFIED
from watchdog.events import FileSystemEventHandler

from clearpath_config_live.clearpath_config_watcher import (
    ClearpathConfigWatcher
)
from clearpath_config_live.robot_description_client import (
    RobotDescriptionClient
)


class ClearpathConfigHandler(FileSystemEventHandler):

    def __init__(
            self,
            watcher: ClearpathConfigWatcher,
            client: RobotDescriptionClient,
            logger
            ) -> None:
        self.watcher = watcher
        self.client = client
        self.future = Future()
        self.logger = logger

    def on_modified(self, event):
        """Check for Updates to Relevant Files in Directory"""
        if event.event_type == EVENT_TYPE_MODIFIED and not event.is_directory:
            if self.watcher.updater.is_file(event.src_path):
                try:
                    self.watcher.update(self)
                    self.future = self.client.call_async(
                        self.watcher.updater.get_robot_description()
                    )
                except Exception as ex:
                    self.future = Future()
                    self.logger.error("Updater failed to regenerate config:")
                    self.logger.error(str(ex))
