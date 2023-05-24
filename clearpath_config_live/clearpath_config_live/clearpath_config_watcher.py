from clearpath_config_live.clearpath_config_updater import (
    ClearpathConfigUpdater
)
from typing import Set
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer


class ClearpathConfigWatcher:

    def __init__(self, config_file: str, logger) -> None:
        self.observer = Observer()
        self.updater = ClearpathConfigUpdater(config_file)
        self.logger = logger

    @property
    def watched(self) -> Set[str]:
        """Get Directories being Watched"""
        return {emitter.watch.path for emitter in self.observer.emitters}

    def start(self, event_handler: FileSystemEventHandler) -> None:
        """Start Tracking Clearpath Config"""
        self.observer.start()
        self.update(event_handler)

    def stop(self) -> None:
        """Stop Tracking Clearpath Config"""
        self.observer.stop()
        self.observer.join()

    def update_watched(self, event_handler: FileSystemEventHandler) -> None:
        """Update Directories being Watched"""
        if self.watched != self.updater.dirs:
            self.observer.unschedule_all()
            for dir in self.updater.dirs:
                self.logger.info("Watching directory: %s" % dir)
                self.observer.schedule(event_handler,
                                       path=dir,
                                       recursive=False)

    def update(self, event_handler: FileSystemEventHandler) -> None:
        """Update Clearpath Config and Watchlist"""
        self.updater.update()
        self.update_watched(event_handler)
