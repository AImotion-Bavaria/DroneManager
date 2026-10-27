import abc
import asyncio

from dronemanager.utils import cancel_running_tasks

class DroneComProtocol(abc.ABC):
    """ABC for drone communication protocols, such as MAVLink."""
    def __init__(self, logger, *args, **kwargs):
        self.logger = logger.getChild(self.__class__.__name__)
        self.running_tasks: set[asyncio.Task] = set()

    def close(self):
        """Closes the communication protocol.

        Implementing classes should call this during their close calls.
        """
        cancel_running_tasks(self.running_tasks)