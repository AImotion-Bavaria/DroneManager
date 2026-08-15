import abc
import collections
import enum
import pathlib

from dronemanager.plugin import Plugin, MetaPlugin
from dronemanager.utils import DM_INSTALL_DIR, SRC_DIR


class Mission(Plugin, abc.ABC):
    PREFIX = "YOUDIDSOMETHINGWRONG"

    def __init__(self, dm, logger, name="YOUDIDSOMETHINGWRONG"):
        super().__init__(dm, logger, name)
        self.PREFIX = name

        # CLI commands for these coroutines are generated automatically based on the signature and type hints.
        # They must be coroutines!
        self.cli_commands = {
            "reset": self.reset,
            "status": self.status,
            "add": self.add_drones,
            "remove": self.remove_drones,
        }

        # These attributes may be used by other part of the software, for example to determine the window size for a map
        self.current_stage: MissionStage | None = None  # For missions with multiple stages
        self.flight_area: FlightArea | None = None  # For missions with a defined flight area
        self.drones = collections.OrderedDict()  # The drones participating in the mission

        # A dictionary with any other information that might be useful for other parts of the library. The external
        # plugin automatically shares the information here.
        self.additional_info = {}

    async def start(self):
        """This function is called when the mission is loaded to start all the necessary processes asynchronously.

        It is NOT a "start this mission" function. By default, launches any background processes, like starting a
        plugin.
        """
        await super().start()

    async def close(self):
        """Shutdown function for the script. It should end any running tasks and clear any resources.

        By default, it cancels any tasks tracked in self.running_tasks.
        """
        await super().close()

    @abc.abstractmethod
    async def reset(self):
        """Resets the mission back to the initial position.

        Keep safety in mind when this requires moving drones.
        """
        raise NotImplementedError

    @abc.abstractmethod
    async def status(self):
        """Should write information about the current status of the mission to the logger under INFO."""
        raise NotImplementedError

    @abc.abstractmethod
    async def add_drones(self, names: list[str]):
        """Add drones to the mission. Implementations should check that the drones are capable and meet mission
        requirements.
        """
        raise NotImplementedError

    @abc.abstractmethod
    async def remove_drones(self, names: list[str]):
        """Remove drones from the mission. Implementations must take measures to prevent missions from running with
        too few drones
        """
        raise NotImplementedError

    @abc.abstractmethod
    async def mission_ready(self, drone: str):
        """Check whether any given drone is ready to keep going, i.e. is still connected etc."""
        raise NotImplementedError


class MissionStage(enum.Enum):
    pass


class FlightArea(abc.ABC):
    def __init__(self, *args, **kwargs):
        pass

    @property
    @abc.abstractmethod
    def x_min(self):
        pass

    @property
    @abc.abstractmethod
    def x_max(self):
        pass

    @property
    @abc.abstractmethod
    def y_min(self):
        pass

    @property
    @abc.abstractmethod
    def y_max(self):
        pass

    @property
    @abc.abstractmethod
    def z_min(self):
        pass

    @property
    @abc.abstractmethod
    def z_max(self):
        pass

    def bounding_box(self):
        return [self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max]


class MissionPlugin(MetaPlugin):

    EXAMPLE_DIR: pathlib.Path = SRC_DIR.joinpath("missions")
    """Directory in the source tree with example modules.

    :meta hide-value:"""

    USER_DIR: pathlib.Path = DM_INSTALL_DIR.joinpath("missions")
    """Directory in the DroneManager install directory where the sub-plugins should be located.

    :meta hide-value:"""

    VALID_CLASS_SUFFIX: str = "Mission"
    """Valid sub-plugins must have class names ending with this string.

    :meta hide-value:"""

    NAMESPACE: str = "missions"
    """Modules with subplugins have this prepended to their import to reduce collisions.

    :meta hide-value:"""

    SUBTYPE: type = Mission
    """The type that subplugins must subclass to be valid.

    :meta hide-value:"""

    PREFIX = "mission"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "load": self.load,
            "status": self.status,
        }

    async def status(self):
        """ Status of running missions and missions that could be loaded."""
        self.logger.info("Status of running missions:")
        for mission in self._loaded:
            await getattr(self, mission).status()
        if len(self._loaded) == 0:
            self.logger.info("No running missions!")
        self.logger.info(f"Available missions for loading: {self.plugin_options()}")
