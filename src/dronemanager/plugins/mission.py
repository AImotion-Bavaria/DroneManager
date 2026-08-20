"""Plugin and abstract base class for missions.

Missions are intended to act as the central implementation point for practical, repeatable applications. A mission
might be a search-and-rescue demo showcasing a swarm of drones coordinating and finding some point of interest, or a
data collection mission, or a series of maneuvers to test a drone with a custom flight controller.

They allow custom scripts to be easily introduced into the CLI while retaining the core functionality of DroneManager.
Once a mission is implemented, loading all relevant components, including other plugins, can be as simple as
``mission-load <your-mission-file>``.

In terms of implementation, missions fundamentally work very similar to plugins, with a few key changes. They must be
subclasses of the :py:class:`Mission` abstract base class, they go into a folder "missions" and they have a
new attribute ``name``, which takes the role of the prefix used for plugins. Unlike with plugins, this is an instance
attribute, allowing multiple instances of the same mission class.

Loading a mission is done with ``mission-load <file> <name?>``, where <file> is the name of the file with the mission,
excluding the suffix, similar to plugins. The name argument is optional, by default the name of the file is used. The
mission plugin must be loaded first (done at startup by default). The custom name is used as the prefix for CLI
commands, and as the attribute for access from DroneManager.

Mission modules must be located in the "mission" directory and should only contain one mission class each. They can
contain any number of non-mission classes.

Compared to normal plugins, there are 4 functions which missions must provide. These are:

    * :py:meth:`add_drones`: Adds drones to the mission.
    * :py:meth:`remove_drones`: Removes drones from the mission.
    * :py:meth:`status`: Should write status information to the logger.
    * :py:meth:`reset`: Resets the mission. The interpretation of what constitutes a mission is up to the implementing
      subclasses

They also come with four attributes:

    * :py:attr:`current_stage`: The current stage for missions with stages. Missions can and should also implement
      their own stage types.
    * :py:attr:`flight_area`: The flight area for mission with flight areas.
    * :py:attr:`drones`: An ordered dictionary of the drones participating in the mission.
    * :py:attr:`additional_info`: A dictionary with any other information that might be useful for other parts of the
      library. The plugin :py:class:`~dronemanager.plugins.external.UDPPlugin` automatically shares the information
      here. Keys should be strings and any items must have a string representation.

Every other aspect of the implementation is up to the subclasses.
"""
import abc
import collections
from collections.abc import Callable
import enum
import logging
import pathlib
from typing import Any

import dronemanager.core
from dronemanager.plugin import Plugin, MetaPlugin
from dronemanager.utils import DM_INSTALL_DIR, SRC_DIR


class Mission(Plugin, abc.ABC):
    """Abstract base class for mission plugins.

    This is a special subtype of plugin with automatic discovery and abstract methods to support appropriate modules.
    Missions should provide a default value for the ``name`` parameter in the constructor, which is used to overwrite
    the PREFIX by default. Multiple instances of the same mission can then be added by using different names for each.

    Implementing subclasses should call :py:meth:`__init__` at the start of their constructors.

    Key functions are:

    * :py:meth:`add_drones`: Add drones to the mission.
    * :py:meth:`remove_drones`: Remove drones from the mission.
    * :py:meth:`status`: Should write status information to the logger.
    * :py:meth:`reset`: Reset the mission. The interpretation of what constitutes a mission is up to the implementing
      subclasses

    Key attributes are:

    * :py:attr:`current_stage`: The current stage for missions with stages. Missions can and should also implement
      their own stage types.
    * :py:attr:`flight_area`: The flight area for mission with flight areas.
    * :py:attr:`drones`: An ordered dictionary of the drones participating in the mission.
    * :py:attr:`additional_info`: A dictionary with any other information that might be useful for other parts of the
      library. The plugin :py:class:`~dronemanager.plugins.external.UDPPlugin` automatically shares the information
      here. Keys should be strings and any items must have a string representation.

    Like normal plugins they can define dependencies, which are then loaded automatically when the mission is loaded.
    """

    PREFIX: str = "YOUDIDSOMETHINGWRONG"
    """The prefix for the CLI commands. Can be ignored as it is overridden by the name parameter."""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str = "YOUDIDSOMETHINGWRONG"):
        """Create a mission instance.

        Args:
            dm: The associated DroneManager instance.
            logger: The logger for output and errors.
            name: The name of the mission. Takes the place of the prefix for CLI commands.
        """
        super().__init__(dm, logger, name)
        self.PREFIX: str = name  #: The prefix for the CLI commands.

        # CLI commands for these coroutines are generated automatically based on the signature and type hints.
        # They must be coroutines!
        #: Available CLI commands. Implementing subclasses can extend this collection.
        self.cli_commands: dict[str, Callable] = {
            "reset": self.reset,
            "status": self.status,
            "add": self.add_drones,
            "remove": self.remove_drones,
        }

        # These attributes may be used by other part of the software, for example to determine the window size for a map
        self.current_stage: MissionStage | None = None  #: The current stage for missions with stages.
        self.flight_area: FlightArea | None = None  #: The flight area for mission with flight areas.
        #: An ordered dictionary of the drones participating in the mission.
        self.drones: dict[str, dronemanager.core.Drone] = collections.OrderedDict()

        #: A dictionary with any other information that might be useful for other parts of the library. The plugin
        #: "external" automatically shares the information here. Any items must have a string representation.
        self.additional_info: dict[str, Any] = {}

    async def start(self):
        """This function is called when the mission is loaded to start the plugin.

        It is NOT a "start this mission" function.
        By default, launches all declared background functions.
        """
        await super().start()

    async def close(self):
        """Shutdown function for the mission.

        By default, cancels all tasks tracked in :py:attr:`self.running_tasks`.
        """
        await super().close()

    @abc.abstractmethod
    async def status(self):
        """Should write information about the current status of the mission to the logger with loglevel "INFO".

        Raises:
            NotImplementedError: Must be overridden.
        """
        raise NotImplementedError

    @abc.abstractmethod
    async def add_drones(self, names: list[str]) -> bool:
        """Add drones to the mission.

        Args:
            names: The list of connected drones to add to the mission.

        Raises:
            NotImplementedError: Must be overridden.

        Returns:
            ``True`` if the drones were successfully added, ``False`` otherwise.
        """
        raise NotImplementedError

    @abc.abstractmethod
    async def remove_drones(self, names: list[str]) -> bool:
        """Remove drones from the mission.

        Args:
            names: A list of drones to remove from the mission.

        Raises:
            NotImplementedError: Must be overridden.

        Returns:
            ``True`` if the drones were successfully removed, ``False`` otherwise.
        """
        raise NotImplementedError

    @abc.abstractmethod
    async def reset(self):
        """Resets the mission back to the initial state.

        The exact interpretation of what constitutes a "reset" is left to the implementation. For a data collection
        mission, this might just reset various sensors and meta, or it might move the drone back to the launch position.

        Keep safety in mind when this requires moving drones.

        Raises:
            NotImplementedError: Must be overridden.
        """
        raise NotImplementedError


class MissionStage(enum.Enum):
    """Abstract base class for mission stages."""


class FlightArea(abc.ABC):
    """Abstract base class for flight area classes.

    The only requirement from the rest of the library is that implementing subclasses must be able to provide a
    bounding box.
    """

    @property
    @abc.abstractmethod
    def x_min(self) -> float:
        """The lower limit along the "x" axis (usually "forward" or "north")."""
        pass

    @property
    @abc.abstractmethod
    def x_max(self) -> float:
        """The upper limit along the "x" axis (usually "forward" or "north")."""
        pass

    @property
    @abc.abstractmethod
    def y_min(self) -> float:
        """The lower limit along the "y" axis (usually "right" or "east")."""
        pass

    @property
    @abc.abstractmethod
    def y_max(self) -> float:
        """The upper limit along the "y" axis (usually "right" or "east")."""
        pass

    @property
    @abc.abstractmethod
    def z_min(self) -> float:
        """The lower limit along the "z" axis (usually "down")."""
        pass

    @property
    @abc.abstractmethod
    def z_max(self) -> float:
        """The upper limit along the "z" axis (usually "down")."""
        pass

    @property
    def bounding_box(self) -> tuple[float, float, float, float, float, float]:
        """The axis aligned bounding box of the flight area.

        Returns:
            The lower and upper limits for each axis.
        """
        return self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max


class MissionPlugin(MetaPlugin):
    """This plugin handles loading and management of mission plugins.

    This plugin does not represent the missions themselves, and implementation of a new mission-type plugin should NOT
    subclass this plugin, but :py:class:`Mission`.

    This plugin has three CLI commands:

    * "load" - :py:meth:`load`: Load a new mission, optionally with a custom name to have multiple missions of the same
      type.
    * "unload" - :py:meth:`unload`: Unload, i.e. close a mission.
    * "status" - :py:meth:`status`: Log information about currently loaded sensors, by calling
      :py:meth:`Mission.status>` for each loaded mission. Also lists missions that could be loaded.
    """

    EXAMPLE_DIR: pathlib.Path = SRC_DIR.joinpath("missions")
    """Directory in the source tree with example modules.

    :meta hide-value:"""

    USER_DIR: pathlib.Path = DM_INSTALL_DIR.joinpath("missions")
    """Directory in the DroneManager install directory where the sub-plugins should be located.

    :meta hide-value:"""

    VALID_CLASS_SUFFIX: str = "Mission"
    """Valid sub-plugins must have class names ending with this string."""

    NAMESPACE: str = "missions"
    """Modules with subplugins have this prepended to their import to reduce collisions."""

    SUBTYPE: type = Mission
    """The type that subplugins must subclass to be valid."""

    PREFIX = "mission"
    """The prefix for the CLI commands, "mission" by default."""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str):
        """Create the MissionPlugin.

        Args:
            dm: The DroneManager instance associated with this plugin.
            logger: The logger for output or errors.
            name: The name of this plugin.
        """
        super().__init__(dm, logger, name)
        #: Available CLI commands.
        self.cli_commands: dict[str, Callable] = {
            "load": self.load,
            "unload": self.unload,
            "status": self.status,
        }

    async def status(self):
        """Prints status information for running missions and lists potential mission files."""
        self.logger.info("Status of running missions:")
        for mission in self._loaded:
            await getattr(self, mission).status()
        if len(self._loaded) == 0:
            self.logger.info("No running missions!")
        self.logger.info(f"Available missions for loading: {self.plugin_options()}")
