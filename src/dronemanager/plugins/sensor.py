"""Plugin and abstract base class for external sensors, such as weather sensors.

Similar to missions, these are a special type of plugin with extra functions to support their intended purpose.
The core extra component is the :py:meth:`~dronemanager.plugins.sensor.Sensor.get_data` function, which should
return whatever data the sensor provides.
Like missions, sensor modules go into their own special folder ``sensors``. Each sensor module can contain exactly one
specific sensor plugin, which must subclass :py:class:`~dronemanager.plugins.sensor.Sensor` and end with "Sensor".
Also like missions, they make use of the ``name`` parameter in their constructors to allow for multiple sensors of the
same class.

They're intended specifically for plugins that connect to and sporadically query some external data source.
Data sources that are processed continuously should not use the :py:class:`Sensor <dronemanager.plugins.sensor.Sensor>`
class, instead implementing their own processing loop, preferably in a separate thread or process.
"""
import abc
import logging
import pathlib
from typing import Callable

import dronemanager.core
from dronemanager.plugin import Plugin, MetaPlugin
from dronemanager.utils import SRC_DIR, DOC_DIR


class Sensor(Plugin, abc.ABC):
    """Abstract base class for specific sensor plugins.

    This is a special subtype of plugin with automatic discovery and abstract methods to support appropriate modules.
    The main change is that sensors should provide a default value for the ``name`` parameter in the constructor, which
    is used to overwrite the PREFIX by default. Multiple instances of the same sensor can then be added by using
    different names for each.

    Implementing subclasses should call :py:meth:`__init__` at the start of their constructors.

    Key functions are:

    * :py:meth:`connect`: Connect to the sensor, for example through UDP.
    * :py:meth:`get_data`: Query the sensor for current data.
    * :py:meth:`disconnect`: Disconnect from the sensor. This should not shut down the sensor plugin itself.
    * :py:meth:`status`: Should write status information to the logger.

    Key attributes are:

    * :py:attr:`last_data`: A copy of the last received data object for caching purposes. Functions or modules may
      access this copy instead of requesting new data if timeliness is not important or requesting data is expensive.

    Like normal plugins they can define dependencies.
    """

    PREFIX: str = "YOUDIDSOMETHINGWRONG"
    """Prefix used for this plugin.

    Implementing classes can ignore this, as it should be overwritten with the name parameter anyway.

    :meta hide-value:"""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str = "YOUDIDSOMETHINGWRONG"):
        """Abstract constructor.

        Args:
            dm: The associated DroneManager instance
            logger: The logger for output and errors.
            name: The name of the sensor plugin. Takes the place of the prefix for CLI commands.
        """
        super().__init__(dm, logger, name)
        self.PREFIX: str = name  # Set the prefix to the name attribute.
        #: Dictionary of available CLI commands.
        #: Implementing subclasses should add, but not remove from this.
        self.cli_commands: dict[str, Callable] = {
            "connect": self.connect,
            "data": self.log_data,
            "status": self.status,
            "disconnect": self.disconnect,
            "reconnect": self.reconnect,
        }

        self.connect_args: any = None  #: Connection args for convenient reconnect.
        self.connect_kwargs: any = None  #: Connection kwargs for convenient reconnect.

        #: The last received/collected data should be stored here for others to use without access delay
        self.last_data: any = None

    async def start(self):
        """This function is called once the sensor plugin is loaded.

        By default, just starts the registered background functions.
        """
        await super().start()

    async def close(self):
        """Shutdown function for sensors.

        By default, disconnects from the sensor and cancels all tasks in self.running_tasks.
        """
        await self.disconnect()
        await super().close()

    @abc.abstractmethod
    async def connect(self, *args: any, **kwargs: any) -> bool:
        """Connect to a sensor.

        Implementing classes should call this at the start of their connect function. The args and kwargs here are used
        to store the connection arguments to allow a blank "reconnect".

        Implementing classes should return whether they were able to connect.

        Args:
            *args: Connection args, stored for reconnect.
            **kwargs: Connection kwargs, stored for reconnect.

        Returns:
            True, as a dummy return value.
        """
        self.connect_args = args
        self.connect_kwargs = kwargs
        return True

    @abc.abstractmethod
    async def get_data(self) -> object | None:
        """Should return whatever information the sensor provides.

        Should return ``None`` if there was an error or the sensor is not available.
        Return values can be any arbitrary type, but should at least have a nice string representation for logging
        purposes.

        Returns:
            The data or ``None``.
        """
        ...

    async def log_data(self):
        """Write data to the logger."""
        self.logger.info(await self.get_data())

    @abc.abstractmethod
    async def status(self):
        """Should write information about the current status of the sensor to the logger under INFO."""
        ...

    @abc.abstractmethod
    async def disconnect(self):
        """Disconnect from a sensor. Should handle any socket clearing etc.

        Implementations should be safe to be called repeatedly.
        """
        ...

    async def reconnect(self):
        """Disconnect and then reconnect from a sensor."""
        await self.disconnect()
        await self.connect(*self.connect_args, **self.connect_kwargs)


class SensorPlugin(MetaPlugin):
    """This plugin handles loading and management of sensor plugins.

    This plugin does not represent the sensors themselves, and implementation of a new sensor-type plugin should NOT
    subclass this plugin, but :py:class:`Sensor`.

    This plugin has three CLI commands:

    * "load" - :py:meth:`load`: Load a new sensor, optionally with a custom name to have multiple sensors of the same
      type.
    * "unload" - :py:meth:`unload`: Unload, i.e. close a sensor.
    * "status" - :py:meth:`status`: Log information about currently loaded sensors, by calling
      :py:meth:`Sensor.status>` for each connected sensor. Also lists sensors that could be loaded.
    """

    EXAMPLE_DIR: pathlib.Path = SRC_DIR.joinpath("sensors")
    """Directory in the source tree with example modules.

    :meta hide-value:"""

    USER_DIR: pathlib.Path = DOC_DIR.joinpath("sensors")
    """Directory in the DroneManager install directory where the sub-plugins should be located.

    :meta hide-value:"""

    VALID_CLASS_SUFFIX: str = "Sensor"
    """Valid sub-plugins must have class names ending with this string."""

    NAMESPACE: str = "sensors"
    """Modules with sub-plugins have this prepended to their import to reduce collisions."""

    SUBTYPE: type = Sensor
    """The type that sub-plugins must subclass to be valid."""

    PREFIX: str = "sensor"
    """The prefix for the CLI commands, "sensor" by default."""

    def __init__(self, dm: dronemanager.core.DroneManager, logger: logging.Logger, name: str):
        """Create the SensorPlugin.

        Args:
            dm: The associated DroneManager instance.
            logger: The logger used for status info and errors.
            name: The name of the plugin.
        """
        super().__init__(dm, logger, name)
        #: Available cli commands.
        self.cli_commands: dict[str, Callable] = {
            "load": self.load,
            "unload": self.unload,
            "status": self.status,
        }

    async def status(self):
        """Status of loaded sensors and sensors that could be loaded."""
        self.logger.info("Status of loaded sensors:")
        for _, sensor in self._loaded.items():
            await sensor.status()
        if len(self._loaded) == 0:
            self.logger.info("No loaded sensors!")
        self.logger.info(f"Available sensors for loading: {self.plugin_options()}")
