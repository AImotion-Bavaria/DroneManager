""" Plugin and abstract base class for external sensors, such as weather sensors.

They're specifically for plugins that connect to and sporadically query some external data source.

Similar to missions, these are a special type of plugin with extra functions to support their intended purpose.
The core extra component is the :py:meth:`~dronemanager.plugins.sensor.Sensor.get_data` function, which should
return whatever data the sensor provides.
Like missions, sensor modules go into their own special folder ``sensors``. Each sensor module can contain exactly one
specific sensor plugin, which must subclass :py:class:`~dronemanager.plugins.sensor.Sensor` and end with "Sensor"

The sensor get_data function may be slow so sources that should be processed continuously should not be
implemented as a sensor.
"""
import abc
import pathlib

from dronemanager.plugin import Plugin, MetaPlugin
from dronemanager.utils import SRC_DIR, DM_INSTALL_DIR


class Sensor(Plugin, abc.ABC):
    PREFIX = "YOUDIDSOMETHINGWRONG"

    def __init__(self, dm, logger, name="YOUDIDSOMETHINGWRONG"):
        super().__init__(dm, logger, name)
        self.PREFIX = name
        self.cli_commands = {
            "connect": self.connect,
            "data": self.log_data,
            "status": self.status,
            "disconnect": self.disconnect,
            "reconnect": self.reconnect,
        }

        # Connection arguments for convenient reconnect.
        self.connect_args = None
        self.connect_kwargs = None

        # The last received/collected data should be stored here for others to use without access delay
        self.last_data = None

    async def start(self):
        """ This function is called when the sensor plugin is loaded to automatically start any background functions.
        """
        await super().start()

    async def close(self):
        """ This function must end all running asyncio tasks. By default, all tasks in self._running_tasks are
        cancelled."""
        await self.disconnect()
        await super().close()

    @abc.abstractmethod
    async def connect(self, *args, **kwargs):
        """ Connect to a sensor."""
        self.connect_args = args
        self.connect_kwargs = kwargs

    @abc.abstractmethod
    async def get_data(self):
        """ Should return whatever information the sensor provides,"""
        pass

    async def log_data(self):
        self.logger.info(await self.get_data())

    @abc.abstractmethod
    async def status(self):
        """ Should write information about the current status of the sensor to the logger under INFO."""
        pass

    @abc.abstractmethod
    async def disconnect(self):
        """ Disconnect from a sensor. Should handle any socket clearing etc."""
        pass

    async def reconnect(self):
        await self.disconnect()
        await self.connect(*self.connect_args, **self.connect_kwargs)


class SensorPlugin(MetaPlugin):
    """ This plugin handles loading and management of sensor plugins.

    Only supports two CLI commands:

    * ``load``: Load a new sensor, optionally with a custom name to have multiple sensors of the same type
    * ``status``: Log information about currently loaded sensors, by calling
      :py:meth:`Sensor.status() <dronemanager.plugins.sensor.Sensor.status>`
    """

    EXAMPLE_DIR: pathlib.Path = SRC_DIR.joinpath("sensors")
    """Directory in the source tree with example modules.

    :meta hide-value:"""

    USER_DIR: pathlib.Path = DM_INSTALL_DIR.joinpath("sensors")
    """Directory in the DroneManager install directory where the sub-plugins should be located.

    :meta hide-value:"""

    VALID_CLASS_SUFFIX: str = "Sensor"
    """Valid sub-plugins must have class names ending with this string.

    :meta hide-value:"""

    NAMESPACE: str = "sensors"
    """Modules with sub-plugins have this prepended to their import to reduce collisions.

    :meta hide-value:"""

    SUBTYPE: type = Sensor
    """The type that sub-plugins must subclass to be valid.

    :meta hide-value:"""

    PREFIX: str = "sensor"
    """ PREFIX: (class attribute) The prefix for the CLI commands, "sensor" by default."""

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "load": self.load,
            "status": self.status,
        }

    async def status(self):
        """ Status of loaded sensors and sensors that could be loaded."""
        self.logger.info("Status of loaded sensors:")
        for sensor in self._loaded:
            await getattr(self, sensor).status()
        if len(self._loaded) == 0:
            self.logger.info("No loaded sensors!")
        self.logger.info(f"Available sensors for loading: {self.plugin_options()}")
