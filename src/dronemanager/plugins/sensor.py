""" Plugin and abstract base class for external sensors, such as weather sensors.

They're specifically for plugins that connect to and sporadically query some external data source.

Similar to missions, these are a special type of plugin with extra functions to support their intended purpose.
The core extra component is the :py:meth:`~dronemanager.plugins.sensor.Sensor.get_data` function, which should
return whatever data the sensor provides.
Like missions, sensor modules go into their own special folder ``sensors``. Each sensor module can contain exactly one
specific sensor plugin, which must subclass :py:class:`~dronemanager.plugins.sensor.Sensor` and end with "Sensor"

Note that sensors are not callback based, so data sources that should be processed continuously should not be
implemented as a sensor."""
import asyncio
import abc
import importlib
import inspect
from pathlib import Path

from dronemanager.plugin import Plugin

# TODO: A check which sensors could actually be connected. Check functions must return quick or be executed in
#  another process.


class SensorPlugin(Plugin):
    """ This plugin handles loading and management of sensor plugins.

    Only supports two CLI commands:

    * ``load``: Load a new sensor, optionally with a custom name to have multiple sensors of the same type
    * ``status``: Log information about currently loaded sensors, by calling
      :py:meth:`Sensor.status() <dronemanager.plugins.sensor.Sensor.status>`

    Attributes:
        PREFIX: (class attribute) The prefix for the CLI commands.
        sensors: A dictionary with the sensor names as keys and their associated sensor objects.
    """

    PREFIX: str = "sensor"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "load": self.load,
            "status": self.status,
        }
        self.sensors: dict[str, Sensor] = {}

    async def close(self):
        """ Stop all sensors.

        Called on DroneManager exit.
        """
        await asyncio.gather(*[sensor.close() for sensor in list(self.sensors.values())])
        await super().close()

    def sensor_options(self):
        """ Get the possible sensor files, by traversing the sensor directory.

        """
        # Go through every file in mission folder
        _base_dir = Path(__file__).parent.parent
        _plugin_dir = _base_dir.joinpath("sensors")
        modules = [name.stem for name in _plugin_dir.iterdir()
                   if name.is_file() and name.suffix == ".py" and not name.stem.startswith("_")]
        return modules

    def _get_sensor_class(self, module: str) -> None | type:
        """ Given a module name, find and return the sensor class type in that module.

        Returns None if no sensor class is found, or the module doesn't meet other requirements.

        Args:
            module: The name of the python module, without the file ending.

        Returns:
            The sensor class if one is found, ``None`` otherwise.
        """
        try:
            plugin_mod = importlib.import_module("." + module, "dronemanager.sensors")
            plugin_classes = [member[1] for member in inspect.getmembers(plugin_mod, inspect.isclass)
                              if issubclass(member[1], Sensor)
                              and not member[1] is Sensor]  # Strict subclass check
            if len(plugin_classes) != 1:
                return None
            return plugin_classes[0]
        except ImportError as e:
            self.logger.error(f"Couldn't load mission {module} due to a python import error!")
            self.logger.debug(repr(e), exc_info=True)
            return None

    async def load(self, sensor_module: str, name: str | None = None):
        """ Load a new sensor, which work like plugins with the name taking the role of the prefix.

        Args:
            sensor_module: The python module with the sensor class that should be loaded.
            name:

        Returns:

        """
        sensor = await self.dm.load_plugin(sensor_module, name, self.sensor_options(), self._get_sensor_class)
        if sensor:
            self.sensors[name] = sensor
        return sensor

    async def status(self):
        """ Status of running missions and missions that could be loaded."""
        self.logger.info("Status of currently connected sensors:")
        for sensor in self.sensors.values():
            await sensor.status()
        if len(self.sensors) == 0:
            self.logger.info("No connected sensors!")
        self.logger.info(f"Available sensor plugins for loading: {self.sensor_options()}")


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
