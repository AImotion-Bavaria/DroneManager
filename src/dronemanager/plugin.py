"""Class for extra, loadable plugins.

Plugins extend the functionality of DroneManager or Drone Classes by providing extra functions. They can also register
their own commands to the CLI.
"""
import abc
import asyncio
from concurrent.futures import Future
import importlib.util
import inspect
import logging
import pathlib
import sys
from typing import Callable, Coroutine

import dronemanager.core
from dronemanager.utils import DM_INSTALL_DIR, SRC_DIR, cancel_running_tasks


# TODO: Figure out scheduling
#   Have to interact with drone queues ("Move to position X, then turn gimbal, then move to position Y)
#   BUT, also want to perform plugin actions immediately mid flight without killing other drone tasks, (except if we do)
# TODO: Figure out how to do help strings for plugins, choices, store_true, etc... general CLI information.

class Plugin(abc.ABC):
    """ Generic plugin class.

    The attribute :py:attr:`cli_commands` is called by the DroneManager CLI (and could be called by other UIs) to
    populate their interfaces. This is a dictionary with coroutines as values and human-readable names as keys. In
    DroneManager the names are used together with the class prefix to determine the command input on the command line,
    while the signature of the function is used to populate the CLI parser.

    The attribute :py:attr:`background_functions` should list coroutines that will be launched during initialization
    and run indefinitely, for example those polling for status updates from a camera. They will be started during
    construction of the class object, usually when the module is loaded. Note that these must be coroutines.

    The attribute :py:attr:`running_tasks` keeps track of any awaitables, such as tasks, currently running. These are
    cancelled automatically when the plugin is closed.

    There is a basic dependency structure for plugins. The attribute :py:attr:`~dronemanager.plugin.Plugin.DEPENDENCIES`
    can be used to list other plugins by their names, on which this plugin depends. These are loaded before this one is.
    The list supports a single-entry deep dot-notation, i.e. "sensor.ecowitt" specifies that we depend on the ecowitt
    plugin, which requires the sensor plugin and should be loaded using their loading functions.

    A common kwarg is "name", for plugins of which multiple copies may be loaded, in which case the name acts as the
    unique identifier.
    """

    PREFIX: str = "abc"
    """(class attribute) The prefix for the CLI commands."""
    DEPENDENCIES: list[str] = []
    """(class attribute) Other plugins that this plugin depends on."""

    def __init__(self, dm: "dronemanager.core.DroneManager", logger: logging.Logger, name: str, *args, **kwargs):
        """

        Args:
            dm:
            logger:
            name:
            *args:
            **kwargs:
        """
        self.dm: "dronemanager.core.DroneManager" = dm
        """The DroneManager instance connected to this plugin."""
        self.logger: logging.Logger = logger.getChild(self.__class__.__name__)
        """The parent logger. A child logger with the name of the class is created below this."""
        self.name: str = name
        """The name for this instance of the plugin."""
        self.cli_commands: dict[str, Callable] = {}
        """A dictionary with input strings as keys and the associated coroutines as
           values. The coroutine should be bare, i.e. ``coro`` instead of ``coro(args)``."""
        self.background_functions: list[Coroutine] = []
        """A list with coroutines which will be launched automatically once the
           plugin has loaded. These coroutines should be complete, i.e. ``coro(args)`` and not ``coro``."""
        self.running_tasks: set[Future | asyncio.Future] = set()
        """A set of awaitables currently running. These are automatically cancelled if the plugin is closed."""

    def start_background_functions(self):
        """Starts declared background functions and tracks them."""
        for coro in self.background_functions:
            self.running_tasks.add(asyncio.create_task(coro))

    async def start(self):
        """Starts the plugin.

        By default, only starts declared background functions.
        """
        self.start_background_functions()

    async def close(self):
        """Close the plugin.

        By default, stops any running functions.
        """
        cancel_running_tasks(self.running_tasks)

    @abc.abstractmethod
    async def status(self):
        """Log status information about the plugin and its attributes."""
        raise NotImplementedError


class MetaPlugin(Plugin, abc.ABC):
    """Plugin Class for plugins that define other plugin types, such as missions or sensors.

    """

    EXAMPLE_DIR: pathlib.Path | None = None
    """Directory in the source tree with shipped components."""

    USER_DIR: pathlib.Path | None = None
    """Directory in the DroneManager install directory where new sub-plugins should be located."""

    VALID_CLASS_SUFFIX: str = ""
    """Valid sub-plugins must have class names ending with this string."""

    NAMESPACE: str = ""
    """Modules with sub-plugins have this prepended to their import to reduce collisions."""

    SUBTYPE: type = Plugin
    """The type that sub-plugins must subclass to be valid."""

    ON_LOAD_COROS = set()
    ON_UNLOAD_COROS = set()

    def __init__(self, dm: "dronemanager.core.DroneManager", logger, name, *args, **kwargs):
        super().__init__(dm, logger, name, *args, **kwargs)
        self._loaded: dict[str, Plugin] = {}
        self._first_time_setup()

    @property
    def loaded(self):
        return self._loaded

    def _first_time_setup(self):
        # If we have don't have a user dir, create it. In the future, maybe also move examples there.
        #if self.USER_DIR is not None and self.EXAMPLE_DIR is not None and not self.USER_DIR.exists():
        #    self.logger.debug(f"Moving examples from {self.EXAMPLE_DIR} to {self.USER_DIR}")
        #    shutil.copytree(self.EXAMPLE_DIR, self.USER_DIR)
        # If we have a user dir, but no example dir, just create it.
        if self.USER_DIR is not None:
            self.logger.debug(f"Creating user folder {self.USER_DIR}")
            self.USER_DIR.mkdir(exist_ok=True)

    @property
    def _module_dirs_list(self):
        module_dirs = list(self.USER_DIR.iterdir())
        module_dirs.extend(list(self.EXAMPLE_DIR.iterdir()))
        return module_dirs

    def plugin_options(self):
        module_dirs = self._module_dirs_list
        # List every potential plugin file or directory in the directory
        modules = [name.stem for name in module_dirs
                   if name.is_file() and name.suffix == ".py" and not name.stem.startswith("_")]
        return modules

    def import_plugin_module(self, module: str):
        loaded_module = importlib.import_module("." + module, f"dronemanager.{self.NAMESPACE}")
        return loaded_module

    def import_user_module(self, module: str):
        user_path = self.USER_DIR.joinpath(f"{module}.py")
        module_name = f"dronemanager.{self.NAMESPACE}.user.{module}"
        spec = importlib.util.spec_from_file_location(module_name, user_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Could not create import spec for {user_path}")
        loaded_module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = loaded_module
        spec.loader.exec_module(loaded_module)
        return loaded_module

    def get_plugin_class(self, module: str) -> type[Plugin]:
        try:
            if self.EXAMPLE_DIR.joinpath(f"{module}.py").is_file():
                plugin_mod = self.import_plugin_module(module)
            else:
                plugin_mod = self.import_user_module(module)
            plugin_classes = [member[1] for member in inspect.getmembers(plugin_mod, inspect.isclass)
                              if issubclass(member[1], self.SUBTYPE)  # Must be appropriate subtype
                              and not member[1] in [self.SUBTYPE, MetaPlugin]  # Must strictly be subclass
                              and member[1].__name__.endswith(self.VALID_CLASS_SUFFIX)]  # Only consider marked classes
            if len(plugin_classes) != 1:
                raise RuntimeWarning(f"Too many or too few plugin classes in the module {module}!")
            return plugin_classes[0]
        except ImportError as e:
            self.logger.error(f"Couldn't load plugin {module} due to a python import error!")
            self.logger.debug(repr(e), exc_info=True)

    async def load(self, module: str, name: str | None = None):
        plugin = None
        if name is None:
            name = module
        try:
            # Basic checks that we can even try to load this plugin
            if hasattr(self.dm, name):
                raise RuntimeError(
                    f"Can't load plugin {module} with name {name} due to possible name "
                    f"collision with an existing attribute! Rename the plugin.")
            if name in self._loaded:
                self.logger.warning(f"Plugin {name} already loaded!")
                return False
            if module not in self.plugin_options():
                self.logger.warning(f"No plugin '{module}' found!")
                return False

            self.logger.info(f"Loading plugin {module} as {name}...")
            plugin = None
            try:
                plugin_class: type[Plugin] = self.get_plugin_class(module)
                if not plugin_class:
                    self.logger.error(
                        f"Module {module} contains no or multiple plugins, which is currently not supported!")
                    return False
                for dependency in plugin_class.DEPENDENCIES:
                    deps = dependency.split(".")
                    if len(deps) == 2:
                        metadep, dep = deps
                        if metadep not in self.dm.plugins:
                            meta_plugin = await self.dm.load(metadep)
                        else:
                            meta_plugin = getattr(self.dm, metadep)
                        await meta_plugin.load(dep)
                    elif len(deps) == 1:
                        if dependency not in self.dm.plugins:
                            await self.dm.load(dependency)
                    else:
                        self.logger.warning("Nested dependencies are only supported to the first level, i.e. one dot.")
                if name in self.dm.config.plugin_settings:
                    kwargs = self.dm.config.plugin_settings[name]
                else:
                    kwargs = {}
                plugin = plugin_class(self.dm, self.logger, name, **kwargs)
                setattr(self.dm, name, plugin)
                self._loaded[name] = plugin
                await plugin.start()
            except Exception as e:
                self.logger.error(f"Couldn't load plugin {name} due to an exception: {repr(e)}!")
                self.logger.debug(repr(e), exc_info=True)
                if plugin is not None:
                    await plugin.close()
                if hasattr(self.dm, name):
                    delattr(self.dm, name)
                if name in self._loaded:
                    self._loaded.pop(name)
                return False
            self.logger.debug(f"Performing callbacks for plugin loading...")
            for func in self.ON_LOAD_COROS:
                res = await asyncio.create_task(func(name, plugin))
                if isinstance(res, Exception):
                    self.logger.warning(f"Couldn't perform a callback {func} for this plugin due to an exception {repr(res)}!")
            self.logger.info(f"Completed loading Plugin {name}!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)
        return plugin

    async def unload(self, name: str):
        if name not in self._loaded:
            self.logger.warning(f"No loaded plugin named {name}!")
            return False
        self.logger.info(f"Unloading plugin {name}")
        plugin = self._loaded.pop(name)
        unload_tasks = set()
        for func in self.ON_UNLOAD_COROS:
            unload_tasks.add(func(name, plugin))
        await asyncio.gather(*unload_tasks, return_exceptions=True)
        await plugin.close()
        delattr(self.dm, name)
        return True

    @abc.abstractmethod
    async def status(self):
        raise NotImplementedError

    async def close(self):
        while len(self._loaded) > 0:
            for name in list(self._loaded.keys()):
                await self.unload(name)
        await super().close()


class PluginLoader(MetaPlugin):
    """Core plugin loader.

    Does not offer any CLI commands as those are covered by the app directly.
    """

    EXAMPLE_DIR: pathlib.Path = SRC_DIR.joinpath("plugins")
    """Directory in the source tree with shipped components."""

    USER_DIR: pathlib.Path = DM_INSTALL_DIR.joinpath("plugins")
    """Directory in the DroneManager install directory where new sub-plugins should be located."""

    VALID_CLASS_SUFFIX: str = "Plugin"
    """Valid sub-plugins must have class names ending with this string."""

    NAMESPACE: str = "plugins"
    """Modules with subplugins have this prepended to their import to reduce collisions."""

    SUBTYPE: type = Plugin
    """The type that subplugins must subclass to be valid."""

    async def status(self):
        """Dummy implementation."""
        pass
