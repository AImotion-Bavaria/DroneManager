import asyncio
import datetime
import inspect
import os
import socket
import sys
import json
import copy
from pathlib import Path
import typing
import importlib
from collections.abc import Collection
from asyncio.exceptions import TimeoutError, CancelledError

from dronemanager.drone import Drone, parse_address, DroneConfigs, DroneConfig
from dronemanager.navigation.rectlocalfence import RectLocalFence
from dronemanager.utils import COMMON_FORMATTER, get_free_port, LOG_DIR
from dronemanager.navigation.core import Waypoint, Fence
from dronemanager.plugin import Plugin

import logging


CONFIG_FILE = Path(__file__).parent.parent.parent.joinpath("config.json")

# TODO: Fence class discovery
FENCES = {
    "localrect": RectLocalFence,
}

# TODO: Trajectory generator/follower discovery and setting/unsetting functions, trajectory follower deactivation

pane_formatter = logging.Formatter('%(asctime)s %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")


class DMConfig:
    """
    Configuration class for DroneManager

    """

    CORE_ENTRIES = ["drones", "mav_system_id", "mav_component_id", "plugin_settings", "default_plugins", "testing"]
    # These must be present

    def __init__(self, drone_configs: DroneConfigs, mav_system_id: int = 246, mav_component_id: int = 190,
                 default_plugins: list[str] | None = None, testing: bool = False,
                 plugin_settings: dict[str, dict[str, typing.Any]] | None = None, **kwargs):
        self.drone_configs: DroneConfigs = drone_configs
        self.mav_system_id: int = mav_system_id
        self.mav_component_id: int = mav_component_id
        self.testing = testing
        if default_plugins is None:
            default_plugins = []
        self.default_plugins = default_plugins
        if plugin_settings is None:
            plugin_settings = {}
        self.plugin_settings: dict[str, dict[str, typing.Any]] = plugin_settings

        # Load any extra attributes
        for kwarg in kwargs:
            if hasattr(self, kwarg):
                raise RuntimeError(f"Attribute collision from entry {kwarg} in configuration file!")
            self.__setattr__(kwarg, kwargs[kwarg])

    @classmethod
    def from_file(cls, filepath: str):
        with open(filepath, "rt") as f:
            configs = []
            json_obj = json.load(f)
            for obj_dict in json_obj["drones"]:
                configs.append(DroneConfig(**obj_dict))
            configs = DroneConfigs(configs)
            mav_sys_id = json_obj["mav_system_id"]
            mav_comp_id = json_obj["mav_component_id"]
            testing = json_obj["testing"]
            if "plugin_settings" in json_obj:
                plugin_settings = json_obj["plugin_settings"]
            if "default_plugins" in json_obj:
                default_plugins = json_obj["default_plugins"]
            extra_entries = {}
            for key in json_obj:
                if key not in cls.CORE_ENTRIES:
                    extra_entries[key] = json_obj[key]
        return cls(configs, mav_system_id=mav_sys_id, mav_component_id=mav_comp_id, plugin_settings=plugin_settings,
                   default_plugins=default_plugins, testing=testing, **extra_entries)

    def to_file(self, filepath: str):
        with open(filepath, "wt") as f:
            outdict = self.__dict__.copy()
            outdict.pop("drone_configs")
            outdict["drones"] = [self.drone_configs[drone].__dict__ for drone in self.drone_configs]
            json.dump(outdict, f, indent=2)


class DroneManager:
    """
    Core class of the library.

    """
    # TODO: Handle MAVSDK crashes - Not sure at all what causes them
    # TODO: Refactor functions other than fly_to to also use the list wrapping convenience

    def __init__(self, drone_class, logger=None, log_to_console=True, console_log_level=logging.INFO):
        self.drone_class = drone_class
        self.drones: dict[str, Drone] = {}
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self.drone_lock = asyncio.Lock()

        self._on_drone_removal_coros = set()
        self._on_drone_connect_coros = set()

        self._on_plugin_load_coros = set()
        self._on_plugin_unload_coros = set()
        self.plugins: set[str] = set()

        self.config = DMConfig.from_file(CONFIG_FILE.as_posix())
        self.drone_configs = self.config.drone_configs

        self.system_id = self.config.mav_system_id
        self.component_id = self.config.mav_component_id

        if logger is None:
            self.logger = logging.getLogger("Manager")
            self.logger.setLevel(logging.DEBUG)
            filename = f"manager_{datetime.datetime.now()}"
            filename = filename.replace(":", "_").replace(".", "_") + ".log"
            os.makedirs(LOG_DIR, exist_ok=True)
            file_handler = logging.FileHandler(os.path.join(LOG_DIR, filename))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(COMMON_FORMATTER)
            self.logger.addHandler(file_handler)
        else:
            self.logger = logger

        if log_to_console:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(console_log_level)
            console_handler.setFormatter(COMMON_FORMATTER)
            self.logger.addHandler(console_handler)

    async def connect_to_drone(self,
                               name: str,
                               mavsdk_server_address: str | None = None,
                               mavsdk_server_port: int | None = None,
                               drone_address: str| None = None,
                               timeout: float = 30,
                               telemetry_frequency: float | None = None,
                               log_telemetry = True):
        if drone_address is None:
            drone_address = self.drone_configs["default"].address
        try:
            scheme, parsed_addr, parsed_port = parse_address(string=drone_address)
        except Exception as e:
            self.logger.warning("Couldn't connect due to an exception parsing the address")
            self.logger.debug(repr(e), exc_info=True)
            return False
        if scheme == "serial":
            self.logger.info(f"Queuing connection to drone {name} @{scheme}://{parsed_addr} with baud {parsed_port}")
        else:
            self.logger.info(f"Queuing connection to drone {name} @{scheme}://{parsed_addr}:{parsed_port}")
        drone = None
        async with self.drone_lock:
            try:
                # Ensure that for each drone there is a one-to-one-to-one relation between name, mavsdk port and drone
                if name in self.drones:
                    self.logger.warning(f"A drone called {name} already exists. Each drone must have a unique name.")
                    return False
                if not mavsdk_server_address:
                    mavsdk_server_port = get_free_port()
                # Check that we don't already have this drone connected.
                for other_name in self.drones:
                    other_drone = self.drones[other_name]
                    _, other_addr, other_port = parse_address(string=other_drone.drone_addr)
                    if parsed_addr == other_addr and parsed_port == other_port:
                        self.logger.warning(f"{other_name} is already connected to drone with address {drone_address}.")
                        return False

                config = copy.deepcopy(self.drone_configs["default"])
                specific_config: DroneConfig = self.drone_configs[name]
                if specific_config:
                    self.logger.debug("Found drone config, using parameters...")
                    for key, value in specific_config.__dict__.items():
                        setattr(config, key, value)
                if telemetry_frequency is not None:
                    if telemetry_frequency < 2:
                        self.logger.error("Can't have a telemetry rate of less than 2!")
                    else:
                        config.position_rate = telemetry_frequency
                drone = self.drone_class(name, mavsdk_server_address, mavsdk_server_port, config=config)
                connected = None
                try:
                    if scheme == "serial":
                        self.logger.info(f"Trying to connect to drone {name} @{scheme}://{parsed_addr} with baud {parsed_port}")
                    else:
                        self.logger.info(f"Trying to connect to drone {name} @{scheme}://{parsed_addr}:{parsed_port}")
                    connected = await asyncio.wait_for(drone.connect(drone_address, system_id=self.system_id,
                                                                     component_id=self.component_id,
                                                                     log_telemetry=log_telemetry),
                                                       timeout)
                except (CancelledError, TimeoutError, OSError, socket.gaierror, AssertionError) as e:
                    if isinstance(e, CancelledError):
                        self.logger.info(f"Aborting connection attempt to {name}")
                    elif isinstance(e, TimeoutError):
                        self.logger.warning(f"Connection attempts to {name} timed out!")
                    elif isinstance(e, OSError) or isinstance(e, socket.gaierror):
                        self.logger.error(f"Address error, probably due to invalid address")
                        self.logger.debug(f"{repr(e)}", exc_info=True)
                    else:
                        self.logger.error("Connection failed due to an exception")
                        self.logger.debug(f"{repr(e)}", exc_info=True)
                    if isinstance(connected, asyncio.Task):
                        connected.cancel()
                    await self._remove_drone_object(name, drone)
                    return False
                if connected:
                    self.logger.info(f"Connected to {name}!")
                    self.drones[name] = drone
                    for func in self._on_drone_connect_coros:
                        try:
                            await asyncio.create_task(func(name, drone))
                        except Exception as e:
                            self.logger.error(f"Failed post-connection process: {repr(e)}")
                            self.logger.debug(repr(e), exc_info=True)
                            await self._remove_drone_object(name, drone)
                            return False
                    return True
                else:
                    self.logger.warning(f"Failed to connect to drone {name}!")
                    await self._remove_drone_object(name, drone)
                    return False
            except TimeoutError:
                self.logger.warning(f"Connection attempts to {name} timed out!")
                await self._remove_drone_object(name, drone)
                return False
            except Exception as e:
                self.logger.error("Couldn't connect to the drone due to an exception: ", repr(e))
                self.logger.debug(repr(e), exc_info=True)
                if drone is not None:
                    await self._remove_drone_object(name, drone)
                return False

    async def disconnect(self, names: str | Collection[str], force=False):
        if isinstance(names, str):
            names = [names]
        self.logger.info(f"Disconnecting {names} ...")
        async with self.drone_lock:
            for name in list(names):
                try:
                    drone = self.drones[name]
                except KeyError:
                    continue
                try:
                    disconnected = await drone.disconnect(force=force)
                except Exception as e:
                    disconnected = False
                    self.logger.error(f"An error occurred during disconnect for {name}")
                    self.logger.debug(repr(e), exc_info=True)
                if disconnected:
                    await self._remove_drone_object(name, drone)
                    self.logger.info(f"Disconnected {name}")

    async def _single_drone_action(self, action, name, start_string, *args, schedule=False, **kwargs):
        return await self._multiple_drone_action(action, [name], start_string, *args, schedule=schedule, **kwargs)

    async def _multiple_drone_action(self, action, names: str | Collection[str], start_string, *args, schedule=False, **kwargs):
        # Convenience check to avoid isues when using multiple drone functions with only a single drone.
        if isinstance(names, str):
            names = [names]
        try:
            coros = [action(self.drones[name], *args, **kwargs) for name in names]
            if schedule:
                self.logger.info("Queuing action: " + start_string.format(names))
                results = [self.drones[name].schedule_task(coros[i]) for i, name in enumerate(names)]
            else:
                self.logger.info(start_string.format(names))
                results = [self.drones[name].execute_task(coros[i]) for i, name in enumerate(names)]
            results = await asyncio.gather(*results, return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    self.logger.error(f"Drone {names[i]} failed due to: {str(result)}")
            return results
        except KeyError:
            self.logger.warning(f"No drones named {[name for name in names if name not in self.drones]}!")
        except Exception as e:
            self.logger.error(repr(e))
            self.logger.debug(repr(e), exc_info=True)

    async def _multiple_drone_multiple_params_action(self, action, names: str | Collection[str],
                                                     start_string: str, *args,
                                                     schedule: bool = False, **kwargs):
        # If we only want to control a single drone, we allow "raw" arguments without an enclosing list or array as a
        # convenience, i.e. fly_to("luke", [x, y, z]) instead of fly_to(["luke], [[x, y, z]])
        try:
            wrap_args = False
            if isinstance(names, str):
                wrap_args = True
                names = [names]
            n_drones = len(names)
            # Assign args and kwargs to specific drones
            drone_args = {name: [] for name in names}
            drone_kwargs = {name: {} for name in names}
            for arg in args:
                if wrap_args:
                    arg = [arg]
                assert len(arg) == n_drones, "Size mismatch between an argument and the number of drones!"
                for j, drone_arg in enumerate(arg):
                    drone_args[names[j]].append(drone_arg)
            for kwarg in kwargs:
                value = kwargs[kwarg]
                if wrap_args:
                    value = [value]
                assert len(value) == n_drones, "Size mismatch between an argument and the number of drones!"
                for j, drone_value in enumerate(value):
                    drone_kwargs[names[j]][kwarg] = drone_value

            coros = [action(self.drones[name], *drone_args[name], **drone_kwargs[name]) for name in names]
            if schedule:
                self.logger.info("Queuing action: " + start_string.format(names))
                results = [self.drones[name].schedule_task(coros[i]) for i, name in enumerate(names)]
            else:
                self.logger.info(start_string.format(names))
                results = [self.drones[name].execute_task(coros[i]) for i, name in enumerate(names)]
            results = await asyncio.gather(*results, return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    self.logger.error(f"Drone {names[i]} failed due to: {repr(result)}")
            return results
        except KeyError:
            self.logger.warning(f"No drones named {[name for name in names if name not in self.drones]}!")
        except Exception as e:
            self.logger.error("Encountered an exception! See the log for details.")
            self.logger.debug(repr(e), exc_info=True)

    async def arm(self, names: str | Collection[str], schedule=False):
        return await self._multiple_drone_action(self.drone_class.arm, names,
                                                 "Arming drone(s) {}.", schedule=schedule)

    async def disarm(self, names: str | Collection[str], schedule=False):
        return await self._multiple_drone_action(self.drone_class.disarm, names,
                                                 "Disarming drone(s) {}.", schedule=schedule)

    async def takeoff(self, names: str | Collection[str], altitude=2.0, schedule=False, allow_in_air=True):
        return await self._multiple_drone_action(self.drone_class.takeoff, names,
                                                 "Takeoff for Drone(s) {}.", altitude, schedule=schedule, allow_in_air=allow_in_air)

    async def change_flightmode(self, names: str | Collection[str], flightmode: str, schedule=False):
        await self._multiple_drone_action(self.drone_class.change_flight_mode,
                                          names,
                                          "Changing flightmode for drone(s) {} to " + flightmode + ".",
                                          flightmode, schedule=schedule)

    async def land(self, names: str | Collection[str], schedule=False):
        await self._multiple_drone_action(self.drone_class.land, names,
                                          "Landing drone(s) {}.", schedule=schedule)

    def set_fence(self, names: str | Collection[str], fence: Fence):
        """ Set a fence on drones"""
        if isinstance(names, str):
            names = [names]
        try:
            for name in names:
                try:
                    self.drones[name].fence = fence
                    self.logger.info(f"Set fence {self.drones[name].fence} on {name}")
                except KeyError:
                    self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error("Couldn't set fence due to an exception")
            self.logger.debug(repr(e), exc_info=True)

    def pause(self, names: str | Collection[str]):
        if isinstance(names, str):
            names = [names]
        self.logger.info(f"Pausing drone(s) {names}")
        for name in names:
            self.drones[name].pause()

    def resume(self, names: str | Collection[str]):
        if isinstance(names, str):
            names = [names]
        self.logger.info(f"Resuming task execution for drone(s) {names}")
        for name in names:
            self.drones[name].resume()

    async def yaw_to(self, names: str | Collection[str], yaw: Collection[float] | float,
                     yaw_rate: Collection[float] | float | None = None, local: Collection[float] | None = None,
                     tol: float | Collection[float] = 2, schedule: bool = True):
        n_drones = 1 if isinstance(names, str) else len(names)
        if isinstance(tol, float) and n_drones > 1:
            tol = [tol for _ in range(n_drones)]
        if isinstance(yaw, float) and n_drones > 1:
            yaw = [yaw for _ in range(n_drones)]
        if isinstance(yaw_rate, float) and n_drones > 1:
            yaw_rate = [yaw_rate for _ in range(n_drones)]
        return await self._multiple_drone_multiple_params_action(self.drone_class.yaw_to, names,
                                                                 f"Yawing drones {names}", yaw,
                                                                 schedule=schedule, yaw_rate=yaw_rate, tolerance=tol,
                                                                 local=local)

    async def fly_to(self, names: str | Collection[str], local: Collection[float] | None = None,
                     gps: Collection[float] | None = None, waypoint: list[Waypoint] | None = None,
                     yaw: Collection[float] | float | None = None, tol: float | Collection[float] = 0.25, schedule=True):
        assert local is not None or gps is not None or waypoint is not None, ("Must provide either waypoints, gps or "
                                                                              "local coordinates!")
        # Maybe allow for single args and then duplicate those for all drones?
        n_drones = 1 if isinstance(names, str) else len(names)
        if isinstance(tol, float) and n_drones > 1:
            tol = [tol for _ in range(n_drones)]
        return await self._multiple_drone_multiple_params_action(self.drone_class.fly_to, names,
                                                                 f"Moving drones {names}", schedule=schedule,
                                                                 tolerance=tol, local=local, gps=gps, waypoint=waypoint,
                                                                 yaw=yaw)

    async def move(self, names: str | Collection[str], offset: Collection[float], yaw: Collection[float] | float | None = None,
                   use_gps: bool | Collection[bool] = True, tol: float | Collection[float] = 0.25,
                   schedule: bool = True):
        """ Move the drones by offsets meters from their current positions. Which coordinate system is used depends on
        no_gps.

        :param names:
        :param offset: An array with the offsets for the drones. Should contain the number of meters to move in NED.
        :param yaw:
        :param use_gps: If False, use the local coordinate system, otherwise use GPS.
        :param tol:
        :param schedule:
        :return:
        """
        n_drones = 1 if isinstance(names, str) else len(names)
        if isinstance(tol, float) and n_drones > 1:
            tol = [tol for _ in range(n_drones)]
        if isinstance(use_gps, bool) and n_drones > 1:
            use_gps = [use_gps for _ in range(n_drones)]
        return await self._multiple_drone_multiple_params_action(self.drone_class.move, names,
                                                                 f"Moving drones {names}", offset,
                                                                 schedule=schedule, yaw=yaw, use_gps=use_gps,
                                                                 tolerance=tol)

    async def wait(self, names: str | Collection[str], delay: float | Collection[float], schedule=True):
        return await self._multiple_drone_multiple_params_action(self.drone_class.wait, names,
                                                                 f"Drones {names} waiting", delay,
                                                                 schedule=schedule)

    async def orbit(self, name, radius, velocity, center_lat, center_long, amsl):
        try:
            await self.drones[name].orbit(radius, velocity, center_lat, center_long, amsl)
            self.logger.info(f"{name} flying in a circle.")
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))

    async def go_to(self, names: str | Collection[str], local: Collection[float] | None = None,
                     gps: Collection[float] | None = None, waypoint: list[Waypoint] | None = None,
                     yaw: Collection[float] | float | None = None, tol: float | Collection[float] = 0.25, schedule=True):
        """ Note that this uses the GO TO Mavlink command instead of offboard mode."""
        assert local is not None or gps is not None or waypoint is not None, ("Must provide either waypoints, gps or "
                                                                              "local coordinates!")
        # Maybe allow for single args and then duplicate those for all drones?
        n_drones = 1 if isinstance(names, str) else len(names)
        if isinstance(tol, float) and n_drones > 1:
            tol = [tol for _ in range(n_drones)]
        return await self._multiple_drone_multiple_params_action(self.drone_class.go_to, names,
                                                                 f"Moving drones {names}", schedule=schedule,
                                                                 tolerance=tol, local=local, gps=gps, waypoint=waypoint,
                                                                 yaw=yaw)

    async def action_stop(self, names):
        if not names:
            self.logger.info("Stopping all drones!")
        else:
            self.logger.info(f"Stopping {names}")
        drones_to_stop = names if names else list(self.drones.keys())
        results = await asyncio.gather(*[self._stop_drone(name) for name in drones_to_stop], return_exceptions=True)
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                self.logger.critical(f"During stopping, drone {drones_to_stop[i]} encountered an exception "
                                     f"{repr(result)}!", exc_info=True)
        return results

    async def kill(self, names):
        if not names:
            self.logger.info("Killing all drones!")
        else:
            self.logger.info(f"Killing {names}")
        drones_to_stop = names if names else list(self.drones.keys())
        results = await asyncio.gather(*[self._kill_drone(name) for name in drones_to_stop], return_exceptions=True)
        return results

    async def _stop_drone(self, name):
        try:
            drone = self.drones[name]
            result = await drone.stop()
            return result
        except KeyError:
            pass

    async def _kill_drone(self, name):
        try:
            drone = self.drones[name]
            result = await drone.kill()
            return result
        except KeyError:
            pass

    async def _remove_drone_object(self, name, drone: Drone):
        try:
            self.drones.pop(name)
        except KeyError:
            pass
        for func in self._on_drone_removal_coros:
            try:
                await asyncio.create_task(func(name))
            except Exception as e:
                self.logger.debug(repr(e), exc_info=True)
        if drone is not None:
            await drone.stop_execution()
            del drone

    def add_remove_func(self, func):
        self._on_drone_removal_coros.add(func)

    def add_connect_func(self, func):
        self._on_drone_connect_coros.add(func)

    async def close(self):
        for plugin in list(self.plugins):
            await self.unload_plugin(plugin)
        await self.disconnect(self.drones)

# PLUGINS ##############################################################################################################

    def plugin_options(self):
        # Go through every file in plugins folder
        _base_dir = Path(__file__).parent
        _plugin_dir = _base_dir.joinpath("plugins")
        modules = [name.stem for name in _plugin_dir.iterdir()
                   if name.is_file() and name.suffix == ".py" and not name.stem.startswith("_")]
        return modules

    def _get_plugin_class(self, module) -> type[Plugin]:
        try:
            plugin_mod = importlib.import_module("." + module, "dronemanager.plugins")
            plugin_classes = [member[1] for member in inspect.getmembers(plugin_mod, inspect.isclass)
                              if issubclass(member[1], Plugin) and not member[1] is Plugin  # Strict subclass check
                              and member[1].__name__.endswith("Plugin")]  # Only load plugins
            if len(plugin_classes) != 1:
                raise RuntimeWarning(f"Too many plugin classes in the module {module}!")
            return plugin_classes[0]
        except ImportError as e:
            self.logger.error(f"Couldn't load plugin {module} due to a python import error!")
            self.logger.debug(repr(e), exc_info=True)

    def currently_loaded_plugins(self):
        return self.plugins

    def add_plugin_load_func(self, func):
        self._on_plugin_load_coros.add(func)

    def add_plugin_unload_func(self, func):
        self._on_drone_connect_coros.add(func)

    async def load_plugin(self, plugin_module: str, plugin_name: str | None = None, options: list[str] | None = None,
                          class_getter: callable = None):
        plugin = None
        if plugin_name is None:
            plugin_name = plugin_module
        if options is None:
            options = self.plugin_options()
        if class_getter is None:
            class_getter = self._get_plugin_class
        try:
            # Basic checks that we can even try to load this plugin
            if hasattr(self, plugin_name):
                raise RuntimeError(f"Can't load plugin {plugin_module} with name {plugin_name} due to possible name "
                                   f"collision with an existing attribute! Rename the plugin.")
            if plugin_name in self.plugins:
                self.logger.warning(f"Plugin {plugin_name} already loaded!")
                return False
            if plugin_module not in options:
                self.logger.warning(f"No plugin '{plugin_name}' found!")
                return False

            self.logger.info(f"Loading plugin {plugin_name}...")
            plugin = None
            try:
                plugin_class = await asyncio.get_running_loop().run_in_executor(None, class_getter, plugin_module)
                if not plugin_class:
                    self.logger.error(f"Module {plugin_name} contains no or multiple plugins, which is currently not "
                                      f"supported!")
                    return False
                for dependency in plugin_class.DEPENDENCIES:
                    deps = dependency.split(".")
                    if len(deps) == 2:
                        dep1, dep2 = deps
                        plugin1 = await self.load_plugin(dep1)
                        subplugin = await plugin1.load(dep2)
                    elif len(deps) == 1:
                        if dependency not in self.plugins:
                            await self.load_plugin(dependency)
                    else:
                        self.logger.warning("Nested dependencies are only supported to the first level, i.e. one dot.")
                if plugin_name in self.config.plugin_settings:
                    kwargs = self.config.plugin_settings[plugin_name]
                else:
                    kwargs = {}
                plugin = plugin_class(self, self.logger, plugin_name, **kwargs)
                setattr(self, plugin_name, plugin)
                self.plugins.add(plugin_name)
                await plugin.start()
            except Exception as e:
                self.logger.error(f"Couldn't load plugin {plugin_name} due to an exception!")
                self.logger.debug(repr(e), exc_info=True)
                if plugin is not None:
                    await plugin.close()
                if hasattr(self, plugin_name):
                    delattr(self, plugin_name)
                if plugin_name in self.plugins:
                    self.plugins.remove(plugin_name)
                return False
            self.logger.debug(f"Performing callbacks for plugin loading...")
            for func in self._on_plugin_load_coros:
                res = await asyncio.create_task(func(plugin_name, plugin))
                if isinstance(res, Exception):
                    self.logger.warning("Couldn't perform a callback for this plugin!")
            self.logger.info(f"Completed loading Plugin {plugin_name}!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)
        return plugin

    async def unload_plugin(self, plugin_name):
        if plugin_name not in self.plugins:
            self.logger.warning(f"No plugin named {plugin_name} loaded!")
            return False
        self.logger.info(f"Unloading plugin {plugin_name}")
        self.plugins.remove(plugin_name)
        plugin = getattr(self, plugin_name)
        unload_tasks = set()
        for func in self._on_plugin_unload_coros:
            unload_tasks.add(func(plugin_name, plugin))
        await asyncio.gather(*unload_tasks, return_exceptions=True)
        await plugin.close()
        delattr(self, plugin_name)
        return True
