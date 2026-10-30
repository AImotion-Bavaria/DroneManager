import asyncio
import datetime
import os
import sys
import time
import json
import copy
import typing
from collections.abc import Collection

import traceback
from typing import Callable

import dronemanager
import dronemanager.drone
from dronemanager.comm_protocols.mav import MAVSDKComm
from dronemanager.drone import Drone, DroneComProtocol, DroneConfigs, DroneConfig
from dronemanager.navigation.core import Waypoint, Fence
from dronemanager.navigation.rectlocalfence import RectLocalFence
from dronemanager.plugin import PluginLoader
from dronemanager.utils import COMMON_FORMATTER, LOG_DIR, get_config

import logging

# TODO: Fence class discovery, probably make them plugins
FENCES = {
    "localrect": RectLocalFence,
}

# TODO: Trajectory generator/follower discovery and setting/unsetting functions, trajectory follower deactivation,
#  probably make them plugins too

pane_formatter = logging.Formatter('%(asctime)s %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")


class DMConfig:
    """
    Configuration class for DroneManager

    """

    CORE_ENTRIES = ["drones", "mav_system_id", "mav_component_id", "plugin_settings", "default_plugins", "default_comm_protocols"]
    # These must be present

    def __init__(self, drone_configs: DroneConfigs, mav_system_id: int = 246, mav_component_id: int = 190,
                 default_plugins: list[str] | None = None, default_comm_protocols: list[str] | None = None,
                 plugin_settings: dict[str, dict[str, typing.Any]] | None = None, **kwargs):
        self.drone_configs: DroneConfigs = drone_configs
        self.mav_system_id: int = mav_system_id
        self.mav_component_id: int = mav_component_id
        if default_plugins is None:
            default_plugins = []
        self.default_plugins = default_plugins
        if default_comm_protocols is None:
            default_comm_protocols = []
        self.default_protocols = default_comm_protocols
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
            if "plugin_settings" in json_obj:
                plugin_settings = json_obj["plugin_settings"]
            if "default_plugins" in json_obj:
                default_plugins = json_obj["default_plugins"]
            if "default_comm_protocols" in json_obj:
                default_protocols = json_obj["default_comm_protocols"]
            extra_entries = {}
            for key in json_obj:
                if key not in cls.CORE_ENTRIES:
                    extra_entries[key] = json_obj[key]
        return cls(configs, mav_system_id=mav_sys_id, mav_component_id=mav_comp_id, plugin_settings=plugin_settings,
                   default_plugins=default_plugins, default_comm_protocols=default_protocols, **extra_entries)

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

    def __init__(self, logger=None, log_to_console=True, console_log_level=logging.INFO):
        self.drones: dict[str, Drone] = {}
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self.drone_lock = asyncio.Lock()

        self._on_drone_removal_coros = set()
        self._on_drone_connect_coros = set()

        self.config = DMConfig.from_file(get_config().as_posix())
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

        self.comm_protocols: dict[str, DroneComProtocol] = {}

        self.plugin_loader = PluginLoader(self, self.logger, "PluginLoader")
        self.logger.info(f"Starting DroneManager version {dronemanager.__version__}")

    async def start(self):
        await asyncio.create_task(self._load_default_plugins())
        await asyncio.create_task(self._load_default_comm_protocols())

    async def _load_default_plugins(self):
        plugin_tasks = []
        for plugin_name in self.config.default_plugins:
            plugin_tasks.append(asyncio.create_task(self.load(plugin_name)))
        await asyncio.gather(*plugin_tasks)
        self.logger.info(f"Loaded startup plugins: {self.plugins}")

    async def _load_default_comm_protocols(self):
        protocol_tasks = []
        for protocol_name in self.config.default_protocols:
            protocol_tasks.append(asyncio.create_task(self.add_comm_protocol(protocol_name)))
        await asyncio.gather(*protocol_tasks)
        self.logger.info(f"Loaded default comm protocols: {self.comm_protocols}")

    async def add_comm_protocol(self, name):
        # TODO: Comm discovery? Make them plugins?
        if name == "mav":
            self.comm_protocols[MAVSDKComm.NAME] = MAVSDKComm(self.logger, system_id=self.system_id,
                                                              component_id=self.component_id)
        else:
            raise ValueError(f"No comm protocol {name}")

    async def list_potential_drones(self, comm_protocol: str):
        return await self.comm_protocols[comm_protocol].discovered_drones()

    async def list_non_drones(self, comm_protocol: str):
        return await self.comm_protocols[comm_protocol].discovered_other()

    async def add_conn(self, comm_protocol: str, conn_string: str):
        return await self.comm_protocols[comm_protocol].add_connection(conn_string)

    async def remove_conn(self, comm_protocol: str, identifier: str):
        return await self.comm_protocols[comm_protocol].remove_connection(identifier)

    async def add_drone(self, comm_protocol: str, drone_name: str, drone_identifier: str, drone_config: DroneConfig | None = None, log_telemetry: bool | None = None, telemetry_frequency: float | None = None):
        # Check for duplicate name
        self.logger.info(f"Queuing addition of {comm_protocol} drone {drone_identifier} as {drone_name}.")
        if drone_name in self.drones:
            self.logger.warning(f"A drone called {drone_name} already exists. Each drone must have a unique name.")
            return False
        # Use default or named config if none is provided as argument.
        if drone_config is None:
            drone_config = copy.deepcopy(self.drone_configs["default"])
            specific_config: DroneConfig = self.drone_configs[drone_name]
            if specific_config:
                self.logger.debug("Found drone config, using parameters...")
                for key, value in specific_config.__dict__.items():
                    setattr(drone_config, key, value)
        # Override config values if parameters are set here.
        if telemetry_frequency is not None:
            if telemetry_frequency < 2:
                self.logger.error("Can't have a telemetry rate of less than 2!")
            else:
                drone_config.position_rate = telemetry_frequency
        if log_telemetry is not None:
            drone_config.log_telemetry = log_telemetry

        async with self.drone_lock:
            self.logger.info(f"Adding {comm_protocol} drone {drone_identifier} as {drone_name}...")
            new_drone = await self.comm_protocols[comm_protocol].add_drone(drone_name, drone_identifier, drone_config)
            if new_drone is not None:
                self.drones[drone_name] = new_drone
            else:
                return False
            for func in self._on_drone_connect_coros:
                try:
                    await asyncio.create_task(func(drone_name, new_drone))
                except Exception as e:
                    self.logger.error(f"Failed post-connection process: {repr(e)}")
                    self.logger.debug(repr(e), exc_info=True)
                    await self.disconnect(drone_name, force=True)
                    return False
            self.logger.info(f"Successfully added {drone_name}!")
            return True

    async def connect_drone(self, name: str, comm_protocol: str | None = None, conn: str | None = None,
                            timeout: float = 30, telemetry_frequency: float | None = None,
                            log_telemetry: bool | None = None):
        # All the config stuff
        drone_config = copy.deepcopy(self.drone_configs["default"])
        specific_config: DroneConfig = self.drone_configs[name]
        if specific_config:
            self.logger.debug("Found drone config, using configuration...")
            for key, value in specific_config.__dict__.items():
                setattr(drone_config, key, value)
        # If a protocol is supplied, prefer that, otherwise use the one from the config (including default)
        if comm_protocol is not None:
            drone_config.protocol = comm_protocol
        else:
            comm_protocol = drone_config.protocol
        # If a connection is supplied, prefer that, otherwise use the one from config (including default)
        if conn is not None:
            drone_config.connection = conn
        else:
            conn = drone_config.connection

        # Add connection and check for drone
        old_drone_idents = [drone.drone_identifier for drone in (await self.comm_protocols[comm_protocol].discovered_drones()).copy()]
        start_time = time.time()
        await self.add_conn(comm_protocol=comm_protocol, conn_string=conn)
        self.logger.info(f"Added connection {conn} using {comm_protocol}")
        # Check every half second or so if there is a new drone
        drone_added = False
        while time.time() - start_time < timeout and not drone_added:
            await asyncio.sleep(0.5)
            new_drones = await self.comm_protocols[comm_protocol].discovered_drones()
            for new_drone in new_drones:
                if str(new_drone.drone_identifier) not in old_drone_idents:
                    # If provided name was not in config, ignore drone_identifier and use any new drone.
                    if not specific_config:
                        await self.add_drone(comm_protocol, name, new_drone.drone_identifier, drone_config=drone_config, telemetry_frequency=telemetry_frequency, log_telemetry=log_telemetry)
                        drone_added = True
                        break
                    else:
                        if str(new_drone.drone_identifier) == drone_config.drone_identifier:
                            await self.add_drone(comm_protocol, name, new_drone.drone_identifier, drone_config=drone_config, telemetry_frequency=telemetry_frequency, log_telemetry=log_telemetry)
                            drone_added = True
                            break
            # Last IDs are now the old IDs, to prevent issues if other drones were disconnected or connected while we were waiting.
            old_drone_idents = [drone.drone_identifier for drone in new_drones]
        if not drone_added:
            self.logger.warning(f"Connection attempt to {name} using {comm_protocol} at {conn} timed out!")

    async def disconnect(self, names: str | Collection[str], force=False):
        # Remove drone(s) from dronemanager.
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
                    await drone.shutdown(force=force)
                except Exception as e:
                    self.logger.error(f"An error occurred during disconnect for {name}")
                    self.logger.debug(repr(e), exc_info=True)
                else:
                    await self._remove_drone_object(name)
                    self.logger.info(f"Disconnected {name}")

    async def _multiple_drone_action(self, action, names: str | Collection[str], start_string, *args, schedule=False, **kwargs):
        # Convenience check to avoid isues when using multiple drone functions with only a single drone.
        if isinstance(names, str):
            names = [names]
        try:
            # Coroutines for each drone depending on action
            if action == "arm":
                coros = [self.drones[name].arm() for name in names]
            elif action == "disarm":
                coros = [self.drones[name].disarm() for name in names]
            elif action == "takeoff":
                coros = [self.drones[name].takeoff(*args, **kwargs) for name in names]
            elif action == "change_flight_mode":
                coros = [self.drones[name].change_flight_mode(*args, **kwargs) for name in names]
            elif action == "land":
                coros = [self.drones[name].land() for name in names]
            else:
                raise ValueError(f"Invalid action {action}")

            # Schedule actions to corresponding drones
            if schedule:
                self.logger.info("Queuing action: " + start_string.format(names))
                results = [self.drones[name].schedule_task(coros[i]) for i, name in enumerate(names)]
            else:
                self.logger.info(start_string.format(names))
                results = [self.drones[name].execute_task(coros[i]) for i, name in enumerate(names)]

            # Wait on and process results
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

            # Coroutines for each drone depending on action
            if action == "yaw_to":
                coros = [self.drones[name].yaw_to(*drone_args[name], **drone_kwargs[name]) for name in names]
            elif action == "fly_to":
                coros = [self.drones[name].fly_to(*drone_args[name], **drone_kwargs[name]) for name in names]
            elif action == "move":
                coros = [self.drones[name].move(*drone_args[name], **drone_kwargs[name]) for name in names]
            elif action == "wait":
                coros = [self.drones[name].wait(*drone_args[name], **drone_kwargs[name]) for name in names]
            else:
                raise ValueError(f"Invalid action {action}")

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
                    exc_tb = traceback.format_exception(result)
                    self.logger.debug("".join(exc_tb))
            return results
        except KeyError:
            self.logger.warning(f"No drones named {[name for name in names if name not in self.drones]}!")
        except Exception as e:
            self.logger.error("Encountered an exception! See the log for details.")
            self.logger.debug(repr(e), exc_info=True)

    async def arm(self, names: str | Collection[str], schedule=False):
        return await self._multiple_drone_action("arm", names,
                                                 "Arming drone(s) {}.", schedule=schedule)

    async def disarm(self, names: str | Collection[str], schedule=False):
        return await self._multiple_drone_action("disarm", names,
                                                 "Disarming drone(s) {}.", schedule=schedule)

    async def takeoff(self, names: str | Collection[str], altitude=2.0, schedule=False, allow_in_air=True):
        return await self._multiple_drone_action("takeoff", names,
                                                 "Takeoff for Drone(s) {}.", altitude, schedule=schedule, allow_in_air=allow_in_air)

    async def change_flightmode(self, names: str | Collection[str], flightmode: str, schedule=False):
        await self._multiple_drone_action("change_flight_mode",
                                          names,
                                          "Changing flightmode for drone(s) {} to " + flightmode + ".",
                                          flightmode, schedule=schedule)

    async def land(self, names: str | Collection[str], schedule=False):
        await self._multiple_drone_action("land", names,
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
        return await self._multiple_drone_multiple_params_action("yaw_to", names,
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
        return await self._multiple_drone_multiple_params_action("fly_to", names,
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
        return await self._multiple_drone_multiple_params_action("move", names,
                                                                 f"Moving drones {names}", offset,
                                                                 schedule=schedule, yaw=yaw, use_gps=use_gps,
                                                                 tolerance=tol)

    async def wait(self, names: str | Collection[str], delay: float | Collection[float], schedule=True):
        return await self._multiple_drone_multiple_params_action("wait", names,
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

    async def _remove_drone_object(self, name):
        try:
            self.drones.pop(name)
        except KeyError:
            pass
        for func in self._on_drone_removal_coros:
            try:
                await asyncio.create_task(func(name))
            except Exception as e:
                self.logger.debug(repr(e), exc_info=True)

    def add_remove_func(self, func):
        self._on_drone_removal_coros.add(func)

    def add_connect_func(self, func):
        self._on_drone_connect_coros.add(func)

    async def close(self):
        await self.disconnect(self.drones)
        for comm_protocol in self.comm_protocols.values():
            await comm_protocol.close()
        await self.plugin_loader.close()

# PLUGINS ##############################################################################################################

    @property
    def plugin_options(self):
        return self.plugin_loader.plugin_options()

    @property
    def plugins(self):
        return self.plugin_loader.loaded

    def add_plugin_load_coro(self, func: Callable):
        """Func should be a naked coroutine."""
        self.plugin_loader.ON_LOAD_COROS.add(func)

    def remove_plugin_load_coro(self, func: Callable):
        """Func should be a naked coroutine."""
        self.plugin_loader.ON_LOAD_COROS.remove(func)

    def add_plugin_unload_coro(self, func: Callable):
        self.plugin_loader.ON_UNLOAD_COROS.add(func)

    def remove_plugin_unload_coro(self, func: Callable):
        self.plugin_loader.ON_UNLOAD_COROS.remove(func)

    async def load(self, plugin_module: str, plugin_name: str | None = None):
        return await self.plugin_loader.load(plugin_module, plugin_name)

    async def unload_plugin(self, plugin_name: str):
        return await self.plugin_loader.unload(plugin_name)
