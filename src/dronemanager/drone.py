import asyncio
import datetime
from collections import deque
import math
import os.path
import threading
from collections.abc import Callable
from abc import ABC, abstractmethod
from typing import Coroutine

import numpy as np

from mavsdk.asyncio.plugins.telemetry import FlightMode as MAVSDKFlightMode
from mavsdk.asyncio.plugins.telemetry import FixType as MAVSDKFixType

from dronemanager.comm_protocols.core import DroneComProtocol
from dronemanager.navigation.core import WayPointType, Waypoint, PathGenerator, PathFollower, Fence
from dronemanager.utils import dist_ned, dist_gps, cancel_running_tasks
from dronemanager.utils import COMMON_FORMATTER
from dronemanager.utils import LOG_DIR

import logging


# TODO: Separate activate/deactivate for follower algorithm, currently can only be activated by move/flyto and cannot
#  be deactivated manually at all.
# TODO: Follower/generator manager system, similar to plugin system. Should program some abstract base class for
#  plugin/missions/fences/generators/followers to use

# TODO: Allow multiple fences

FlightMode = MAVSDKFlightMode
FixType = MAVSDKFixType


class Battery:
    def __init__(self):
        self.id = None
        self.remaining = math.nan
        self.consumed = math.nan
        self.voltage = math.nan
        self.temperature = math.nan

    def __str__(self):
        return f"Remain: {self.remaining}   Consumed: {self.consumed}   V: {self.voltage}   T: {self.temperature}"


class DroneConfig:
    """ Convenience class for drone configurations.

    These exist for convenience purposes, to allow people to define drone objects with fixed parameters for easy reuse.
    They can be saved to and loaded from files. A given configuration is used when dm.connect_to_drone is called with
    the drone name matching the configuration.
    """

    def __init__(self, drone_name: str, address: str | None,
                 position_rate: float = 5.0, log_telemetry: bool = False,
                 max_h_vel: float = 10.0, max_down_vel: float = 1.0, max_up_vel: float = 3.0, max_h_acc: float = 1.5,
                 max_v_acc: float = 0.5, max_h_jerk: float = 0.5, max_v_jerk: float = 0.5, max_yaw_vel: float = 60,
                 max_yaw_acc: float = 30, max_yaw_jerk: float = 30, size: float = 1.0, rtsp: str | None = None, **kwargs):
        self.drone_name = drone_name
        self.address = address
        self.position_rate = position_rate
        self.max_h_vel = max_h_vel
        self.max_down_vel = max_down_vel
        self.max_up_vel = max_up_vel
        self.max_h_acc = max_h_acc
        self.max_v_acc = max_v_acc
        self.max_h_jerk = max_h_jerk
        self.max_v_jerk = max_v_jerk
        self.max_yaw_vel = max_yaw_vel
        self.max_yaw_acc = max_yaw_acc
        self.max_yaw_jerk = max_yaw_jerk
        self.log_telemetry = log_telemetry
        self.rtsp = rtsp  # Connection string in format "rtsp://192.168.1.31:8900/live", such as when connecting with VLC

        self.size = size
        # Size of the drone from propeller to propeller. This is used to adjust for example the fence limits, so that
        # no part of the drone exceeds the fence. It is a good idea to make this a little bit larger than the actual size.

        for kwarg, value in kwargs.items():
            self.__setattr__(kwarg, value)

    def __str__(self):
        return str(self.__dict__)


class DroneConfigs:

    def __init__(self, configs: list[DroneConfig]):
        self.configs = configs

    def __getitem__(self, item):
        for config in self.configs:
            if config.drone_name == item:
                return config
        return None

    def __setitem__(self, key: str, value: DroneConfig):
        exists = False
        for i, config in enumerate(self.configs):
            if config.drone_name == key:
                self.configs[i] = value
                exists = True
                break
        if not exists:
            self.configs.append(value)

    def __contains__(self, item):
        for config in self.configs:
            if config.drone_name == item:
                return True
        return False

    def __iter__(self):
        return [config.drone_name for config in self.configs].__iter__()

    def __len__(self):
        return len(self.configs)


class DroneParams:

    def __init__(self, raw = None):
        self.raw = raw
        self.max_h_vel = None  # m/s
        self.max_up_vel = None  # m/s
        self.max_down_vel = None  # m/s
        self.max_yaw_rate = None  # degrees/s


class Drone(ABC, threading.Thread):

    VALID_FLIGHTMODES = set()
    VALID_SETPOINT_TYPES = set()

    def __init__(self, name: str, gcs_ident, comm_protocol, *args, log_to_file=True, config: DroneConfig | None = None, **kwargs):
        threading.Thread.__init__(self)
        self.name = name
        self.gcs_ident = gcs_ident
        self.comm: DroneComProtocol = comm_protocol
        self.drone_addr = None
        self.drone_identifier = None
        self.autopilot = None
        if config:
            self.config = config
        else:
            self.config = DroneConfig(name, self.drone_addr)
        self.action_queue: deque[tuple[Coroutine, asyncio.Future]] = deque()
        self.current_action: asyncio.Task | None = None
        self.current_action_tasks: set[asyncio.Task] = set()  # TODO: Go through all functions and add any intermediates
        self.should_stop = threading.Event()
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        self.logging_handlers = []
        self.telem_logger = logging.getLogger(f"{name}_telemetry")
        self.telem_logger.setLevel(logging.DEBUG)
        self.telem_logging_handlers = []
        self.log_to_file = log_to_file
        if self.log_to_file:
            stamp = datetime.datetime.now()
            log_file_name = f"drone_{self.name}_{stamp}"
            log_file_name = log_file_name.replace(":", "_").replace(".", "_") + ".log"
            os.makedirs(LOG_DIR, exist_ok=True)
            file_handler = logging.FileHandler(os.path.join(LOG_DIR, log_file_name))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(COMMON_FORMATTER)
            self.add_handler(file_handler)

            telemetry_file_name = f"telemetry_{self.name}_{stamp}"
            telemetry_file_name = telemetry_file_name.replace(":", "_").replace(".", "_") + ".log"
            telemetry_handler = logging.FileHandler(os.path.join(LOG_DIR, telemetry_file_name))
            telemetry_handler.setLevel(logging.DEBUG)
            telemetry_handler.setFormatter(COMMON_FORMATTER)
            self.add_telemetry_handler(telemetry_handler)

        self.position_update_rate: float = 10
        self.fence: Fence | None = None
        self.path_generator: PathGenerator | None = None
        self.path_follower: PathFollower | None = None

        self.return_position: Waypoint | None = None  # Position of the drone immediately after takeoff

        self.is_paused = False
        self.drone_params: DroneParams | None = None
        self.start()
        asyncio.create_task(self._task_scheduler())

    def run(self):
        while not self.should_stop:
            pass

    async def _task_scheduler(self):
        while True:
            try:
                while len(self.action_queue) > 0:
                    if self.is_paused:
                        await asyncio.sleep(0.1)
                    else:
                        action, fut = self.action_queue.popleft()
                        self.current_action = asyncio.create_task(action)
                        try:
                            result = await self.current_action
                            fut.set_result(result)
                            self.current_action = None
                        except asyncio.CancelledError:
                            pass
                        except Exception as e:
                            fut.set_exception(e)
                        finally:
                            # If we are done but there are still subtasks running, cancel them
                            cancel_running_tasks(self.current_action_tasks)
                else:
                    await asyncio.sleep(0.1)
            except Exception as e:
                self.logger.error("Encountered an exception in the task scheduler")
                self.logger.debug(repr(e), exc_info=True)

    def schedule_task(self, coro) -> asyncio.Future:
        fut = asyncio.get_running_loop().create_future()
        self.action_queue.append((coro, fut))
        return fut

    def execute_task(self, coro) -> asyncio.Future:
        self.clear_queue()
        self.cancel_action()
        return self.schedule_task(coro)

    def add_handler(self, handler):
        self.logger.addHandler(handler)
        self.logging_handlers.append(handler)

    def add_telemetry_handler(self, handler):
        self.telem_logger.addHandler(handler)
        self.telem_logging_handlers.append(handler)

    @abstractmethod
    async def stop_execution(self):
        """ Stops the thread. This function should be called at the end of any implementing function.

        :return:
        """
        self.should_stop.set()
        for handler in self.logging_handlers:
            self.logger.removeHandler(handler)
        for handler in self.telem_logging_handlers:
            self.telem_logger.removeHandler(handler)

    def pause(self):
        """ Pause task execution by setting self.is_paused to True.

        Note that it is not possible to "pause" what the drone is doing in a general way. What "pausing" a task does or
        if a task can even be paused depends on the specific task and implementation. Subclasses must define and
        implement this behaviour themselves.
        However, pausing is always possible between tasks, and this is the default behaviour for subclasses that do not
        implement any of their own: When paused, drones will finish their current task and then wait until unpaused
        before beginning the next task."""
        self.is_paused = True
        self.logger.debug("Pausing...")

    def resume(self):
        """ Resume executing tasks. """
        self.is_paused = False
        self.logger.debug("Resuming...")

    @abstractmethod
    def add_message_callback(self, message_name: str, callback: Callable):
        """ Add callback to a specific message. The callback is executed every time a corresponding message is received.

        The callback signature should be callback(message). Note that the types of messages available can depend on the
        specific communication protocol (i.e. MAVLink).

        Args:
            message_name: The target message
            callback: The callback

        """
        pass

    @abstractmethod
    def remove_message_callback(self, message_name: str, callback: Callable):
        """ Remove a callback.

        Args:
            message_name: The target message
            callback: The callback

        """
        pass

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @property
    @abstractmethod
    def is_armed(self) -> bool:
        pass

    @property
    @abstractmethod
    def flightmode(self) -> FlightMode:
        pass

    @property
    @abstractmethod
    def in_air(self) -> bool:
        pass

    @property
    @abstractmethod
    def fix_type(self) -> FixType:
        pass

    @property
    @abstractmethod
    def position_global(self) -> np.ndarray:
        """

        :return: Array with the GPS coordinates [latitude, longitude, AMSL]
        """
        pass

    @property
    @abstractmethod
    def position_ned(self) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def velocity(self) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def speed(self) -> float:
        pass

    @property
    @abstractmethod
    def attitude(self) -> np.ndarray:
        """ RPY in degrees"""
        pass

    @property
    @abstractmethod
    def batteries(self) -> dict[int, Battery]:
        pass

    @property
    def parameters_loaded(self) -> bool:
        return self.drone_params is not None

    @abstractmethod
    async def connect(self, drone_address, *args, **kwargs):
        pass

    @abstractmethod
    async def load_parameters(self):
        pass

    @abstractmethod
    async def disconnect(self, force=False) -> bool:
        pass

    @abstractmethod
    async def arm(self) -> bool:
        pass

    @abstractmethod
    async def disarm(self) -> bool:
        pass

    @abstractmethod
    async def takeoff(self, altitude=2.0, allow_in_air=False) -> bool:
        """ Takes off to the specified altitude above current position.

        Note that altitude is positive.

        :param altitude: Takeoff altitude above.
        :return:
        """
        pass

    @abstractmethod
    async def change_flight_mode(self, flightmode) -> bool:
        pass

    def is_at_waypoint(self, waypoint: Waypoint, pos_tolerance=0.25, vel_tolerance=0.1, yaw_tolerance=1) -> bool:
        """ Definition of "is at" depends on the waypoint type. At most checks position, yaw and
        velocity.

        :param waypoint:
        :param pos_tolerance: In meters
        :param vel_tolerance: In m/s
        :param yaw_tolerance: In degrees
        :return:
        """
        if waypoint.yaw is not None:
            yaw_good = self.is_at_heading(waypoint.yaw, tolerance=yaw_tolerance)
        else:
            yaw_good = True
        if waypoint.type == WayPointType.POS_GLOBAL:
            return self.is_at_gps(waypoint.gps, tolerance=pos_tolerance) and yaw_good
        elif waypoint.type == WayPointType.POS_NED:
            return self.is_at_pos(waypoint.pos, tolerance=pos_tolerance) and yaw_good
        elif waypoint.type == WayPointType.POS_VEL_NED:
            return self.is_at_pos(waypoint.pos, tolerance=pos_tolerance) and self.is_at_vel(waypoint.vel, tolerance=vel_tolerance) and yaw_good
        elif waypoint.type == WayPointType.POS_VEL_ACC_NED:
            self.logger.debug("Asked to check a waypoint with acceleration, but no acceleration checks are implemented")
            return self.is_at_pos(waypoint.pos, tolerance=pos_tolerance) and self.is_at_vel(waypoint.vel, tolerance=vel_tolerance) and yaw_good
        elif waypoint.type == WayPointType.VEL_NED:
            return self.is_at_vel(waypoint.vel, tolerance=vel_tolerance) and yaw_good
        else:
            raise ValueError("Invalid waypoint type for this function")

    def is_at_pos(self, target_pos, tolerance=0.25) -> bool:
        """

        :param target_pos: Array with target position. If a yaw is also passed (i.e. array length 4), it is ignored.
        :param tolerance: How close we have to be to the target position to be considered "at" it.
        :return:
        """
        cur_pos = self.position_ned
        return dist_ned(cur_pos, target_pos[:3]) < tolerance

    def is_at_heading(self, target_heading, tolerance=1) -> bool:
        target_heading = (target_heading + 180) % 360 - 180
        cur_heading = self.attitude[2]
        return abs(cur_heading - target_heading) < tolerance

    def is_at_gps(self, target_gps, tolerance=0.25) -> bool:
        return dist_gps(target_gps, self.position_global) < tolerance

    def is_at_vel(self, target_vel, tolerance=0.1):
        cur_vel = self.velocity
        return dist_ned(cur_vel, target_vel) < tolerance

    @abstractmethod
    async def yaw_to(self, target_yaw, yaw_rate=30, local=None, tolerance=2):
        pass

    @abstractmethod
    async def spin_at_rate(self, yaw_rate, duration, direction="cw") -> bool:
        pass

    def set_fence(self, fence_type: type["Fence"], *args, **kwargs):
        self.fence = fence_type(self.logger, *args, **kwargs)

    def check_waypoint(self, waypoint: "Waypoint"):
        """ Check if a waypoint is valid and within any geofence (if such a fence is set)"""
        try:
            waypoint_valid = not self.fence or (self.fence and self.fence.check_waypoint_compatible(waypoint))
        except Exception as e:
            self.logger.debug(f"Invalid waypoint due to exception: {repr(e)}", exc_info=True)
            waypoint_valid = False
        return waypoint_valid

    @abstractmethod
    async def set_setpoint(self, setpoint: "Waypoint") -> bool:
        pass

    async def wait(self, delay: float):
        """ Wait delay seconds.

        This function is useful with scheduling to schedule short waits between moves."""
        await asyncio.sleep(delay)

    @abstractmethod
    async def fly_to(self, local: np.ndarray | None = None, gps: np.ndarray | None = None, yaw: float | None = None,
                     waypoint: Waypoint | None = None, tolerance=0.25):
        """ Fly to the specified position.

        :param local:
        :param gps:
        :param yaw:
        :param waypoint:
        :param tolerance:
        :return:
        """
        pass

    @abstractmethod
    async def move(self, offset: np.ndarray, yaw: float | None = None, use_gps=True, tolerance=0.25):
        """ Move from the current position by the specified distances.

        :param offset: A numpy array with the information how much to move along each axis in meters.
        :param yaw:
        :param use_gps:
        :param tolerance:
        :return:
        """
        pass

    @abstractmethod
    async def orbit(self, radius, velocity, latitude, longitude, amsl) -> bool:
        pass

    @abstractmethod
    async def land(self) -> bool:
        pass

    @abstractmethod
    async def stop(self) -> bool:
        pass

    @abstractmethod
    async def kill(self) -> bool:
        pass

    def clear_queue(self) -> None:
        """ Clears the action queue.

        Does not cancel the current action.

        :return:
        """
        self.logger.debug("Clearing action queue")
        self.action_queue.clear()

    def cancel_action(self) -> None:
        """ Cancels the current action task

        :return:
        """
        if self.current_action:
            self.logger.debug("Cancelling current action!")
            self.current_action.cancel()
            for task in self.current_action_tasks:
                if task is not None:
                    task.cancel()
            self.current_action_tasks = set()
