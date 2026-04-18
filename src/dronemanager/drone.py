import asyncio
import datetime
from collections import deque
import math
import os.path
import threading
import platform
import time
from subprocess import Popen, DEVNULL
from abc import ABC, abstractmethod
from typing import Coroutine

import numpy as np

from mavsdk import System
from mavsdk.telemetry import FlightMode as MAVSDKFlightMode
from mavsdk.telemetry import FixType as MAVSDKFixType
from mavsdk.telemetry import StatusTextType
from mavsdk.action import ActionError, OrbitYawBehavior
from mavsdk.offboard import PositionNedYaw, PositionGlobalYaw, VelocityNedYaw, AccelerationNed, OffboardError, \
    VelocityBodyYawspeed
from mavsdk.manual_control import ManualControlError

from dronemanager.utils import dist_ned, dist_gps, relative_gps, coroutine_awaiter
from dronemanager.utils import parse_address, COMMON_FORMATTER, get_free_port
from dronemanager.utils import LOG_DIR
from dronemanager.mavpassthrough import MAVPassthrough
from dronemanager.navigation.core import WayPointType, Waypoint, PathGenerator, PathFollower, Fence
from dronemanager.navigation.directtargetgenerator import DirectTargetGenerator
from dronemanager.navigation.gmp3generator import GMP3Generator
from dronemanager.navigation.ruckigfollower import RuckigOfflineFollower

import logging

_cur_dir = os.path.dirname(os.path.abspath(__file__))
_mav_server_file = f'"{os.path.join(_cur_dir, "mavsdk_server_bin.exe")}"'


# TODO: Separate activate/deactivate for follower algorithm, currently can only be activated by move/flyto and cannot
#  be deactivated manually at all.
# TODO: Have a look at the entire connection procedure, make some diagrams, plan everything out and refactor any
#  issues nicely
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

    def __init__(self, name, *args, log_to_file=True, config: DroneConfig | None = None, **kwargs):
        threading.Thread.__init__(self)
        self.name = name
        self.drone_addr = None
        self.drone_ip = None
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
        self.log_to_file = log_to_file
        if self.log_to_file:
            log_file_name = f"drone_{self.name}_{datetime.datetime.now()}"
            log_file_name = log_file_name.replace(":", "_").replace(".", "_") + ".log"
            os.makedirs(LOG_DIR, exist_ok=True)
            file_handler = logging.FileHandler(os.path.join(LOG_DIR, log_file_name))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(COMMON_FORMATTER)
            self.add_handler(file_handler)

        self.position_update_rate: float = 10
        self.fence: Fence | None = None
        self.path_generator: PathGenerator | None = None
        self.path_follower: PathFollower | None = None

        self.return_position: Waypoint | None = None  # Position of the drone immediately after takeoff

        self.is_paused = False
        self.mav_conn: MAVPassthrough | None = None
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

    @abstractmethod
    async def stop_execution(self):
        """ Stops the thread. This function should be called at the end of any implementing function.

        :return:
        """
        self.should_stop.set()

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
    def autopilot(self) -> str:
        return self.mav_conn.drone_autopilot

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
    async def connect(self, drone_addr, *args, **kwargs):
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


class DroneMAVSDK(Drone):

    VALID_FLIGHTMODES = {"hold", "offboard", "return", "land", "takeoff", "position", "altitude"}
    # This attribute is for checking which flight modes can be changed into manually
    VALID_SETPOINT_TYPES = {WayPointType.POS_NED,
                            WayPointType.POS_VEL_NED,
                            WayPointType.POS_VEL_ACC_NED,
                            WayPointType.VEL_NED,
                            WayPointType.POS_GLOBAL}
    # What type of path setpoints this classes fly_<> commands can follow. This limits what Trajectory generators
    # can be used.

    def __init__(self, name, mavsdk_server_address: str | None = None, mavsdk_server_port: int = 50051,
                 config: DroneConfig | None = None):
        # TODO: Currently exceptions in this block are not logged to drone manager, as the handler is added only after
        #  connecting.
        super().__init__(name, config=config)
        self.system: System | None = None
        self.server_addr = mavsdk_server_address
        self.server_port = mavsdk_server_port
        self.gcs_system_id = None
        self.gcs_component_id = None
        self.drone_system_id = None           # Populated during connection process
        self.drone_component_id = None
        self._server_process: Popen | None = None
        self._is_connected: bool = False
        self._is_armed: bool = False
        self._flightmode: FlightMode = FlightMode.UNKNOWN
        self._in_air: bool = False
        self._gps_info: FixType | None = None
        self._position_g: np.ndarray = np.zeros((4,))  # Latitude, Longitude, AMSL, Relative altitude to takeoff
        self._position_ned: np.ndarray = np.zeros((3,))     # NED
        self._velocity: np.ndarray = np.zeros((3,))         # NED
        self._attitude: np.ndarray = np.zeros((3,))         # Roll, pitch and yaw, with positives right up and right.
        self._heading: float = math.nan
        self._batteries: dict[int, Battery] = {}
        self._running_tasks = set()

        # How often (per second) we request position information from the drone. The same interval is used by path
        # planning algorithms for their time resolution.
        self.position_update_rate = config.position_rate

        self.mav_conn: MAVPassthrough = MAVPassthrough(loggername=f"{name}_MAVLINK", log_messages=self.config.position_rate)

        # Init path generator
        try:
            #self.path_generator = GMP3Generator(self, 1/self.position_update_rate, self.logger)
            self.path_generator = DirectTargetGenerator(self, self.logger, WayPointType.POS_NED)
        except Exception as e:
            self.logger.error("Couldn't initialize path generator due to an exception!")
            self.logger.debug(repr(e), exc_info=True)

        # Init path follower
        try:
            self.path_follower = RuckigOfflineFollower(self, self.logger, 1 / self.position_update_rate,
                                                       WayPointType.POS_VEL_ACC_NED,
                                                       max_vel=self.config.max_h_vel,
                                                       max_down_vel=self.config.max_down_vel,
                                                       max_up_vel=self.config.max_up_vel, max_acc=self.config.max_h_acc,
                                                       max_v_acc=self.config.max_v_acc, max_jerk=self.config.max_h_jerk,
                                                       max_v_jerk=self.config.max_v_jerk,
                                                       max_yaw_vel=self.config.max_yaw_vel,
                                                       max_yaw_acc=self.config.max_yaw_acc,
                                                       max_yaw_jerk=self.config.max_yaw_jerk)
            #self.path_follower = DirectSetpointFollower(self, self.logger, 1/self.position_update_rate,
            #                                                  WayPointType.POS_VEL_NED)
        except Exception as e:
            self.logger.error("Couldn't initialize path follower due to an exception!")
            self.logger.debug(repr(e), exc_info=True)

        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized Drone {self.name}, {self.__class__.__name__}:\n   {attr_string}")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_armed(self) -> bool:
        return self._is_armed

    @property
    def flightmode(self) -> FlightMode:
        return self._flightmode

    @property
    def in_air(self) -> bool:
        return self._in_air

    @property
    def fix_type(self) -> FixType:
        return self._gps_info

    @property
    def position_global(self) -> np.ndarray:
        return self._position_g[:3]

    @property
    def altitude_above_takeoff(self) -> float:
        return self._position_g[3]

    @property
    def position_ned(self) -> np.ndarray:
        return self._position_ned

    @property
    def velocity(self) -> np.ndarray:
        return self._velocity

    @property
    def speed(self) -> float:
        return np.linalg.norm(self._velocity, 2)

    @property
    def attitude(self) -> np.ndarray:
        return self._attitude

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def batteries(self) -> dict[int, Battery]:
        return self._batteries

    async def connect(self, drone_address, system_id=0, component_id=0, log_telemetry=None) -> bool:
        # If we are on windows, we can't rely on the MAVSDK to have the binary installed.
        # If we use serial, loc is the path and appendix the baudrate, if we use udp it is IP and port
        self.gcs_system_id = system_id
        self.gcs_component_id = component_id
        scheme, loc, appendix = parse_address(string=drone_address)
        self.drone_addr = f"{scheme}://{loc}:{appendix}"
        self.logger.debug(f"Connecting to drone {self.name} @ {self.drone_addr}")
        if self.mav_conn:
            mavsdk_passthrough_port = get_free_port()
            mavsdk_passthrough_string = f"udp://:{mavsdk_passthrough_port}"
            passthrough_gcs_string = f"127.0.0.1:{mavsdk_passthrough_port}"
        else:
            if scheme == "serial":
                mavsdk_passthrough_string = f"serial://{loc}"
            else:
                mavsdk_passthrough_string = f"{scheme}://:{appendix}"

        if log_telemetry is None:
            log_telemetry = self.config.log_telemetry

        try:
            if self.server_addr is None:
                self.logger.debug(f"Starting up own MAVSDK Server instance with app port {self.server_port} and remote "
                                  f"connection {mavsdk_passthrough_string}")
            if self.server_addr is None and platform.system() == "Windows":
                try:
                    self.logger.debug(f"On windows, using local server file {_mav_server_file}")
                    self._server_process = Popen(f"{_mav_server_file} -p {self.server_port} "
                                                 f"{mavsdk_passthrough_string}", stdout=DEVNULL, stderr=DEVNULL)
                    # TODO: Come up with some way of capturing the output that actually works
                    # Things tried:
                    # - Asyncio subprocess and async for loops over output - Didn't consistently get console output,
                    #   randomly broke the program such that connections stopped working entirely
                    # - Using a separate thread that polled the output - Didn't consistently get console output, issue
                    #   is buffering on the subprocess side that seem unavoidable with this approach
                    # - Redirecting our stdout - Didn't work at all for the mavsdk server output and was very
                    #   inconsistent for other outputs. Seems like each script might need to do its own redirecting,
                    #   which is obviously not practical.
                    self.server_addr = "127.0.0.1"
                except FileNotFoundError:
                    self.logger.error("Missing the MAVSDK server binary! This must be downloaded manually on Windows, "
                                      "see the documentation.")
                    return False
            self.system = System(mavsdk_server_address=self.server_addr, port=self.server_port,
                                 sysid=system_id, compid=component_id)

            connected = asyncio.create_task(self.system.connect(system_address=mavsdk_passthrough_string))
            self._running_tasks.add(connected)

            # Create passthrough
            if self.mav_conn:
                self.mav_conn.log_messages = log_telemetry
                # Wait to try and make sure that the mavsdk server has started before booting up passthrough
                await asyncio.sleep(0.5)
                self.logger.debug(
                    f"Connecting passthrough to drone @{loc}:{appendix} and MAVSDK server @{passthrough_gcs_string}")
                self.mav_conn.connect_drone(loc, appendix, scheme=scheme)
                self.mav_conn.connect_gcs(passthrough_gcs_string)

                while not self.mav_conn.connected_to_drone() or not self.mav_conn.connected_to_gcs():
                    self.logger.debug(f"Waiting on passthrough to connect. "
                                      f"Drone: {self.mav_conn.connected_to_drone()}, "
                                      f"GCS: {self.mav_conn.connected_to_gcs()}")
                    await asyncio.sleep(0.1)
                self.logger.debug("Connected passthrough!")
                self.drone_system_id = self.mav_conn.drone_system
                self.drone_component_id = self.mav_conn.drone_component

            await connected

            async for state in self.system.core.connection_state():
                if state.is_connected:
                    await self._configure_message_rates()
                    await self._schedule_update_tasks()
                    self.logger.debug("Connected!")
                    self.config.address = self.drone_addr
                    param_load_task = asyncio.create_task(self.load_parameters())
                    param_load_task_awaiter = asyncio.create_task(coroutine_awaiter(param_load_task, self.logger))
                    self._running_tasks.add(param_load_task)
                    self._running_tasks.add(param_load_task_awaiter)
                    return True
        except Exception as e:
            self.logger.debug(f"Exception during connection: {repr(e)}", exc_info=True)
        return False

    async def load_parameters(self):
        self.logger.info(f"Loading parameters...")
        try:
            parameters = await self.system.param.get_all_params()
            raw_params = {}
            for param in parameters.int_params:
                raw_params[param.name] = (param.value, int)
            for param in parameters.float_params:
                raw_params[param.name] = (param.value, float)
            for param in parameters.custom_params:
                raw_params[param.name] = (param.value, str)
            drone_params = DroneParams(raw_params)
            if self.autopilot == "PX4":
                drone_params.max_h_vel = drone_params.raw['MPC_XY_VEL_MAX'][0]
                drone_params.max_up_vel = drone_params.raw['MPC_Z_VEL_MAX_UP'][0]
                drone_params.max_down_vel = drone_params.raw['MPC_Z_VEL_MAX_DN'][0]
                drone_params.max_yaw_rate = drone_params.raw['MPC_MAN_Y_MAX'][0]
            elif self.autopilot == "Ardupilot":
                drone_params.max_h_vel = drone_params.raw['LOIT_SPEED'][0] * 10  # cm/s
                drone_params.max_up_vel = drone_params.raw['PILOT_SPEED_UP'][0] * 10
                drone_params.max_down_vel = drone_params.raw['PILOT_SPEED_DN'][0] * 10
                drone_params.max_yaw_rate = drone_params.raw['PILOT_Y_RATE'][0]  # degrees per second
                if drone_params.max_down_vel == 0:
                    drone_params.max_down_vel = drone_params.max_up_vel
            else:
                self.logger.warning("Couldn't parse parameters for this autopilot, drone speeds might"
                                    "not work properly.")
            self.drone_params = drone_params
            self.logger.info(f"Loaded parameters!")
        except Exception as e:
            self.logger.warning("Couldn't load parameters!")
            self.logger.debug(repr(e), exc_info=True)
        # TODO: Should probably do some kind of parameter checking: If the drone velocity and acceleration parameters are
        #  lower than the ones set in our config, algorithms might not work properly

    async def disconnect(self, force=False):
        self.clear_queue()
        self.cancel_action()
        if force or not self._is_armed:
            if force:
                self.logger.debug("Force disconnecting from drone...")
            await self.stop_execution()
            return True
        else:
            self.logger.warning("Can't disconnect from an armed drone!")
            return False

    async def _schedule_update_tasks(self) -> None:
        self._running_tasks.add(asyncio.create_task(self._connect_check()))
        self._running_tasks.add(asyncio.create_task(self._arm_check()))
        self._running_tasks.add(asyncio.create_task(self._flightmode_check()))
        self._running_tasks.add(asyncio.create_task(self._inair_check()))
        self._running_tasks.add(asyncio.create_task(self._gps_check()))
        self._running_tasks.add(asyncio.create_task(self._g_pos_check()))
        self._running_tasks.add(asyncio.create_task(self._vel_rpos_check()))
        self._running_tasks.add(asyncio.create_task(self._att_check()))
        self._running_tasks.add(asyncio.create_task(self._battery_check()))
        self._running_tasks.add(asyncio.create_task(self._status_check()))
        self._running_tasks.add(asyncio.create_task(self._ensure_message_rates()))

    async def _configure_message_rates(self) -> None:
        if self.is_connected:
            try:
                await self.system.telemetry.set_rate_position(self.position_update_rate)
                await self.system.telemetry.set_rate_position_velocity_ned(self.position_update_rate)
                await self.system.telemetry.set_rate_attitude_euler(self.position_update_rate)
                await self.system.telemetry.set_rate_altitude(self.position_update_rate)
                await self.system.telemetry.set_rate_battery(self.position_update_rate)
                await self.system.telemetry.set_rate_gps_info(self.position_update_rate)
            except Exception as e:
                self.logger.warning(f"Couldn't set message rate!")
                self.logger.debug(f"{repr(e)}", exc_info=True)

    async def _ensure_message_rates(self):
        # Send our desired message rates every so often to ensure they are adhered to
        while True:
            if self.is_connected:
                await self._configure_message_rates()
            await asyncio.sleep(5)

    async def _connect_check(self):
        if self.mav_conn:
            while True:
                self._is_connected = self.mav_conn.connected_to_drone() and self.mav_conn.connected_to_gcs()
                await asyncio.sleep(1 / self.position_update_rate)
        else:
            async for state in self.system.core.connection_state():
                self._is_connected = state.is_connected

    async def _arm_check(self):
        async for arm in self.system.telemetry.armed():
            self._is_armed = arm

    async def _flightmode_check(self):
        async for flightmode in self.system.telemetry.flight_mode():
            self._flightmode = flightmode

    async def _inair_check(self):
        async for in_air in self.system.telemetry.in_air():
            self._in_air = in_air

    async def _gps_check(self):
        async for gps in self.system.telemetry.gps_info():
            self._gps_info = gps.fix_type

    async def _g_pos_check(self):
        async for pos in self.system.telemetry.position():
            self._position_g[0] = pos.latitude_deg
            self._position_g[1] = pos.longitude_deg
            self._position_g[2] = pos.absolute_altitude_m
            self._position_g[3] = pos.relative_altitude_m

    async def _vel_rpos_check(self):
        async for pos_vel in self.system.telemetry.position_velocity_ned():
            self._velocity[0] = pos_vel.velocity.north_m_s
            self._velocity[1] = pos_vel.velocity.east_m_s
            self._velocity[2] = pos_vel.velocity.down_m_s
            self._position_ned[0] = pos_vel.position.north_m
            self._position_ned[1] = pos_vel.position.east_m
            self._position_ned[2] = pos_vel.position.down_m

    async def _att_check(self):
        async for att in self.system.telemetry.attitude_euler():
            self._attitude[0] = att.roll_deg
            self._attitude[1] = att.pitch_deg
            self._attitude[2] = att.yaw_deg

    async def _heading_check(self):
        async for heading in self.system.telemetry.heading():
            self._heading = heading.heading_deg

    async def _battery_check(self):
        async for battery in self.system.telemetry.battery():
            battery_id = battery.id
            if battery_id in self._batteries:
                own_battery = self._batteries[battery_id]
            else:
                own_battery = Battery()
                own_battery.id = battery_id
                self._batteries[battery_id] = own_battery
            own_battery.consumed = battery.capacity_consumed_ah
            own_battery.remaining = battery.remaining_percent
            own_battery.voltage = battery.voltage_v
            own_battery.temperature = battery.temperature_degc

    async def _status_check(self):
        async for message in self.system.telemetry.status_text():
            if message.type is StatusTextType.DEBUG:
                self.logger.debug(f"{message.text}")
            elif message.type in [StatusTextType.INFO, StatusTextType.NOTICE]:
                self.logger.info(f"{message.text}")
            elif message.type is StatusTextType.WARNING:
                self.logger.warning(f"{message.text}")
            else:
                self.logger.error(f"{message.text}")

    async def arm(self):
        timeout = 5
        self.logger.info("Arming!")
        await super().arm()
        result = await self._error_wrapper(self.system.action.arm, ActionError)
        if result and not isinstance(result, Exception):
            start_time = time.time()
            while not self.is_armed:
                await asyncio.sleep(1 / self.position_update_rate)
                if time.time() - start_time > timeout:
                    self.logger.warning("Arming timed out!")
                    return False
            self.logger.info("Armed!")
        else:
            self.logger.warning("Couldn't arm!")
        return result

    async def disarm(self):
        timeout = 5
        self.logger.info("Disarming!")
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        await super().disarm()
        result = await self._error_wrapper(self.system.action.disarm, ActionError)
        if result and not isinstance(result, Exception):
            start_time = time.time()
            while self.is_armed:
                await asyncio.sleep(1 / self.position_update_rate)
                if time.time() - start_time > timeout:
                    self.logger.warning("Disarming timed out!")
                    return False
            self.logger.info("Disarmed!")
        else:
            self.logger.warning("Couldn't disarm!")
        return result

    def _can_takeoff(self):
        if not self.is_armed:
            raise RuntimeError("Can't take off without being armed!")

    async def takeoff(self, altitude=2.0, allow_in_air=True) -> bool:
        """

        :param altitude:
        :return:
        """
        if not allow_in_air and self.in_air:
            self.logger.debug("Can't takeoff again, already in the air!")
            return False
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        res =  await self._takeoff_using_offboard(altitude=altitude)
        if isinstance(res, Exception) or not res:
            return False
        self.return_position = Waypoint(WayPointType.POS_NED, pos=self.position_ned, yaw=self.attitude[2])
        return res

    def _get_pos_ned_yaw(self) -> np.ndarray:
        pos_yaw = np.zeros((4,), dtype=float)
        pos_yaw[:3] = self.position_ned
        pos_yaw[3] = self.attitude[2]
        return pos_yaw

    async def _takeoff_using_takeoffmode(self, altitude=2.0):
        """

        :param altitude: Currently ignored.
        :return:
        """
        self.logger.info("Trying to take off...")
        await super().takeoff(altitude=altitude)
        self._can_takeoff()
        result = await self._error_wrapper(self.system.action.takeoff, ActionError)
        if isinstance(result, Exception):
            self.logger.warning("Takeoff denied!")
        while self.flightmode is not FlightMode.TAKEOFF:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info(f"Taking off to {altitude}m over launch!")
        while self.flightmode is FlightMode.TAKEOFF:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Completed takeoff!")
        return True

    async def _takeoff_using_offboard(self, altitude=2.0, tolerance=0.25):
        """

        :param altitude:
        :param tolerance:
        :return:
        """
        self.logger.info(f"Trying to take off to {altitude}m in offboard mode...")
        await super().takeoff(altitude=altitude)
        self._can_takeoff()
        target_pos_yaw = self._get_pos_ned_yaw()
        target_pos_yaw[2] = target_pos_yaw[2] - altitude
        await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=target_pos_yaw[:3], yaw=target_pos_yaw[3]))
        if self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")
        self.logger.info(f"Taking off to {target_pos_yaw[2]} in local coordinates!")
        while True:
            if self.is_at_pos(target_pos_yaw, tolerance=tolerance):
                self.logger.info(f"Takeoff completed!")
                return True
            await asyncio.sleep(1 / self.position_update_rate)

    async def change_flight_mode(self, flightmode: str, timeout: float = 5):
        self.logger.info(f"Changing flight mode to {flightmode}")
        await super().change_flight_mode(flightmode)
        start_time = time.time()
        if flightmode == "hold":
            result = await self._error_wrapper(self.system.action.hold, ActionError)
            target_flight_mode = FlightMode.HOLD
        elif flightmode == "offboard":
            result = await self._error_wrapper(self.system.offboard.start, OffboardError)
            target_flight_mode = FlightMode.OFFBOARD
        elif flightmode == "return":
            result = await self._error_wrapper(self.system.action.return_to_launch, ActionError)
            target_flight_mode = FlightMode.RETURN_TO_LAUNCH
        elif flightmode == "land":
            result = await self._error_wrapper(self.system.action.land, ActionError)
            target_flight_mode = FlightMode.LAND
        elif flightmode == "takeoff":
            self._can_takeoff()
            result = await self._error_wrapper(self.system.action.takeoff, ActionError)
            target_flight_mode = FlightMode.TAKEOFF
        elif flightmode == "position":
            result = await self._error_wrapper(self.system.manual_control.start_position_control, ManualControlError)
            target_flight_mode = FlightMode.POSCTL
        elif flightmode == "altitude":
            result = await self._error_wrapper(self.system.manual_control.start_altitude_control, ManualControlError)
            target_flight_mode = FlightMode.ALTCTL
        else:
            raise KeyError(f"{flightmode} is not a valid flightmode!")
        if isinstance(result, Exception):
            self.logger.warning(f"Couldn't change flight mode due to exception {repr(result)}")
            return False
        elif not result:
            self.logger.warning("Flightmode change denied!")
            return False
        else:
            while self.flightmode != target_flight_mode and time.time() < start_time + timeout:
                await asyncio.sleep(1/self.position_update_rate)
            if self.flightmode != target_flight_mode:
                self.logger.warning("Drone accepted command, but flight mode change timed out! "
                                    "Possible connection issue.")
                return False
            else:
                self.logger.info(f"New flight mode {self.flightmode}!")
                return True

    async def set_setpoint(self, setpoint: "Waypoint"):
        setpoint_type = setpoint.type
        if setpoint_type == WayPointType.POS_NED:
            point_ned_yaw = PositionNedYaw(*setpoint.pos, setpoint.yaw)
            return await self._error_wrapper(self.system.offboard.set_position_ned, OffboardError, point_ned_yaw)
        elif setpoint_type == WayPointType.POS_VEL_NED:
            point_ned_yaw = PositionNedYaw(*setpoint.pos, setpoint.yaw)
            velocity_ned_yaw = VelocityNedYaw(*setpoint.vel, setpoint.yaw)
            return await self._error_wrapper(self.system.offboard.set_position_velocity_ned, OffboardError,
                                             point_ned_yaw, velocity_ned_yaw)
        elif setpoint_type == WayPointType.POS_VEL_ACC_NED:
            yaw = setpoint.yaw
            point_ned_yaw = PositionNedYaw(*setpoint.pos, yaw)
            velocity_ned_yaw = VelocityNedYaw(*setpoint.vel, yaw)
            acc_ned = AccelerationNed(*setpoint.acc)
            return await self._error_wrapper(self.system.offboard.set_position_velocity_acceleration_ned,
                                             OffboardError,
                                             point_ned_yaw,
                                             velocity_ned_yaw,
                                             acc_ned)
        elif setpoint_type == WayPointType.VEL_NED:
            vel_yaw = VelocityNedYaw(*setpoint.vel, setpoint.yaw)
            return await self._error_wrapper(self.system.offboard.set_velocity_ned, OffboardError, vel_yaw)
        elif setpoint_type == WayPointType.VEL_BODY:
            vel_yawrate = VelocityBodyYawspeed(*setpoint.vel, setpoint.yaw_rate)
            return await self._error_wrapper(self.system.offboard.set_velocity_body, OffboardError, vel_yawrate)
        elif setpoint_type == WayPointType.POS_GLOBAL:
            latitude, longitude, amsl = setpoint.gps
            alt_type = PositionGlobalYaw.AltitudeType.AMSL
            position = PositionGlobalYaw(lat_deg=latitude, lon_deg=longitude, alt_m=amsl,
                                         yaw_deg=setpoint.yaw, altitude_type=alt_type)
            return await self._error_wrapper(self.system.offboard.set_position_global, OffboardError, position)
        else:
            raise RuntimeError("Invalid SetPointType!")

    def _can_do_in_air_commands(self):
        # TODO: Figure out how to do this with ardupilot. Currently the in_air detection seems very poor
        # Currently not used as the in-air detection is unreliable with many of our drones.
        if self.autopilot == "ardupilot":
            return True
        if not self.is_armed or not self.in_air:
            return False
        return True

    async def yaw_to(self, target_yaw, yaw_rate=30, local=None, tolerance=2):
        """Yawing to the target heading as you do so at the specified rate, maintaining current position.

        Uses the local coordinate system for to determine and maintain position. Pausable.

        :param target_yaw: Heading as a degree fom -180 to 180, right positive, 0 forward.
        :param yaw_rate:
        :param local: Position setpoint during yaw.
        :param tolerance: How close we have to get to the heading before this function returns.
        :return:
        """
        # Add 180, take modulo 360 and subtract 180 to get proper range
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        og_yaw = self.attitude[2]
        dif_yaw = (target_yaw - og_yaw + 180) % 360 - 180
        time_required = abs(dif_yaw / yaw_rate)
        n_steps = math.ceil(time_required * self.position_update_rate)
        step_size = dif_yaw/n_steps
        if local is None:
            local = np.asarray(self.position_ned, dtype=float)
        for i in range(n_steps):
            if not self.is_paused:
                yaw = og_yaw + step_size*(i+1)
                await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=local, yaw=yaw))
            await asyncio.sleep(1 / self.position_update_rate)
        while not self.is_at_heading(target_heading=target_yaw, tolerance=tolerance):
            await asyncio.sleep(1 / self.position_update_rate)
        return True

    async def spin_at_rate(self, yaw_rate, duration, direction="cw"):
        """ Spin in place at the given rate for the given duration.

        Pausable.

        :param yaw_rate:
        :param duration:
        :param direction:
        :return:
        """
        await super().spin_at_rate(yaw_rate, duration, direction=direction)
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        pos = self.position_ned
        og_yaw = self.attitude[2]
        freq = 10
        n_steps = math.ceil(duration * freq)
        step_size = yaw_rate / freq
        if direction == "ccw":
            step_size = - step_size
        for i in range(n_steps):
            if not self.is_paused:
                yaw = og_yaw + step_size*(i+1)
                await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=pos, yaw=yaw))
            await asyncio.sleep(1/freq)

    async def fly_to(self, local: np.ndarray | None = None, gps: np.ndarray | None = None, yaw: float | None = None,
                     waypoint: Waypoint | None = None, tolerance=0.25, put_into_offboard=True, log=True):
        """ Fly to a specified point in offboard mode. Uses path generators and followers to get there.

        If multiple target are provided (for example GPS and local coordinates), we prefer coordinates in this fashion:
        Waypoint > GPS > local, i.e. in the example, the local coordinates would be ignored.

        :param local:
        :param gps:
        :param yaw:
        :param waypoint:
        :param tolerance:
        :param put_into_offboard:
        :param log:
        :return:
        """
        # Check that we have one full set of coordinates and are in a flyable state
        #if not self._can_do_in_air_commands():
        #    raise RuntimeError("Can't fly a landed or unarmed drone!")
        assert local is not None or gps is not None or waypoint is not None, \
            "Must provide a full set of either NED coordinates, GPS coordinates or a waypoint!"

        # Check that we have a path generator and follower who are compatible with each other and the drone
        assert (self.path_follower is not None
                and self.path_follower.setpoint_type in self.VALID_SETPOINT_TYPES)
        assert (self.path_generator is not None
                and self.path_generator.waypoint_type in self.path_follower.WAYPOINT_TYPES)

        # Determine target waypoint, prefering waypoint over GPS over local and using current yaw if none is provided
        if waypoint is not None:
            target = waypoint
            if waypoint.yaw is None:
                # Maintain current yaw if none given
                waypoint.yaw = self.attitude[2]
        elif gps is not None:
            if yaw is None:
                yaw = self.attitude[2]
            target = Waypoint(WayPointType.POS_GLOBAL, gps=gps, yaw=yaw)
        else:
            if yaw is None:
                yaw = self.attitude[2]
            target = Waypoint(WayPointType.POS_NED, pos=local, yaw=yaw)

        # Check that the target is valid
        assert self.check_waypoint(target), f"Invalid target position {target}, probably due to fence."

        use_gps = target.type == WayPointType.POS_GLOBAL

        if log:
            if use_gps:
                self.logger.info(f"Flying to Lat: {target.gps[0]} Long: {target.gps[1]} AMSL: {target.gps[2]} "
                                 f"facing {yaw} with tolerance {tolerance}")
            else:
                self.logger.info(f"Flying to N: {target.pos[0]} E: {target.pos[1]} D: {target.pos[2]} facing {yaw} "
                                 f"with tolerance {tolerance}")

        if put_into_offboard and self._flightmode != FlightMode.OFFBOARD:
            if use_gps:
                cur_lat, cur_long, cur_amsl = self.position_global
                cur_yaw = self.attitude[2]
                await self.set_setpoint(Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray([cur_lat, cur_long, cur_amsl]),
                                                 yaw=cur_yaw))
            else:
                cur_pos = self.position_ned
                cur_yaw = self.attitude[2]
                await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=cur_pos, yaw=cur_yaw))
            await self.change_flight_mode("offboard")

        # Check that both path generator and follower can handle GPS coordinates
        if use_gps:
            if not self.path_generator.CAN_DO_GPS:
                raise RuntimeError("Path generator can't use GPS coordinates!")
            if not self.path_follower.CAN_DO_GPS:
                raise RuntimeError("Path follower can't use GPS coordinates!")
        self.path_generator.set_target(target)

        # Create path and activate follower algorithm if not already active
        self.logger.debug("Creating path...")
        path_task = asyncio.create_task(self.path_generator.create_path())
        self.current_action_tasks.add(path_task)
        have_path = await path_task
        if not have_path:
            self.logger.warning("The path generator couldn't generate a path!")
            return False
        if not self.path_follower.is_active:
            self.logger.debug("Starting follower algorithm...")
            self.path_follower.activate()

        while True:
            # Check if we have arrived at target waypoint
            if use_gps:
                reached = (self.is_at_gps(target.gps, tolerance=tolerance)
                           and self.is_at_heading(target.yaw, tolerance=1))
            else:
                reached = (self.is_at_pos(target.pos, tolerance=tolerance) and
                           self.is_at_heading(target.yaw, tolerance=1))

            # Print message and stop if we have reached waypoint
            if reached:
                self.logger.info("Reached target position!")
                return True
            await asyncio.sleep(1 / self.position_update_rate)

    async def move(self, offset, yaw: float | None = None, use_gps=True, tolerance=0.25):
        self.logger.info("Starting move")
        target_yaw = self.attitude[2] + yaw
        if use_gps:
            target_lat, target_long, target_amsl = relative_gps(self.position_global, offset)
            waypoint = Waypoint(WayPointType.POS_GLOBAL, gps=[target_lat, target_long, target_amsl], yaw=target_yaw)
        else:
            waypoint = Waypoint(WayPointType.POS_NED, pos=self.position_ned + offset, yaw=target_yaw)
        return await self.fly_to(waypoint=waypoint, put_into_offboard=True, tolerance=tolerance)

    async def go_to(self, local: np.ndarray | None = None, gps: np.ndarray | None = None, yaw: float | None = None,
                     waypoint: Waypoint | None = None, tolerance=0.25,):
        assert local is not None or gps is not None or waypoint is not None, \
            "Must provide a full set of either NED coordinates, GPS coordinates or a waypoint!"

        # If flightmode is not Hold, goto hold
        if self.flightmode != FlightMode.HOLD:
            self.logger.info("Changing flightmode to hold")
            await self.change_flight_mode("hold")

        if waypoint is not None:
            if waypoint.type is WayPointType.POS_GLOBAL:
                gps = waypoint.gps
            else:
                offset = waypoint.pos - self.position_ned
                gps = relative_gps(self.position_global, offset)
            yaw = waypoint.yaw
        elif gps is not None:
            pass
        elif local is not None:
            offset = local - self.position_ned
            gps = relative_gps(self.position_global, offset)

        # Send goto command
        await self.system.action.goto_location(*gps, yaw)

        while True:
            # Check if we have arrived at target waypoint
            reached = (self.is_at_gps(gps, tolerance=tolerance) and self.is_at_heading(yaw, tolerance=1))

            # Print message and stop if we have reached waypoint
            if reached:
                self.logger.info("Reached target position!")
                return True
            await asyncio.sleep(1 / self.position_update_rate)

    async def orbit(self, radius, velocity, center_lat, center_long, amsl):
        await super().orbit(radius, velocity, center_lat, center_long, amsl)
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        yaw_behaviour = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER
        await self._error_wrapper(self.system.action.do_orbit, ActionError, radius, velocity, yaw_behaviour,
                                  center_lat, center_long, amsl)

    async def land(self):
        self.logger.info("Trying to land...")
        if self.path_follower.is_active:
            await self.path_follower.deactivate()
        await super().land()
        return await self._land_using_offbord_mode()

    async def _land_using_offbord_mode(self, error_thresh=0.00001, min_time=1):
        self.logger.info("Landing!")
        ema_alt_error = 0
        going_down = True
        old_alt = self.position_ned[2]
        start_time = time.time()
        target_pos = self._get_pos_ned_yaw()
        if self._flightmode != FlightMode.OFFBOARD:
            await self.set_setpoint(Waypoint(WayPointType.POS_VEL_NED, pos=target_pos[:3], yaw=target_pos[3]))
            await self.change_flight_mode("offboard")
        update_freq = 2
        try:
            await self.system.telemetry.set_rate_position_velocity_ned(self.position_update_rate)
            update_freq = self.position_update_rate
        except Exception as e:
            self.logger.debug(f"Couldn't set message rate: {repr(e)}", exc_info=True)
        while going_down:
            cur_alt = self.position_ned[2]
            ema_alt_error = (cur_alt - old_alt) + 0.33 * ema_alt_error
            if ema_alt_error < error_thresh and time.time() > start_time + min_time:
                going_down = False
            old_alt = cur_alt
            target_pos[2] = cur_alt + 0.5
            await self.set_setpoint(Waypoint(WayPointType.POS_VEL_NED, pos=target_pos[:3], vel=[0, 0, 0.3], yaw=target_pos[3]))
            await asyncio.sleep(1/update_freq)
        self.logger.info("Landed!")
        return True

    async def _land_using_landmode(self):
        result = await self._error_wrapper(self.system.action.land, ActionError)
        if isinstance(result, Exception):
            self.logger.warning("Couldn't go into land mode")
        while self.flightmode is not FlightMode.LAND:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Landing!")
        while self.in_air:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Landed!")
        await self.change_flight_mode("hold")
        return True

    async def manual_control_position(self):
        self.clear_queue()
        self.cancel_action()
        await self.path_follower.deactivate()
        result = await self._error_wrapper(self.system.manual_control.start_position_control, ManualControlError)
        return result

    async def manual_control_altitude(self):
        self.clear_queue()
        self.cancel_action()
        await self.path_follower.deactivate()
        result = await self._error_wrapper(self.system.manual_control.start_altitude_control, ManualControlError)
        return result

    async def set_manual_control_input(self, x, y, z, r):
        result = await self._error_wrapper(self.system.manual_control.set_manual_control_input, ManualControlError, x, y, z, r)
        return result

    async def stop_execution(self):
        """ Stops all coroutines, closes all connections, etc.

        :return:
        """
        if self.path_follower:
            if self.path_follower.is_active:
                await self.path_follower.deactivate()
            self.path_follower.close()
        try:
            if self.mav_conn:
                await self.mav_conn.stop()
                del self.mav_conn
        except AttributeError:
            pass
        if self.system is not None:
            self.system.__del__()
        if self._server_process:
            self._server_process.terminate()
        for handler in self.logging_handlers:
            self.logger.removeHandler(handler)
        await super().stop_execution()

    async def stop(self):
        # Override whatever else is going on and land
        self.clear_queue()
        self.cancel_action()
        if not self.is_connected:
            return True
        await self.land()
        await self.disarm()
        await super().stop()
        return True

    async def kill(self):
        await self._error_wrapper(self.system.action.kill, ActionError)
        await super().kill()
        return True

    async def _error_wrapper(self, func, error_type, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except error_type as e:
            self.logger.error(e._result.result_str)
            return False
        return True
