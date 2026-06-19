import asyncio
import logging

import numpy as np
from abc import ABC, abstractmethod
from enum import Enum, auto

from dronemanager.utils import dist_ned, relative_gps, heading_ned, heading_gps, offset_from_gps
import dronemanager


class WayPointType(Enum):
    # Type of setpoint              # Expected data structure
    POS_NED = auto()                # array [pos_n, pos_e, pos_d, yaw]
    POS_VEL_NED = auto()            # array [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, yaw]
    POS_VEL_ACC_NED = auto()        # array [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, yaw]
    VEL_NED = auto()                # array [vel_n, vel_e, vel_d, yaw]
    VEL_BODY = auto()
    POS_GLOBAL = auto()             # array [lat, long, amsl, yaw]


class Waypoint:
    def __init__(self, waypoint_type: WayPointType,
                 pos: np.ndarray | None = None,
                 vel: np.ndarray | None = None,
                 acc: np.ndarray | None = None,
                 gps: np.ndarray | None = None,
                 yaw: float | None = None,
                 yaw_rate: float | None = None,):
        # Internal data structure, form [x, y, z, xvel, yvel, zvel, xacc, yacc, zacc, lat, long, amsl, yaw, yaw_rate]
        self._array: np.ndarray = np.empty((14,), dtype=np.float64)

        self._array[:3] = pos
        self._array[3:6] = vel
        self._array[6:9] = acc
        self._array[9:12] = gps
        self._array[12] = yaw
        self._array[13] = yaw_rate
        self.type = waypoint_type

    def __str__(self):
        return f"{self.type}, {self.pos}, {self.vel}, {self.acc}, {self.gps}, {self.yaw}, {self.yaw_rate}"

    @property
    def pos(self):
        return self._array[:3]

    @pos.setter
    def pos(self, new_pos: np.ndarray):
        self._array[:3] = new_pos

    @property
    def vel(self):
        return self._array[3:6]

    @vel.setter
    def vel(self, new_vel: np.ndarray):
        self._array[3:6] = new_vel

    @property
    def acc(self):
        return self._array[6:9]

    @acc.setter
    def acc(self, new_acc: np.ndarray):
        self._array[6:9] = new_acc

    @property
    def gps(self):
        return self._array[9:12]

    @gps.setter
    def gps(self, new_gps: np.ndarray):
        self._array[9:12] = new_gps

    @property
    def yaw(self):
        return self._array[12]

    @yaw.setter
    def yaw(self, new_yaw: float):
        self._array[12] = new_yaw

    @property
    def yaw_rate(self):
        return self._array[13]

    @yaw_rate.setter
    def yaw_rate(self, new_yaw_rate: float):
        self._array[13] = new_yaw_rate

    def distance(self, other: "Waypoint"):
        return dist_ned(self.pos, other.pos)

    def heading_ned(self, other: "Waypoint"):
        return heading_ned(self.pos, other.pos)

    def heading_gps(self, other: "Waypoint"):
        return heading_gps(self.gps, other.gps)

    def shift_gps(self, north: float, east: float, down: float) -> "Waypoint":
        """Returns a new waypoint, offset by north, east and down from this waypoint. Note that the yaw is kept."""
        new_gps = relative_gps(self.gps, [north, east, down])
        return Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray(new_gps), yaw=self.yaw)

    def offset_gps(self, initial: "Waypoint", target: "Waypoint"):
        """ Creates a vector between two waypoints and then creates a new Waypoint offset from this
        waypoint by the same distance and heading."""
        new_gps = offset_from_gps(self.gps, initial.gps, target.gps)
        return Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray(new_gps), yaw=self.yaw)


class Fence(ABC):
    """ Abstract base class for geo-fence type classes and methods.

    """
    def __init__(self, *args, **kwargs):
        self.logger = logging.getLogger("Manager.Fence")
        self.active = True

    @abstractmethod
    def check_waypoint_compatible(self, point: Waypoint) -> bool:
        """ Should return True if the waypoint fits within the fence, and false otherwise.

        "Fits within" is taken broadly here, a fence can be inclusive, exclusive, around a dynamic obstacle, or
         anything else. As long as True is returned when the waypoint is "good" and False otherwise, it works."""
        pass

    @abstractmethod
    def controller_safety(self, drone, forward, right, down, yaw, *args, **kwargs):
        """ This function should adjust the controller inputs to prevent the drone from exceeding the fence.

        Inputs are in the body frame of the drone. This function must be fast, expect it to be called at up to 100 Hz.

        Args:
            drone:
            forward:
            right:
            down:
            yaw:
            *args:
            **kwargs:

        Returns:
            The adjusted inputs as a tuple (forward, right, down, yaw)
        """
        pass

    @property
    @abstractmethod
    def bounding_box(self) -> np.ndarray:
        """ Should return an axis aligned bounding box for other components to use.

        Output array should have shape (6,) and contain the limits as [north_lower, north_upper, east_lower,
        east_upper, down_lower, down_upper].
        """
        pass


class PathGenerator(ABC):
    """
    Abstract base class for path generators.

    """

    CAN_DO_GPS = False
    WAYPOINT_TYPES = set()
    """ These determine the type of intermediate waypoints a path generator may produce"""

    def __init__(self, drone: "dronemanager.drone.Drone", logger, waypoint_type):
        """

        Args:
            drone:
            logger:
            waypoint_type:
        """
        assert waypoint_type in self.WAYPOINT_TYPES, (f"Invalid waypoint type {waypoint_type} "
                                                      f"for path generator {self.__class__.__name__}")
        self.drone = drone
        self.logger = logger
        self.waypoint_type = waypoint_type
        self.target_position: Waypoint | None = None

    def set_target(self, waypoint: Waypoint):
        """ Sets the target position that we will try to fly towards.
        """
        self.target_position = waypoint

    @abstractmethod
    async def create_path(self) -> bool:
        """ Function that performs whatever calculations are initially necessary to be able to produce waypoints.
        This function may be quite slow. """

    @abstractmethod
    def next(self) -> Waypoint:
        """ The next waypoint, once the follower algorithm asks for another one.

        This function must execute quickly, as it might be called with a high frequency during flight. Path
        followers should call it when they think they have reached the current waypoint. Should return None if the
        path generator hasn't produced any waypoints or has run out."""


class PathFollower(ABC):
    """ Abstract Base class to "follow" a given path and maintain position at waypoints.

    A path follower can work with different types of waypoints, but must be able to process WayPoinType.POS_NED,
    as that is the default case when a generator isn't providing waypoints.
    """

    CAN_DO_GPS = False
    SETPOINT_TYPES = set()
    WAYPOINT_TYPES = set()

    def __init__(self, drone: "dronemanager.drone.Drone", logger, dt, setpoint_type: WayPointType):
        assert setpoint_type in self.SETPOINT_TYPES, (f"Invalid setpoint type {setpoint_type} "
                                                      f"for path follower {self.__class__.__name__}")
        assert setpoint_type in drone.VALID_SETPOINT_TYPES, (f"Invalid setpoint type {setpoint_type} "
                                                             f"for drone {drone.__class__.__name__}")
        self.logger = logger
        self.drone = drone
        self.setpoint_type = setpoint_type
        self.dt = dt  # How often to send setpoints to the FC
        self.current_waypoint: Waypoint | None = None
        self._active = False
        self._following_task: asyncio.Coroutine | None = None
        self._is_waypoint_new = False

    def activate(self):
        if not self._active:
            self._active = True
            self._following_task = asyncio.create_task(self.follow())
        else:
            self.logger.debug("Can't activate path follower, it is already active.")

    async def deactivate(self):
        if self._active:
            try:
                self.logger.debug("Path follower deactivating...")
                self._active = False
                await self._following_task
                self._following_task = None
                self.current_waypoint = None
                self._is_waypoint_new = False
            except Exception as e:
                self.logger.error(repr(e), exc_info=True)
        else:
            self.logger.debug("Can't deactivate path follower, because it isn't active.")

    @property
    def is_active(self):
        return self._active

    async def follow(self):
        """ Follows waypoints produced from a path generator by sending setpoints to the drone FC.

        Requests a new waypoint from the PathGenerator when get_next_waypoint returns True. If the PG does not produce
        a waypoint, holds position instead.

        :return:
        """
        # Use current position as dummy waypoint in case og bugs in get_next_waypoint function or similar
        dummy_waypoint = Waypoint(WayPointType.POS_NED, pos=self.drone.position_ned,
                                  vel=np.zeros((3,)), yaw=self.drone.attitude[2])
        have_waypoints = False
        using_dummy_waypoint = False
        waypoint = dummy_waypoint
        while self.is_active:
            try:
                if self.get_next_waypoint():
                    #self.logger.debug("Getting new waypoint from path generator...")
                    waypoint = self.drone.path_generator.next()
                    if not waypoint:
                        if not using_dummy_waypoint:
                            if have_waypoints: # Had waypoints, but the generator isn't producing any new ones.
                                self.logger.debug("Generator no longer producing waypoints, using old waypoint")
                                dummy_waypoint = self.current_waypoint
                            else:  # Never had a waypoint
                                self.logger.debug(f"Don't have any waypoints from the generator yet, using current position: {self.drone.position_ned}")
                                dummy_waypoint = Waypoint(WayPointType.POS_NED, pos=self.drone.position_ned,
                                                          yaw=self.drone.attitude[2])
                                self._is_waypoint_new = True
                            waypoint = dummy_waypoint
                            using_dummy_waypoint = True
                        else:
                            #self.logger.debug("Still using current position...")
                            waypoint = dummy_waypoint
                        have_waypoints = False
                    else:
                        self._is_waypoint_new = True
                        have_waypoints = True
                        using_dummy_waypoint = False
                        self.logger.debug(f"Follower got waypoint: {waypoint}")
                    self.current_waypoint = waypoint
                await self.set_setpoint(waypoint)
                self._is_waypoint_new = False
                await asyncio.sleep(self.dt)
            except Exception as e:
                self.logger.error("Encountered an exception during following algorithm:", repr(e))
                self.logger.debug(repr(e), exc_info=True)

    @abstractmethod
    def get_next_waypoint(self) -> bool:
        """ Function that determines when to get the next waypoint from the path generator.

        PathGenerator.next() is called during the follow loop when this function returns True. It should always
        return True if we don't have a waypoint already."""

    @abstractmethod
    async def set_setpoint(self, waypoint):
        """ Function that determines the next setpoint required to get to the target waypoint. This function is called
        once every dt seconds using either the next waypoint from the path generator or the drones current
        position.

        :return:
        """

    def close(self):
        if self._following_task:
            self._following_task.cancel()
